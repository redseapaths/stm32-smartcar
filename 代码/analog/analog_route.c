/* ============================================================
 *  hybrid_route.c —— 混合版：直行段 PID 循迹 + 转弯交给里程表
 *
 *  思路（2026-11-12 用户定的方案）：
 *    - 走多长、转多少度  → 全部由**里程表**负责（和 blind_route.c 一样）
 *    - 直行过程中的左右修正 → 交给 **PID 循迹**（八路数字量传感器）
 *
 *  为什么这么分：里程表能保证"走够距离、转到角度"，
 *  循迹 PID 只负责"别跑偏"。两者互不打架：
 *    里程 = (dl+dr)/2        转弯 = (dl-dr)/2    （符号各管各的）
 *
 *  引脚（与现有接线一致，别改）：
 *    PA0..PA7  X1..X8 八路循迹（上拉输入，压白线读 1）
 *    PA8   TIM1_CH1  PWM 右轮        PA11  TIM1_CH4  PWM 左轮
 *    PB12/PB13 右轮方向 AIN1/AIN2    PB14/PB15 左轮方向 BIN1/BIN2
 *    PB6/PB7   左编码器 A/B（TIM4 硬件四倍频）
 *    PB8/PB9   右编码器 A/B（EXTI9_5 软件四倍频）
 *    PA9/PA10  USART1 115200，8N1
 *    PC13      板载 LED（低电平点亮）
 *
 *  串口命令：
 *    g  立刻开始（不等 3 秒倒计时）
 *    c  上电 3 秒内发：进入传感器观察模式（不跑车，只看八路读数）
 *    x  紧急停车
 *    p  暂停 / 继续
 *
 *  转弯符号：turn = +90 右转（顺时针），-90 左转
 *  循迹误差：err > 0 表示线在车的右边
 * ============================================================ */

#include "stm32f10x.h"
#include <math.h>

static void delay_ms(uint32_t ms);
static void motor_set(int32_t right, int32_t left);
static void motor_stop(void);

/* ==================== 一、可调参数 ==================== */

/* 每厘米编码器计数（四倍频后）。2026-11-08 推车 100cm 标定得 2700 */
#define ENC_PER_CM      2700.0f

/* 电机方向：车往前跑时某个轮子倒转，就把对应的改成 0 */
#define RIGHT_INVERT    1
#define LEFT_INVERT     1

/* 编码器方向：手推车往前走，两个轮子的里程都应该是正数 */
#define ENC_RIGHT_SIGN  (+1)
#define ENC_LEFT_SIGN   (-1)

/* 轮距（左右轮中心距离，厘米） */
#define WHEEL_BASE_CM   16.75f

/* 转弯修正系数：用户 2026-11-12 实测定为 0.68
 * （车实际转得不够就加大，转过头就减小；新值 = 当前值 × 期望角度 / 实测角度） */
#define TURN_SCALE      0.68f

/* ---------- 速度 ---------- */
#define SPEED_LEG       420       /* 直行基准速度（v1.2.0: 340->420） */
#define SPEED_TURN      460       /* 原地转（v1.2.0: 380->460） */
#define SPEED_MIN       240       /* 快到位时降到这么慢（v1.2.0: 200->240） */
#define WHEEL_MIN       150       /* 内侧轮最低速，别让它停死被拖 */
#define DECEL_CM        8.0f      /* 差这么多厘米开始减速（v1.2.0: 5->8，提速后必须提前减速） */
/* 最后一段（turn==0 的收尾段，跑完就结束）不用再考虑转弯，直接全速冲 */
#define FINAL_SPRINT    1
#define SPEED_LAST      999       /* 最后一段的 PWM（最大 999；电池扛不住/冲过头就降到 800） */

/* ---------- 不停车转弯（v1.4.0）----------
 * 原理：到转弯点不刹车，两轮都继续保持正转，只是外轮快、内轮慢，
 *       于是车一边往前走一边画弧，弧走完正好转够角度。
 *       好处：少了"停车→反向→起步"这一下，实测转角比原地转更准。
 *       代价：转弯过程中车在前进，所以必须**提前**开始转（TURN_ROLL_ADV）。
 */
#define TURN_ROLL        0        /* 1 = 不停车边走边转，0 = 老方案（停下原地转）
                                   * v1.4.2: 1->0，滚转半径太大经常丢线，退回停车原地转
                                   *          （低延时参数沿用 v1.2.0，每弯固定等待 360ms） */
#define TURN_ROLL_R      20.0f    /* 弧线半径（到转弯中心，cm）：12=急 20=中 25=平缓 */
#define TURN_ROLL_PWM    320      /* 转弯时的线速度 PWM（内轮=这值×(1-B/2R)） */
#define TURN_ROLL_ADV    10.0f    /* 提前量 cm：直行段提前这么多就开始转
                                     （几何上正切要 ADV≈R；ADV 小=切内道，大=走外道） */
#define TURN_SCALE_ROLL  1.00f    /* 弧线转弯的角度修正：转多了减小、转少了加大 */
#define TURN_TRIM_NEXT   1        /* 1 = 把转弯过程中已经前进的距离从下一段里程里扣掉 */

/* 转弯时已经走出去的"沿新方向"距离，由 turn_roll 填，下一段开始时消费掉 */
static float g_trim_cm = 0.0f;
#define AUTO_START_MS   1000u     /* 上电多久自动开跑（这段时间内按 k 可先标定） */

/* ---------- 环岛：不拆段、不靠里程拐弯，用更激进的 PID 咬住白线 ---------- */
#define RING_PID        1         /* 1 = 第 RING_LEG 段换用下面这套激进的参数 */
#define RING_LEG        18        /* 环岛所在段号 */
#define RING_KP         14.0f     /* 直行是 5.0 —— 环里给到 2.8 倍 */
#define RING_KD         16.0f     /* 直行是 6.0 */
#define RING_STEER_MAX  620       /* 直行是 400；真正卡住差速的是下面的 WHEEL_MIN */
#define RING_WHEEL_MIN  60        /* 直行是 150 —— 环里允许内轮几乎刹停，差速才够大 */
#define RING_BOOST      140       /* |e| >= 25（已经偏到边上）时额外再加这么多 PWM */
#define RING_SPEED      280       /* 环里慢一点，别甩出去 */
#define RING_EXIT_LOST  0u        /* 0 = 关掉提前出环，环岛只按 LEGS[RING_LEG].cm 的长度走 */
#define RING_USE_NOPID  1         /* 1 = 环岛段也套用 NOPID_ZONES（默认开：表里 leg=18 的区间生效） */

/* ---------- 循迹 PID ---------- */
/* err 是按位加权算出来的位置，范围大约 -35..+35：
 *   全部在中间 → 0；偏一个传感器 → ±5；偏到边上 → ±35            */
#define KP              5.5f
#define KI              0.0f      /* 先不用积分，免得越积越歪 */
#define KD              7.0f
#define STEER_MAX       400       /* PID 输出限幅（PWM 单位） */
#define STEER_SIGN      (+1)      /* 车往反方向跑就改成 -1 */

/* 传感器方向：X1(AD0=000) 装在车头左侧，所以 X1 应给"负"权重。
 * 2026-11-13 实测：SENSOR_REVERSE=1 时 X1 压线读回 e=+35（线在左却报正），翻成 0。 */
#define SENSOR_REVERSE  0
/* 逐路启用掩码：某一路坏了（永远 1 或永远 0）就把对应位清 0，别让它捣乱 */
#define TRACK_ENABLE_MASK 0xFFu
/* 极性翻转：正常情况下压白线读 1，不用翻 */
#define TRACK_INV       0x00u

/* 丢线（八路全蓝）时：先按上一次的方向继续修，超过 LOST_MS 就直着走 */
#define LOST_MS         300u
/* 横向宽白（路口/横线）时 PID 暂停修正，免得被带偏 */
#define JUNC_POP        7
#define JUNC_HOLD_MS    200u

/* ---------- 时间/保护 ---------- */
#define LEG_SETTLE_MS   50        /* v1.6.4: 100->15->50 段/转之间的稳定等待（稳一点，每处约 50ms） */
#define STOP_BRAKE_MS   50        /* v1.6.4: 20->50 刹车后等轮子真停（原来 60/70）。低于 15 时反转冲击电流会让 2S 电池掉压复位 */
#define TURN_TIMEOUT_MS 15000u
#define STALL_MS        800u

/* 调试打印间隔 */
#define REPORT_MS       1000u

/* ==================== 二、1ms 时基 ==================== */

static volatile uint32_t g_ms = 0;
void SysTick_Handler(void) { g_ms++; }
static uint32_t millis(void) { return g_ms; }

static void sys_tick_init(void)
{
    SysTick->LOAD = 72000u - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x7u;
}

static void delay_ms(uint32_t ms)
{
    uint32_t t0 = g_ms;
    while ((uint32_t)(g_ms - t0) < ms) { __NOP(); }
}

/* ==================== 三、串口 ==================== */

static void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 4)) | (0xBu << 4);   /* PA9  复用推挽 */
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 8)) | (0x8u << 8);   /* PA10 输入模式（方向由 ODR 决定） */
    GPIOA->BSRR = (1u << 10);                                 /* ODR=1 才是上拉，否则是下拉 */
    USART1->BRR = 0x271;                                      /* 72MHz/115200 */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void uart_char(char c)
{
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = (uint16_t)(uint8_t)c;
}

static void uart_str(const char *s) { while (*s) uart_char(*s++); }

static void uart_int(int32_t v)
{
    char buf[12];
    int8_t i = 0;
    uint32_t u;
    if (v < 0) { uart_char('-'); u = (uint32_t)(-(int64_t)v); }  /* INT32_MIN 也不炸 */
    else       { u = (uint32_t)v; }
    if (u == 0) { uart_char('0'); return; }
    while (u > 0 && i < 11) { buf[i++] = (char)('0' + u % 10u); u /= 10u; }
    while (i > 0) uart_char(buf[--i]);
}

static void uart_num(const char *tag, int32_t v) { uart_str(tag); uart_int(v); }

static int uart_try_get(void)
{
    if (USART1->SR & USART_SR_RXNE) return (int)(USART1->DR & 0xFFu);
    if (USART1->SR & USART_SR_ORE) (void)USART1->DR;
    return -1;
}

/* ==================== 四、LED ==================== */

static void led_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH = (GPIOC->CRH & ~(0xFu << 20)) | (0x2u << 20);
    GPIOC->BSRR = (1u << 13);
}

static void led_set(int on)
{
    if (on) GPIOC->BRR  = (1u << 13);
    else    GPIOC->BSRR = (1u << 13);
}

/* ==================== 五、电机：TB6612 ==================== */

static void motor_gpio_dir(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRH = (GPIOB->CRH & ~(0xFFFFu << 16)) | (0x3333u << 16);
    GPIOB->BRR = (1u << 12) | (1u << 13) | (1u << 14) | (1u << 15);
}

static void pwm_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_TIM1EN;
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 0))  | (0xBu << 0);    /* PA8  */
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 12)) | (0xBu << 12);   /* PA11 */

    TIM1->PSC  = 71;        /* 1MHz */
    TIM1->ARR  = 999;       /* 1kHz */
    TIM1->CCR1 = 0;
    TIM1->CCR4 = 0;
    TIM1->CCMR1 = (TIM1->CCMR1 & ~(7u << 4))  | (6u << 4);
    TIM1->CCMR2 = (TIM1->CCMR2 & ~(7u << 12)) | (6u << 12);
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1  |= TIM_CR1_CEN;
}

static void motor_set(int32_t right, int32_t left)
{
    if (right >  999) right =  999;
    if (right < -999) right = -999;
    if (left  >  999) left  =  999;
    if (left  < -999) left  = -999;

#if RIGHT_INVERT
    right = -right;
#endif
#if LEFT_INVERT
    left = -left;
#endif

    if (right >= 0) { GPIOB->BSRR = (1u << 12); GPIOB->BRR = (1u << 13); }
    else            { GPIOB->BRR  = (1u << 12); GPIOB->BSRR = (1u << 13); }
    if (left >= 0)  { GPIOB->BSRR = (1u << 14); GPIOB->BRR = (1u << 15); }
    else            { GPIOB->BRR  = (1u << 14); GPIOB->BSRR = (1u << 15); }

    TIM1->CCR1 = (uint16_t)(right < 0 ? -right : right);
    TIM1->CCR4 = (uint16_t)(left  < 0 ? -left  : left);
}

static void motor_stop(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR4 = 0;
    GPIOB->BRR = (1u << 12) | (1u << 13) | (1u << 14) | (1u << 15);
}

/* ==================== 六、编码器 ==================== */

static volatile int32_t enc_r = 0;
static volatile uint8_t enc_r_state = 0;

static const int8_t Q[4][4] = {
    {  0, -1,  1,  0 },
    {  1,  0,  0, -1 },
    { -1,  0,  0,  1 },
    {  0,  1, -1,  0 }
};

static void enc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    /* PB6/PB7 原来是浮空输入(0x44)，若左编码器是开漏输出会读不到高电平；
       统一改成上拉输入(0x88)，与 PB8/PB9 一致，推挽/开漏都能用 */
    GPIOB->CRL = (GPIOB->CRL & ~(0xFFu << 24)) | (0x88u << 24);
    GPIOB->CRH = (GPIOB->CRH & ~(0xFFu << 0))  | (0x88u << 0);
    /* 0x88 只是「上拉/下拉输入」，到底上拉还是下拉由 ODR 决定：
       ODR=0 是下拉。PB6/PB7 必须一起置 1 才是上拉（2026-11-13 review 指出） */
    GPIOB->BSRR = (1u << 6) | (1u << 7) | (1u << 8) | (1u << 9);

    AFIO->EXTICR[2] = (AFIO->EXTICR[2] & ~0x00FFu) | 0x0011u;
    EXTI->IMR  |= (1u << 8) | (1u << 9);
    EXTI->RTSR |= (1u << 8) | (1u << 9);
    EXTI->FTSR |= (1u << 8) | (1u << 9);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    TIM4->PSC   = 0;
    TIM4->ARR   = 0xFFFF;
    TIM4->CCMR1 = (TIM4->CCMR1 & ~0xFFFFu) | 0x0101u;
    TIM4->SMCR  = (TIM4->SMCR & ~7u) | 3u;
    TIM4->CNT   = 0;
    TIM4->CR1  |= TIM_CR1_CEN;

    enc_r_state = ((GPIOB->IDR & (1u << 8)) ? 2u : 0u)
                | ((GPIOB->IDR & (1u << 9)) ? 1u : 0u);
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t idr = GPIOB->IDR;                 /* 读一次，避免两次读之间状态变化 */
    uint8_t st = ((idr & (1u << 8)) ? 0x02u : 0u) | ((idr & (1u << 9)) ? 0x01u : 0u);
    int8_t d = Q[enc_r_state][st];
    enc_r_state = st;
    enc_r += d;
    EXTI->PR = (1u << 8) | (1u << 9);
}

/* 取走两轮各自的里程（cm，带符号），并清零 */
static void odom_take2(float *lcm, float *rcm)
{
    NVIC_DisableIRQ(EXTI9_5_IRQn);
    int16_t cl = (int16_t)TIM4->CNT;
    int16_t cr = (int16_t)enc_r;
    TIM4->CNT = 0;
    enc_r = 0;
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    *lcm = (float)((int32_t)cl * ENC_LEFT_SIGN)  / ENC_PER_CM;
    *rcm = (float)((int32_t)cr * ENC_RIGHT_SIGN) / ENC_PER_CM;
}

/* ==================== 七、八路循迹（模拟量 ADC） ==================== */

/* 2026-11-13 换红光灰度传感器：只有 AO 模拟输出，没有比较器数字输出。
 * 所以这里不是"读 0/1"，而是：
 *   1) ADC 读 8 路原始值（0..4095）
 *   2) 每路标定 蓝底值 / 白线值，归一化成 0..1000 的"白度"
 *   3) 用白度加权求线的**亚位中心**（不是 ±5 的台阶），PID 的微分项会平顺很多
 */

#define SENSOR_N      8

/* 传感器板只有 AD0/AD1/AD2/OUT 四根线 —— 板载 CD4051 八选一模拟开关：
 * 三根地址线选通道，OUT 送出选中那一路的模拟量，所以 STM32 只要 1 个 ADC 脚。
 *
 * 模块走 5V 供电，所以：
 *   ① 地址线用 **5V 耐压的 FT 脚 + 开漏输出 + 外部 10k 上拉到 5V**
 *      （4051 在 5V 下 VIH≈3.5V，3.3V 推挽输出卡在门槛上，会偶发选错通道）
 *        AD0 -> PB10      AD1 -> PB11      AD2 -> PA15
 *   ② OUT 是 0..5V，PA3 模拟脚不耐 5V，必须 **2:1 分压** 后再接
 *        10k 串 + 20k 对地  →  5V 变成 3.3V
 *
 * 如果模块其实由 3.3V 供电（或板载已分压），把 MUX_FT_PINS 改成 0，
 * 地址线可直接接 PA0/PA1/PA2 推挽输出（那就是 v0.6.1 的接法）。 */
#define MUX_FT_PINS   0        /* 0 = 实测 OUT 满幅 3.3V，地址线直连 PA0..PA2 */

#define MUX_AD0_PIN   10       /* PB10 */
#define MUX_AD1_PIN   11       /* PB11 */
#define MUX_AD2_PIN   15       /* PA15 */
#define ADC_CH        3        /* PA3 = ADC1_IN3（经分压） */

#define ADC_MAXV      4095
#define CAL_MIN_SPAN  300      /* 白线-蓝底 至少差这么多 ADC，否则这路算废 */
#define LINE_V        300      /* 白度超过 300 才认为这一路"压在线上" */

static uint16_t g_adc[SENSOR_N];
static uint16_t g_min[SENSOR_N];      /* 扫描到的最小值 */
static uint16_t g_max[SENSOR_N];      /* 扫描到的最大值 */
static uint16_t g_blk[SENSOR_N];      /* 蓝底读数 */
static uint16_t g_wht[SENSOR_N];      /* 白线读数 */
static uint8_t  g_ok_mask = 0xFFu;    /* 哪些通道能用 */
static uint8_t  g_cal_ok  = 0;
static uint16_t g_thr[SENSOR_N];

static void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    /* 先把 ADC1 复位一次：上电后残留状态（校准没做完、ADON 卡住）会
       让第一次转换永远等不到 EOC —— 就是现象里"打完 banner 就没了"。 */
    RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
    for (volatile uint32_t k = 0; k < 200u; k++) { __NOP(); }
    RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;
    for (volatile uint32_t k = 0; k < 200u; k++) { __NOP(); }

#if MUX_FT_PINS
    /* PB10/PB11/PA15 = 开漏输出 50MHz（CRH 里配 0x7）：拉低有效，
       高电平靠外部 10k 上拉到 5V —— 这几根是 FT（5V 耐压）脚 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRH = (GPIOB->CRH & ~0x0000FF00u) | 0x00007700u;   /* PB10, PB11 */
    GPIOA->CRH = (GPIOA->CRH & ~0xF0000000u) | 0x70000000u;   /* PA15 */
    GPIOB->ODR |= (1u << MUX_AD0_PIN) | (1u << MUX_AD1_PIN);   /* 释放 = 高 */
    GPIOA->ODR |= (1u << MUX_AD2_PIN);
    /* PA0..PA7 全部模拟输入（PA0..PA2 这次不用，别留成悬空输入） */
    GPIOA->CRL = 0x00000000u;
#else
    /* PA0..PA2 = 推挽输出 50MHz（0x3），PA3 = 模拟输入，PA4..PA7 保持模拟 */
    GPIOA->CRL = 0x00000333u;
    GPIOA->ODR &= ~0x0007u;
#endif

    /* ADC 时钟 = PCLK2/6 = 12MHz（不能超过 14MHz） */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;

    ADC1->CR1   = 0;                    /* 独立模式、12 位 */
    ADC1->CR2   = ADC_CR2_ADON;         /* 清掉 CONT/EXTTRIG 残留，只留 ADON */
    ADC1->SMPR2 = 0xB6DB6Du;            /* ch0..ch9 采样 55.5 周期（SMP=101）
                                         注意：原来的 0x924924 是 41.5 周期，注释写错了 */
    ADC1->SQR1  = 0;                    /* 规则序列长度 = 1 */
    ADC1->CR2  |= ADC_CR2_ADON;
    delay_ms(10);            /* 实测 tSTAB 要够，1ms 不够 */

    /* ADC 自校准（上电必须做一次，否则读数有固定偏差） */
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    delay_ms(5);
    ADC1->CR2 |= ADC_CR2_CAL;
    delay_ms(5);
    ADC1->CR2 = ADC_CR2_ADON;           /* 校准后重写 ADON */
    delay_ms(2);
}

/* 读一个通道（0..7）。一次转换约 5.7us，八路一轮 ≈ 60us，对 2ms 环无压力 */

/* 微秒级忙等：优先用 DWT 周期计数器（精确），没开起来就退回空转。
 * 原来的 n*9 次空转只是"大概"，改动优化等级就会漂；ADC 只需保证 >6us，
 * 但既然有硬件计数器就别再猜了。 */
#define DWT_CTRL_REG    (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT_REG  (*(volatile uint32_t *)0xE0001004u)
#define DEMCR_REG       (*(volatile uint32_t *)0xE000EDFCu)
static uint8_t g_cyc_ok = 0;

static void cyc_init(void)
{
    DEMCR_REG |= (1u << 24);              /* TRCENA */
    DWT_CYCCNT_REG = 0;
    DWT_CTRL_REG |= (1u << 0);            /* CYCCNTENA */
    for (volatile uint32_t k = 0; k < 200u; k++) { __NOP(); }   /* 等计数器跑几步再判 */
    if (DWT_CYCCNT_REG != 0) g_cyc_ok = 1;
}

static void busy_us(uint32_t n)
{
    if (!g_cyc_ok) {
        volatile uint32_t k;
        for (k = 0; k < n * 20u; k++) { __NOP(); }
        return;
    }
    uint32_t t0 = DWT_CYCCNT_REG;
    uint32_t need = n * 72u;              /* 72MHz → 72 周期 = 1us */
    while ((uint32_t)(DWT_CYCCNT_REG - t0) < need) { }
}

/* 标准方式：写通道 → SWSTART → 等 EOC */

/* 备用：开转换后固定等 80us 再读 DR，不管 EOC */

static uint32_t g_adc_wait_us = 60;   /* 实测：EXTTRIG 触发后 ~40us 就有结果 */

static uint16_t adc_read(uint8_t ch)
{
    /* 2026-11-13 实测（v0.7.3 的 A..G 七种触发方式）：
     *   只写 SWSTART        -> sr=0  dr=0     转换根本不启动
     *   EXTTRIG + EXTSEL=7  -> sr=18 dr=4084  ✅ 唯一可靠
     *   写两次 ADON         -> sr=18 dr=4077  ✅ 也能启动
     *   CONT 连续模式       -> sr=0  dr=0     ❌
     * 所以统一走「外部触发源 = SWSTART」这条路：先写 EXTSEL/EXTTRIG，再写 SWSTART。
     * 转换完不看 EOC（这颗片子标志位时好时坏），等固定时间直接读 DR。 */
    ADC1->SQR3 = ch;
    ADC1->CR2 |= (7u << 17) | ADC_CR2_EXTTRIG;   /* EXTSEL = SWSTART(111) */
    ADC1->CR2 |= ADC_CR2_SWSTART;                /* 产生一次触发 */
    busy_us(g_adc_wait_us);
    return (uint16_t)(ADC1->DR & 0x0FFFu);
}

/* 上电自诊断：判断到底是「转换慢」还是「PA3 上根本没电压」 */


/* ---- v0.7.3：ADC 触发方式穷举 ---- */


/* 上电探测：把 3 种方式的结果都打出来，一看就知道卡在哪 */


/* 一次完整试验：复位 → 配通道/采样 → (可选)校准 → 触发 → 等 3ms 读 SR/DR
 * cr2_extra 里带 CONT 时是连续模式；带 EXTTRIG 时先用 SWSTART 当外部触发源 */


/* adc_scan() 定义在下面 */

/* 选通 4051 的某一路（AD0=低位） */
static void mux_select(uint8_t ch)
{
#if MUX_FT_PINS
    uint32_t b = GPIOB->ODR & ~((1u << MUX_AD0_PIN) | (1u << MUX_AD1_PIN));
    uint32_t a = GPIOA->ODR & ~(1u << MUX_AD2_PIN);
    if (ch & 1u) b |= (1u << MUX_AD0_PIN);
    if (ch & 2u) b |= (1u << MUX_AD1_PIN);
    if (ch & 4u) a |= (1u << MUX_AD2_PIN);
    GPIOB->ODR = b;
    GPIOA->ODR = a;
#else
    uint32_t odr = GPIOA->ODR & ~0x0007u;
    odr |= (uint32_t)(ch & 0x07u);
    GPIOA->ODR = odr;
#endif
    /* 模拟开关切换后要等输出稳住：72MHz 下 60 次空转 ≈ 2us */
    for (volatile uint32_t k = 0; k < 60u; k++) { __NOP(); }
}

static void adc_scan(void)
{
    uint8_t i;
    for (i = 0; i < SENSOR_N; i++) {
        mux_select(i);
        g_adc[i] = adc_read(ADC_CH);
    }
}

/* 白度：0 = 蓝底，1000 = 白线（超出范围钳住） */
static int32_t whiteness(uint8_t i)
{
    int32_t span = (int32_t)g_wht[i] - (int32_t)g_blk[i];
    int32_t num;
    if (span < 0) span = -span;
    if (span < CAL_MIN_SPAN) return 0;

    num = (int32_t)g_adc[i] - (int32_t)g_blk[i];
    /* 2026-11-13 修正：本模块蓝底≈满量程、白线更低，也就是 wht < blk。
     * 原来只按 wht > blk 算，白线读数一律被钳成 0 -> vsum 恒 0 -> e=999/0
     * -> 现象就是"跑起来 pid 一直是 0"。这里按标定方向自动翻号。 */
    if (g_wht[i] < g_blk[i]) num = -num;

    int32_t v = num * 1000 / span;
    if (v < 0)    v = 0;
    if (v > 1000) v = 1000;
    return v;
}

/* 线的位置误差：-35(最左) .. +35(最右)，用白度加权。
 * 岔路处理：先按"连续压线段"分簇，只取离上一帧位置最近的那一簇算质心。
 * 若把旁边岔路那一簇一起加权，质心会被拉向岔口——这正是一跑岔路就歪的原因。 */
static int16_t g_center_i = 35;    /* 上一帧位置，单位 0.1 通道，35 = 正中 */

static int16_t sense_err(int16_t *vsum_out, int16_t *n_on_out)
{
    static const int16_t W[SENSOR_N] = { -35, -25, -15, -5, 5, 15, 25, 35 };
    int32_t vsum = 0, n = 0, i, k;
    uint8_t on[SENSOR_N];
    int32_t vv[SENSOR_N];
    int32_t best_a = -1, best_b = -1, best_c = 35, best_d = 0x7FFFFFFF;

    adc_scan();

    for (i = 0; i < SENSOR_N; i++) {
        int32_t v = 0;
        if (g_ok_mask & (uint8_t)(1u << i)) v = whiteness((uint8_t)i);
        vv[i] = v;
        on[i] = (v >= LINE_V) ? 1u : 0u;
        if (on[i]) n++;
        vsum += v;
    }
    if (vsum_out) *vsum_out = (int16_t)(vsum / 1000);
    if (n_on_out) *n_on_out = (int16_t)n;
    /* 丢线判据交给下面的分簇（没有任何一路 v>=LINE_V 才会 999）。
       原来这里还有个 vsum<800，实测白线 175、蓝底 4095 时
       只压到一点点线就会 vsum 不足而误报丢线，已去掉。 */

    /* 找出所有连续簇，挑中心离上一帧最近的一簇（car follows the nearest line） */
    i = 0;
    while (i < SENSOR_N) {
        if (!on[i]) { i++; continue; }
        int32_t a = i, b = i;
        while (b + 1 < SENSOR_N && on[b + 1]) b++;
        int32_t cs = 0, cw = 0;
        for (k = a; k <= b; k++) { cs += vv[k] * (k * 10 + 5); cw += vv[k]; }
        if (cw > 0) {
            int32_t c = cs / cw;              /* 0..70 → 通道 0.0..7.0 */
            int32_t dd = c - g_center_i;
            if (dd < 0) dd = -dd;
            if (dd < best_d) { best_d = dd; best_a = a; best_b = b; best_c = c; }
        }
        i = b + 1;
    }
    if (best_a < 0) return 999;

    g_center_i = best_c;

    /* 只用这一簇算误差 */
    int32_t wsum = 0, vsel = 0;
    for (k = best_a; k <= best_b; k++) {
        uint8_t pos = SENSOR_REVERSE ? (uint8_t)(SENSOR_N - 1 - k) : (uint8_t)k;
        wsum += vv[k] * (int32_t)W[pos];
        vsel += vv[k];
    }
    if (vsel <= 0) return 999;

    int32_t e = wsum / vsel;
    if (e >  999) e =  999;
    if (e < -999) e = -999;
    return (int16_t)e;
}

/* ---- 标定：上电按 'k'，3 秒内把车横着扫过白线 ---- */
static void cal_do(void)
{
    uint8_t i;
    uint16_t snap[SENSOR_N];

    uart_str("\r\n=== CALIBRATE (analog) ===\r\n");

    /* 先拍一张：此刻车通常压在线上，用来判断"白线是大还是小" */
    adc_scan();
    for (i = 0; i < SENSOR_N; i++) snap[i] = g_adc[i];
    for (i = 0; i < SENSOR_N; i++) { g_min[i] = 4095; g_max[i] = 0; }

    uart_str("put the car ON the white line, then slide it sideways within 3s ...\r\n");

    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < 3000u) {
        adc_scan();
        for (i = 0; i < SENSOR_N; i++) {
            if (g_adc[i] < g_min[i]) g_min[i] = g_adc[i];
            if (g_adc[i] > g_max[i]) g_max[i] = g_adc[i];
        }
        led_set(1); delay_ms(20);
        led_set(0); delay_ms(20);
    }

    /* 判定极性：中间两路此刻在线上，看它离 max 近还是离 min 近 */
    int32_t to_max = 0, to_min = 0;
    for (i = 2; i <= 5; i++) {
        uint16_t s = snap[i];
        if (s < g_min[i] || s > g_max[i]) continue;
        to_max += (int32_t)(g_max[i] - s);
        to_min += (int32_t)(s - g_min[i]);
    }
    /* 极性判定：snap 是"按 k 那一刻"的读数（车应压在白线上）。
     *   snap 接近 min  → to_max 大 → 白线是低值 → white_is_max = 0
     *   snap 接近 max  → to_min 大 → 白线是高值 → white_is_max = 1
     * 原代码写的是 (to_max >= to_min)，正好相反 —— 标定会把 blk/wht 互换，
     * 之后 whiteness 全反、PID 恒为 0 或乱打方向。2026-11-14 修正。 */
    int32_t denom = to_max + to_min;
    uint8_t white_is_max;
    if (denom <= 0)                              white_is_max = 0;  /* 判不了：按本硬件（蓝底高/白线低） */
    else if ((to_max - to_min) * 10 >  denom * 3) white_is_max = 0; /* 明显靠 min */
    else if ((to_min - to_max) * 10 >  denom * 3) white_is_max = 1; /* 明显靠 max */
    else                                          white_is_max = 0; /* 居中：按已知硬件 */
    uart_str(white_is_max ? "snap near MAX -> white = MAX\r\n"
                          : "snap near MIN -> white = MIN\r\n");

    g_ok_mask = 0;
    uart_str("ch  min  max  span  blk  wht  thr  ok\r\n");
    for (i = 0; i < SENSOR_N; i++) {
        int32_t span = (int32_t)g_max[i] - (int32_t)g_min[i];
        if (span < CAL_MIN_SPAN) {
            uart_str("  ch"); uart_int(i); uart_str(": span too small, DISABLED\r\n");
            continue;
        }
        if (white_is_max) { g_blk[i] = g_min[i]; g_wht[i] = g_max[i]; }
        else              { g_blk[i] = g_max[i]; g_wht[i] = g_min[i]; }
        g_thr[i] = (uint16_t)((g_blk[i] + g_wht[i]) / 2);
        g_ok_mask |= (uint8_t)(1u << i);

        uart_str("  ch"); uart_int(i);
        uart_str(" : "); uart_int(g_min[i]);
        uart_str(" ");   uart_int(g_max[i]);
        uart_str(" ");   uart_int(span);
        uart_str("  b="); uart_int(g_blk[i]);
        uart_str(" w=");  uart_int(g_wht[i]);
        uart_str(" t=");  uart_int(g_thr[i]);
        uart_str(" OK\r\n");
    }

    if (g_ok_mask == 0) {
    uart_str("!! no channel has enough contrast: check power / height / sensor gap\r\n");
        g_cal_ok = 0;
        return;
    }

    g_cal_ok = 1;
    uart_str("white is "); uart_str(white_is_max ? "MAX" : "MIN");
    uart_str(", ok_mask=0x");
    {
        static const char hx[] = "0123456789ABCDEF";
        uart_char(hx[(g_ok_mask >> 4) & 0xF]);
        uart_char(hx[g_ok_mask & 0xF]);
    }
    uart_str("\r\n=== CALIBRATE DONE ===\r\n");
}

/* 没标定时的兜底：假设 0=蓝底 4095=白线（真跑前务必按 k） */
static void cal_default(void)
{
    uint8_t i;
    for (i = 0; i < SENSOR_N; i++) {
        /* 2026-11-14 实测：蓝底 ADC ≈ 4095（饱和），白线 ≈ 150..200。
         * 蓝底取高值、白线取低值，按实测改默认值，不标定也能用 */
        g_blk[i] = 4095; g_wht[i] = 175; g_thr[i] = 2135;
    }
    g_ok_mask = 0xFFu;
    g_cal_ok = 0;
}

/* 观察模式：不跑车，只看八路 ADC 与白度，方便调高度/看对比度 */
static void sensor_watch(void)
{
    uart_str("\r\n=== ANALOG WATCH ===\r\n");
    uart_str("adc(0..4095) and whiteness(0..1000). 'k'=recal, 'x'=quit\r\n");
    while (1) {
        int c = uart_try_get();
        if (c == 'x' || c == 'X') {
            g_center_i = 35;              /* 退出观察，别把观察时的位置带进正式跑 */
            uart_str("quit watch\r\n");
            return;
        }
        if (c == 'k' || c == 'K') { cal_do(); continue; }

        int16_t vs = 0, non = 0;
        int16_t e = sense_err(&vs, &non);

        uart_str("adc:");
        for (uint8_t i = 0; i < SENSOR_N; i++) {
            uart_char(' ');
            uart_int(g_adc[i]);
        }
        uart_str("  w:");
        for (uint8_t i = 0; i < SENSOR_N; i++) {
            uart_char(' ');
            uart_int(whiteness(i));
        }
        uart_num("  e=", (e == 999 ? 999 : (int32_t)e));
        uart_num(" v=", vs);
        uart_str("\r\n");

        led_set(1); delay_ms(100); led_set(0); delay_ms(100);
    }
}

/* ==================== 八、公共状态 ==================== */

static uint8_t g_abort  = 0;
static uint8_t g_paused = 0;

/* 'x' 要求 600ms 内连发两次 —— 不接串口线时 RX 悬空、电机 PWM 会把噪声
   灌进来，单字节就可能被误认成 'x'（车跑一半自己停）。 */
static uint8_t poll_cmd(void)
{
    static uint32_t x_t = 0;
    static uint8_t  x_cnt = 0;
    int c;
    while ((c = uart_try_get()) >= 0) {
        if (c == 'x' || c == 'X') {
            uint32_t now = millis();
            if (x_cnt && (uint32_t)(now - x_t) < 600u) { g_abort = 1; x_cnt = 0; }
            else { x_cnt = 1; x_t = now; }
        } else if (c == 'p' || c == 'P') {
            g_paused = !g_paused;
        }
    }
    if (x_cnt && (uint32_t)(millis() - x_t) >= 600u) x_cnt = 0;
    return g_abort;
}

static void wait_if_paused(void)
{
    if (!g_paused) return;
    motor_stop();
    led_set(1);
    uart_str("  paused (press p to continue)\r\n");
    while (g_paused) {
        int c = uart_try_get();
        if (c == 'p' || c == 'P') g_paused = 0;
        if (c == 'x' || c == 'X') { g_abort = 1; g_paused = 0; }
        /* 暂停时如果有人在推车，计数会攒到 int16 溢出；边等边丢掉 */
        { float a, b; odom_take2(&a, &b); }
        led_set(((millis() / 200u) & 1u) ? 0 : 1);
        delay_ms(5);
    }
    led_set(0);
    uart_str("  resumed\r\n");
}

static int32_t speed_for(float remain, int32_t full)
{
    int32_t sp = full;
    if (remain < DECEL_CM) {
        sp = SPEED_MIN + (int32_t)((float)(full - SPEED_MIN) * remain / DECEL_CM);
        if (sp < SPEED_MIN) sp = SPEED_MIN;
    }
    return sp;
}

/* ==================== 九、直行段：里程定长度 + PID 循迹纠偏 ==================== */

/* 返回 1 = 走完，0 = 被 x 打断，-1 = 堵转/超时 */
static uint8_t in_nopid(int leg, float done);   /* 定义在路线表那边 */
static uint8_t in_no_scan(int leg, float done); /* 段中找线的禁用区间，定义在路线表那边 */

/* 由 main 在进入"最后一段"之前置 1：这一段不看剩余距离，直接全速 */
static uint8_t g_sprint = 0;

/* ==================== 九点五、转弯后找线 ====================
 * 2026-11-16：纯里程转弯是开环的，每段都会带一点偏差，29 段叠起来必然跑飞。
 * 现在每段转弯之后先确认"回到线上了"再走：
 *   - 只要有一路压线(白度>=SCAN_ANY_V)就算在线，绝不判丢线；
 *   - 判定丢线就原地左右各扫 SCAN_DEG，扫到立刻走（不慢速前进）；
 *   - 整轮扫不到就后退 SCAN_BACK_CM 再扫，直到扫到为止；
 *   - 扫线期间的车轮转动不计入下一段里程（drive_pid 开头会 drain odom）。
 */
#define SCAN_ANY_V      120      /* 任一路白度 >= 这个值就算"看到线"，比 LINE_V 灵敏 */
#define SCAN_DEG        30.0f    /* 左右各扫 30 度 */
#define SCAN_BACK_TURN_DEG 90.0f /* 同向摸两次都失败后，反方向转 90 度再扫 */
#define SCAN_PWM        300      /* 原地扫线速度 */
#define SCAN_BACK_CM    5.0f     /* 一轮扫不到后退 5cm */
#define SCAN_BACK_PWM   300
#define SCAN_MAX_ROUNDS 12       /* 安全上限：12 轮 = 后退 60cm 还无线就报错停机 */
#define SCAN_ROT_MS     4000u    /* 单次转动超时保护 */
#define ALIGN_HEAD_CM   30.0f    /* 段开头 30cm 内丢线，走"找线"而不是直走（旧，v1.6.0 起按 LEG_EDGE_CM 判定） */
#define LEG_EDGE_CM     20.0f    /* 段首/段尾各 20cm 内丢线不扫线（留给转弯衔接/甩尾），只有中间段才立刻扫 */
#define MID_LOST_MS     200u     /* v1.6.5: 段中丢线多久算真丢（用户要求 200ms @42cm/s 约 8cm） */

/* 上一次转弯的方向：+1 右转，-1 左转，0 = 还没转过。
   丢线时先朝这个方向继续摸，因为绝大多数情况是刚才转少了 */
static int8_t g_last_turn_dir = 0;
/* 最后一次 PID 修正的方向：+1 = 往右修，-1 = 往左修。
 * 段中丢线时第一次扫线固定朝它的反方向（往右修还丢，说明线跑到左边去了，先往左摸） */
static int8_t g_last_pid_dir = 0;

/* 有任何一路压在线上吗？ */
static int scan_any_line(void)
{
    int i;
    adc_scan();
    for (i = 0; i < SENSOR_N; i++) {
        if (!(g_ok_mask & (uint8_t)(1u << i))) continue;
        if (whiteness((uint8_t)i) >= SCAN_ANY_V) return 1;
    }
    return 0;
}

/* 原地转 deg 度；转到、或途中任一路压线，立刻停。
 * 返回 1 = 见到线，0 = 转完了没见到，-1 = 用户按了 x */
static int scan_rot(float deg, int32_t dir)
{
    float need = 3.14159265f * WHEEL_BASE_CM * (deg > 0 ? deg : -deg) / 360.0f * TURN_SCALE;
    float got = 0.0f;
    uint32_t t0 = millis();
    int32_t sp;

    { float jl, jr; odom_take2(&jl, &jr); }

    while (got < need) {
        if (poll_cmd()) { motor_stop(); return -1; }

        float dl, dr;
        odom_take2(&dl, &dr);
        float d = (dir > 0) ? (dl - dr) : (dr - dl);
        if (d > 0.0f) got += d * 0.5f;

        if (scan_any_line()) { motor_stop(); return 1; }

        sp = speed_for(need - got, SCAN_PWM);
        if (dir > 0) motor_set(-sp,  sp);
        else         motor_set( sp, -sp);

        if ((uint32_t)(millis() - t0) > SCAN_ROT_MS) break;
        led_set(1);
        delay_ms(4);
        led_set(0);
    }
    motor_stop();
    delay_ms(120);
    return scan_any_line() ? 1 : 0;
}

/* 后退 cm（同样扫到线就停）。1=见到线，0=退够了，-1=按了 x */
static int scan_back(float cm)
{
    float got = 0.0f;
    uint32_t t0 = millis();

    { float jl, jr; odom_take2(&jl, &jr); }

    while (got < cm) {
        if (poll_cmd()) { motor_stop(); return -1; }

        float dl, dr;
        odom_take2(&dl, &dr);
        float d = (dl + dr) * 0.5f;
        if (d < 0.0f) got += -d;

        if (scan_any_line()) { motor_stop(); return 1; }

        motor_set(-SCAN_BACK_PWM, -SCAN_BACK_PWM);

        if ((uint32_t)(millis() - t0) > 8000u) break;
        delay_ms(4);
    }
    motor_stop();
    delay_ms(120);
    return scan_any_line() ? 1 : 0;
}

/* 主入口：已经在线就直接放行；否则原地扫、扫不到后退 5cm 再扫。
 * 返回 1 = 在线可以走，0 = 用户中止，-1 = 若干轮之后放弃 */
static int8_t scan_find_line_ex(const char *why, int8_t first_dir)
{
    int32_t r;
    int8_t f;
    int8_t d1 = (first_dir != 0) ? first_dir : +1;   /* 第一下朝哪边扫 */

    if (scan_any_line()) {
        uart_str("  [align] ");
        uart_str(why);
        uart_str(": on line, go\r\n");
        return 1;
    }

    uart_str("  [align] ");
    uart_str(why);
    uart_str(": LOST -> scan in place\r\n");

    if (first_dir != 0) {
        uart_num("    first sweep dir=", d1);
        uart_str(" (opposite of last pid fix)\r\n");
    }

    /* 转完找线时才用转弯方向提示；段中丢线（first_dir!=0）直接进左右扫 */
    if (first_dir == 0 && g_last_turn_dir != 0) {
        uart_num("    last turn dir=", g_last_turn_dir);
        uart_str(", try +30deg same way\r\n");

        f = (int8_t)scan_rot(SCAN_DEG, g_last_turn_dir);
        if (f < 0) return 0;
        if (f == 1) goto found;

        uart_str("    still lost, another +30deg same way\r\n");
        f = (int8_t)scan_rot(SCAN_DEG, g_last_turn_dir);
        if (f < 0) return 0;
        if (f == 1) goto found;

        /* 同方向摸了 60° 都没有：反方向转 90°（越过原朝向 30°），再走左右扫 */
        uart_num("    no line, turn opposite ", (int32_t)SCAN_BACK_TURN_DEG);
        uart_str("deg, then sweep\r\n");
        f = (int8_t)scan_rot(SCAN_BACK_TURN_DEG, -g_last_turn_dir);
        if (f < 0) return 0;
        if (f == 1) goto found;
    }

    for (r = 1; r <= SCAN_MAX_ROUNDS; r++) {
        uart_num("    scan round=", r);
        uart_str("\r\n");

        f = (int8_t)scan_rot(SCAN_DEG, d1);          /* 先朝 d1 扫 30 */
        if (f < 0) return 0;
        if (f == 1) goto found;

        f = (int8_t)scan_rot(SCAN_DEG * 2.0f, -d1);  /* 反 60（越过中线到另一侧 30） */
        if (f < 0) return 0;
        if (f == 1) goto found;

        f = (int8_t)scan_rot(SCAN_DEG, d1);          /* 回正 */
        if (f < 0) return 0;
        if (f == 1) goto found;

        uart_str("    no line in +-30deg, back ");
        uart_int((int32_t)(SCAN_BACK_CM * 10.0f));
        uart_str("mm then scan again\r\n");

        f = (int8_t)scan_back(SCAN_BACK_CM);
        if (f < 0) return 0;
        if (f == 1) goto found;
    }

    uart_str("  ** [align] FAIL: still no line after ");
    uart_int(SCAN_MAX_ROUNDS);
    uart_str(" rounds, stopping here **\r\n");
    motor_stop();
    return -1;

found:
    uart_str("  [align] line found, continue\r\n");
    motor_stop();
    delay_ms(10);
    return 1;
}

/* 老调用点（转弯后找线）：仍沿用上一次转弯方向做提示 */
static int8_t scan_find_line(const char *why)
{
    return scan_find_line_ex(why, 0);
}

static int8_t drive_pid(float target_cm, int leg_no, uint8_t stop_at_end)
{
    float done = 0.0f;
    uint32_t t0 = millis(), t_stall = millis(), t_report = millis();
    uint32_t t_lost = 0, t_junc = 0;
    /* 超时按段长给：保底 10s + 每厘米 150ms（约 6.7cm/s 的怂速），
       固定 30s 对 339cm 那段余量太小，电池一软就误报 */
    uint32_t leg_to = 10000u + (uint32_t)(target_cm * 150.0f);
    float stall_ref = 0.0f;
    float e_prev = 0.0f, e_int = 0.0f;
    int32_t last_pid = 0;
    uint8_t e_first = 1;      /* 首帧只记 e_prev，不算 D，避免一进段就猛打方向 */
    uint8_t no_pid_last = 0;
    /* 左右轮单独监视：PWM 给了却没转，就是硬件/驱动的问题 */
    float acc_l = 0.0f, acc_r = 0.0f;
    int32_t last_pl = 0, last_pr = 0;
    uint8_t wd_l = 0, wd_r = 0;
    int16_t last_e = 0;
    /* 本段有没有"在线过"。没见过线 -> 说明这一段一开头就没对上线，才允许段内原地扫；
       已经在线过再丢线 -> 那是赛道本身的断口（短段跨接缝很常见），绝不能停车乱转 */
    uint8_t seen_line = 0;

    /* 上一动作（转弯/找线）留下的计数先丢掉：找线的路程不计入里程表 */
    { float jl0, jr0; odom_take2(&jl0, &jr0); }

    /* 环岛段：整套增益、限幅、基准速度都换掉 */
    uint8_t is_ring = (uint8_t)(RING_PID && (leg_no == RING_LEG));
    float   kp  = is_ring ? RING_KP : KP;
    float   kd  = is_ring ? RING_KD : KD;
    float   smx = is_ring ? (float)RING_STEER_MAX : (float)STEER_MAX;
    int32_t spd = is_ring ? RING_SPEED : SPEED_LEG;
    int32_t wmn = is_ring ? RING_WHEEL_MIN : WHEEL_MIN;

    if (is_ring) {
        uart_str("  [RING] aggressive PID: kp=");
        uart_int((int32_t)(kp * 20.0f));
        uart_str(" kd=");
        uart_int((int32_t)(kd * 20.0f));
        uart_str(" steer_max=");
        uart_int((int32_t)smx);
        uart_str(" speed=");
        uart_int(spd);
        uart_str(" wheel_min=");
        uart_int(wmn);
        uart_str("\r\n");
    }

    while (done < target_cm) {
        if (poll_cmd()) { motor_stop(); return 0; }
        wait_if_paused();

        /* --- 1. 里程 --- */
        float dl, dr;
        odom_take2(&dl, &dr);
        done += (dl + dr) * 0.5f;
        acc_l += dl; acc_r += dr;

        /* --- 2. 循迹误差（模拟量加权） --- */
        int16_t vs = 0, non = 0;
        int16_t e = sense_err(&vs, &non);

        if (e == 999) {                       /* 八路全蓝：丢线 */
            if (t_lost == 0) t_lost = millis();
            if (is_ring && RING_EXIT_LOST > 0u && done > 50.0f &&
                (uint32_t)(millis() - t_lost) > RING_EXIT_LOST) {
                uart_str("  [RING] line lost after ");
                uart_int((int32_t)(done * 10.0f));
                uart_str("mm -> assume ring exit\r\n");
                break;
            }
            {
                /* v1.6.5 位置判定：段首/段尾各 LEG_EDGE_CM 之内不扫线，其余立刻扫 */
                float remain = target_cm - done;
                uint8_t mid = (uint8_t)((done > LEG_EDGE_CM && remain > LEG_EDGE_CM &&
                                         !in_no_scan(leg_no, done)) ? 1 : 0);
                uint32_t trig = mid ? MID_LOST_MS : LOST_MS;

                if ((uint32_t)(millis() - t_lost) > trig) {
                    if (mid) {
                        /* 段中丢线：立刻进"扫线+后退"模式，不再直着走。
                           第一次扫线固定朝最后一次 PID 修正的反方向 */
                        int8_t af;
                        int8_t first = (int8_t)(-g_last_pid_dir);
                        motor_stop();
                        uart_str("  [align] mid lost @");
                        uart_int((int32_t)(done * 10.0f));
                        uart_str("mm\r\n");
                        af = scan_find_line_ex("mid lost", first);
                        if (af == 0) { motor_stop(); return 0; }
                        if (af < 0)  { motor_stop(); return -1; }
                        /* 找线时的转动/后退不计入本段里程 */
                        { float jl, jr; odom_take2(&jl, &jr); }
                        acc_l = 0.0f; acc_r = 0.0f;
                        wd_l = 0; wd_r = 0;
                        t_lost = 0; t_stall = millis(); stall_ref = done;
                        e_first = 1;
                        e = 0;
                    } else if (done <= LEG_EDGE_CM && !seen_line) {
                        /* 段首 20cm 内且本段一次线都没见过（转弯没对上）：原地找线 */
                        int8_t af;
                        motor_stop();
                        af = scan_find_line("head lost");
                        if (af == 0) { motor_stop(); return 0; }
                        if (af < 0)  { motor_stop(); return -1; }
                        { float jl, jr; odom_take2(&jl, &jr); }
                        acc_l = 0.0f; acc_r = 0.0f;
                        wd_l = 0; wd_r = 0;
                        t_lost = 0; t_stall = millis(); stall_ref = done;
                        e_first = 1;
                        e = 0;
                    } else {
                        e = 0;      /* 段首/段尾 20cm 内：不扫线，保持直行 */
                        e_first = 1;
                    }
                }
                else                                     e = last_e;/* 还没判死，先按上次的方向修 */
            }
        } else {
            t_lost = 0;
            last_e = e;
            seen_line = 1;
        }

        /* --- 3. 横向宽白（路口/横线）：PID 暂停，免得被带偏 --- */
        if (non >= JUNC_POP) t_junc = millis();
        uint8_t frozen = (t_junc && (uint32_t)(millis() - t_junc) < JUNC_HOLD_MS);

        /* --- 4. PID（直行只需要 P 和 D，积分先不用） --- */
        float pid = 0.0f;
        if (e != 999 && !frozen) {
            if (e_first) { e_prev = (float)e; e_first = 0; }
            float de = (float)e - e_prev;
            e_int += (float)e;
            pid = kp * (float)e + KI * e_int + kd * de;
            if (is_ring) {                       /* 大偏差（快偏到边上）再硬推一记 */
                if (e >= 25)      pid += (float)RING_BOOST;
                else if (e <= -25) pid -= (float)RING_BOOST;
            }
            if (pid >  smx) pid =  smx;
            if (pid < -smx) pid = -smx;
            e_prev = (float)e;
        } else {
            /* 丢线或路口冻结：这一帧不算 D，恢复后第一帧也不砸，
               免得 200ms 的累积差一次性变成方向冲击 */
            e_first = 1;
        }

        /* 岔路区间：进区间后 PID 归零，纯靠里程走直（只打一次状态变化） */
        uint8_t no_pid = (uint8_t)((is_ring && !RING_USE_NOPID) ? 0 : in_nopid(leg_no, done));
        if (no_pid != no_pid_last) {
            no_pid_last = no_pid;
            if (no_pid) uart_str("  [no-pid ON] straight by odometry\r\n");
            else        uart_str("  [no-pid OFF] pid back on\r\n");
        }
        if (no_pid) {
            pid = 0.0f;
            e_first = 1;
        }

        last_e = e;
        last_pid = (int32_t)pid;

        /* --- 5. 速度 + 差速 --- */
        int32_t sp = g_sprint ? SPEED_LAST : speed_for(target_cm - done, spd);

        int32_t steer = (int32_t)(pid * STEER_SIGN);
        if (steer > 10)       g_last_pid_dir = 1;    /* 往右修 */
        else if (steer < -10) g_last_pid_dir = -1;   /* 往左修 */
        int32_t pr = sp - steer;      /* err>0（线在右）→ 右轮慢一点 → 车往右拐 */
        int32_t pl = sp + steer;

        /* 钳位不能白钳：一轮被顶到 WHEEL_MIN 时，另一轮要相应退回来，
           否则 (pr+pl)/2 会比 sp 大 —— 急弯时车反而加速，弯里更容易冲出去 */
        if (pr < wmn) { pr = wmn; pl = 2 * sp - pr; }
        if (pl < wmn) { pl = wmn; pr = 2 * sp - pl; }
        /* 顶到 999 时只夹自身、不再反算另一轮：
           否则 pl=999+steer 被夹回 999 后 pr 又被反算成 999，差速全丢，满速时打不了方向 */
        if (pr > 999) pr = 999;
        if (pl > 999) pl = 999;
        if (pr < 0) pr = 0;
        if (pl < 0) pl = 0;

        motor_set(pr, pl);
        last_pr = pr; last_pl = pl;

        /* --- 6. 保护 --- */
        if (done - stall_ref > 0.2f) {
            stall_ref = done;
            t_stall = millis();
        } else if ((uint32_t)(millis() - t_stall) > STALL_MS) {
            motor_stop();
            uart_str("  ** STALL: encoder not moving (wiring/battery/blocked) **\r\n");
            return -1;
        }
        if ((uint32_t)(millis() - t0) > leg_to) {
            motor_stop();
            uart_str("  ** TIMEOUT: leg took too long **\r\n");
            return -1;
        }

        /* --- 7. 打印 --- */
        if ((uint32_t)(millis() - t_report) >= REPORT_MS) {
            t_report = millis();
            uart_str("    ");
            uart_num("d=", (int32_t)(done * 10.0f));
            uart_str("/");
            uart_int((int32_t)(target_cm * 10.0f));
            uart_num("mm sp=", sp);
            if (is_ring) uart_str(" RING");
            uart_num(" e=", (e == 999 ? 999 : (int32_t)e));
            uart_num(" pid=", last_pid);
            uart_num(" v=", vs);
            uart_str(" n=");
            uart_int(non);
            uart_str("\r\n");
            uart_str("      L pwm=");
            uart_int(last_pl);
            uart_str(" dL=");
            uart_int((int32_t)(acc_l * 10.0f));
            uart_str("mm  R pwm=");
            uart_int(last_pr);
            uart_str(" dR=");
            uart_int((int32_t)(acc_r * 10.0f));
            uart_str("mm\r\n");

            /* 看门狗：给了 >=200 的 PWM，这 500ms 里编码器却几乎没动 */
            if (last_pl >= 200 && acc_l < 0.15f) wd_l++; else wd_l = 0;
            if (last_pr >= 200 && acc_r < 0.15f) wd_r++; else wd_r = 0;
            if (wd_l >= 2) {
                uart_str("      !! LEFT wheel: pwm>0 but no encoder motion (wire/motor/driver/B-channel)\r\n");
                wd_l = 0;
            }
            if (wd_r >= 2) {
                uart_str("      !! RIGHT wheel: pwm>0 but no encoder motion (wire/motor/driver/A-channel)\r\n");
                wd_r = 0;
            }
            acc_l = 0.0f; acc_r = 0.0f;
        }

        led_set(1);
        delay_ms(2);
        led_set(0);
    }

    if (stop_at_end) {
        motor_stop();
        delay_ms(STOP_BRAKE_MS);
    }
    /* stop_at_end == 0：这一段后面紧跟不停车转弯，保持速度，别刹车 */
    return 1;
}

/* ==================== 十、转弯：纯里程表 ==================== */

/* deg > 0 右转，deg < 0 左转 */
static int8_t turn_deg(float deg)
{
    if (deg == 0.0f) return 1;

    float need = 3.14159265f * WHEEL_BASE_CM * (deg > 0 ? deg : -deg) / 360.0f;
    need *= TURN_SCALE;

    float got = 0.0f;
    uint32_t t0 = millis(), t_stall = millis();
    float stall_ref = 0.0f;
    int32_t dir = (deg > 0) ? 1 : -1;

    /* 起转前丢掉停车滑行留下的计数 */
    { float jl, jr; odom_take2(&jl, &jr); }

    uart_str("  turn ");
    uart_int((int32_t)deg);
    uart_str(" deg, need=");
    uart_int((int32_t)(need * 10.0f));
    uart_str("mm per wheel\r\n");

    while (got < need) {
        if (poll_cmd()) { motor_stop(); return 0; }
        wait_if_paused();

        float dl, dr;
        odom_take2(&dl, &dr);

        /* 两轮位移差才是转角；直行滑行时差≈0，不计入 */
        float d = (dir > 0) ? (dl - dr) : (dr - dl);
        if (d > 0.0f) got += d * 0.5f;

        int32_t sp = speed_for(need - got, SPEED_TURN);
        if (dir > 0) motor_set(-sp,  sp);
        else         motor_set( sp, -sp);

        if (got - stall_ref > 0.2f) { stall_ref = got; t_stall = millis(); }
        else if ((uint32_t)(millis() - t_stall) > STALL_MS) {
            motor_stop();
            uart_str("  ** STALL: cannot turn **\r\n");
            return -1;
        }
        if ((uint32_t)(millis() - t0) > TURN_TIMEOUT_MS) {
            motor_stop();
            uart_str("  ** TIMEOUT: turn timeout **\r\n");
            return -1;
        }

        led_set(1);
        delay_ms(2);
        led_set(0);
    }

    motor_stop();
    delay_ms(STOP_BRAKE_MS);
    uart_str("  turn done, walked=");
    uart_int((int32_t)(got * 10.0f));
    uart_str("mm\r\n");
    g_last_turn_dir = (int8_t)dir;      /* 丢线时优先朝这个方向摸 */
    g_trim_cm = 0.0f;                   /* 原地转没有前进，不用扣 */
    return 1;
}

/* ============ 不停车转弯：边走边转（v1.4.0）============
 * dir = deg 的符号：>0 右转（顺时针），<0 左转。
 *
 * 右转时：左轮=外轮=v_out，右轮=内轮=v_in，两轮都为正（都往前）。
 * 角度依旧用两轮位移差：d = (dl - dr)（右转）/ (dr - dl)（左转），
 * 与原地转是同一个运动学关系 ω = (v_out - v_in)/B，所以 need 公式不变。
 */
static int8_t turn_roll(int16_t deg)
{
    if (deg == 0) return 1;

    float adeg = (deg > 0) ? (float)deg : -(float)deg;
    float need = 3.14159265f * WHEEL_BASE_CM * adeg / 360.0f * TURN_SCALE_ROLL;
    float r = WHEEL_BASE_CM / (2.0f * TURN_ROLL_R);
    int32_t v_out = (int32_t)(TURN_ROLL_PWM * (1.0f + r));
    int32_t v_in  = (int32_t)(TURN_ROLL_PWM * (1.0f - r));
    int32_t dir = (deg > 0) ? 1 : -1;
    float got = 0.0f;
    uint32_t t0, t_stall;
    float stall_ref = 0.0f;

    if (v_out > 999) v_out = 999;
    if (v_in  > 999) v_in  = 999;
    if (v_in  < 0)   v_in  = 0;

    uart_str("  roll-turn ");
    uart_int((int32_t)deg);
    uart_str(" deg, need=");
    uart_int((int32_t)(need * 10.0f));
    uart_str("mm  L=");
    uart_int(v_out);
    uart_str(" R=");
    uart_int(v_in);
    uart_str("\r\n");

    { float jl, jr; odom_take2(&jl, &jr); }   /* 丢掉之前的计数（车本身还在动） */
    t0 = millis();
    t_stall = t0;

    while (got < need) {
        if (poll_cmd()) { motor_stop(); return 0; }
        wait_if_paused();

        float dl, dr, d;
        odom_take2(&dl, &dr);
        d = (dir > 0) ? (dl - dr) : (dr - dl);
        if (d > 0.0f) got += d * 0.5f;

        if (dir > 0) motor_set(v_in,  v_out);   /* 右转：右内 左外 */
        else         motor_set(v_out, v_in);    /* 左转：左内 右外 */

        if (got - stall_ref > 0.2f) { stall_ref = got; t_stall = millis(); }
        else if ((uint32_t)(millis() - t_stall) > STALL_MS) {
            motor_stop();
            uart_str("  ** STALL: roll turn **\r\n");
            return -1;
        }
        if ((uint32_t)(millis() - t0) > TURN_TIMEOUT_MS) {
            motor_stop();
            uart_str("  ** TIMEOUT: roll turn **\r\n");
            return -1;
        }

        led_set(1);
        delay_ms(2);
        led_set(0);
    }

    motor_stop();
    delay_ms(STOP_BRAKE_MS);
    uart_str("  roll done, walked=");
    uart_int((int32_t)(got * 10.0f));
    uart_str("mm\r\n");

    /* 这一段圆弧让车沿"新方向"已经前进 adv，下一段的里程要扣掉它 */
    {
        float th = adeg * 3.14159265f / 180.0f;
        float adv = TURN_ROLL_R * sinf(th) - TURN_ROLL_ADV * cosf(th);
        if (adv < 0.0f) adv = 0.0f;
        if (adv > 80.0f) adv = 80.0f;
        g_trim_cm = TURN_TRIM_NEXT ? adv : 0.0f;
        uart_str("  rolled forward ");
        uart_int((int32_t)(g_trim_cm * 10.0f));
        uart_str("mm along the next leg (will be trimmed)\r\n");
    }

    g_last_turn_dir = (int8_t)dir;
    return 1;
}

/* ==================== 十一、路线表 ==================== */

typedef struct { uint16_t cm; int16_t turn; } Leg;

static const Leg LEGS[] = {
    {  85, +90 },   /*  1 N21→N1 */
    {  48, +90 },   /*  2 N1→N2 */
    { 205, -90 },   /*  3 N2→N59 */
    {  30, -90 },   /*  4 N59→N60 */
    {  40, +90 },   /*  5 N60→N52 */
    { 140, -90 },   /*  6 N52→N55 */
    {  24, -90 },   /*  7 N55→N41 */
    { 140, +90 },   /*  8 N41→N38 */
    { 90, +90 },   /*  9 N38→N13 */
    { 140, -90 },   /* 10 N13→N16 */
    {  32, +90 },   /* 11 N16→N6 */
    {  25, +90 },   /* 12 N6→N7 */
    {  38, -90 },   /* 13 N7→N17 */
    {  94, +90 },   /* 14 N17→N20 */
    {  65, +90 },   /* 15 N20→N35 */
    {  25, -90 },   /* 16 N35→N33 */
    {  45, +30 },   /* 17 N33→N49 */
    /* 环岛：一段到底，方向交给激进 PID 循线（原 6 段合计 173cm） */
    { 175, +30 },   /* 18 RING (aggressive PID) */
    {  40, +90 },   /* 19 N49→N65 */
    {  95, +90 },   /* 20 N65→N63 */
    { 135, -90 },   /* 21 N63→N25 */
    {  72, +90 },   /* 22 N25→N24 */
    {  31, -90 },   /* 23 N24→N15 */
    {  62, +90 },   /* 24 N15→N13 */
    {  30, +90 },   /* 25 N13→N3 */
    { 275, +90 },   /* 26 N3→N10 */
    { 215, +90 },   /* 27 N10→N67 */
    { 365, +90 },   /* 28 N67→N58 */
    { 110,  +0 },   /* 29 N58→N28 */
};

#define LEG_N ((int)(sizeof(LEGS) / sizeof(LEGS[0])))

/* ==== 岔路区间：这些里程范围内 PID 关掉，纯靠里程走直 ==== */
typedef struct { uint8_t leg; float from; float to; } NoPidZone;

static const NoPidZone NOPID_ZONES[] = {
    {  3,  125.0,  140.0 },
    {  5,   20.0,  999.0 },
    {  6,   20.0,   35.0 },
    {  7,    0.0,  999.0 },
    {  8,  110.0,  999.0 },
    { 10,   60.0,   75.0 },
    { 10,  130.0,  140.0 },
    { 11,   20.0,  999.0 },
    { 12,    0.0,  999.0 },
    { 13,   20.0,  999.0 },
    { 14,   60.0,  999.0 },
    { 16,   10.0,  999.0 },
    { 17,   25.0,  999.0 },
    { 18,  150.0,  999.0 },
    { 19,   25.0,  999.0 },
    { 21,   50.0,   90.0 },
    { 21,  120.0,  135.0 },
    { 25,   10.0,  999.0 },
    { 26,  140.0,  180.0 },
    { 26,  220.0,  265.0 },
    { 27,   30.0,   40.0 },
    { 27,   60.0,   90.0 },
    { 28,    0.0,   10.0 },
    { 28,  120.0,  140.0 },
    { 28,  220.0,  280.0 },
};
#define NOPID_N ((int)(sizeof(NOPID_ZONES) / sizeof(NOPID_ZONES[0])))

static uint8_t in_nopid(int leg, float done)
{
    int k;
    for (k = 0; k < NOPID_N; k++) {
        if (NOPID_ZONES[k].leg == (uint8_t)leg &&
            done >= NOPID_ZONES[k].from && done < NOPID_ZONES[k].to) return 1u;
    }
    return 0u;
}

/* ---- 段中"立刻找线"的禁用区间（v1.6.5）----
 * 有的位置本来就该丢线（虚线/断口/大横线），在那里停车扫线只会把车拧歪，
 * 所以列进这张表：落在区间内时，丢线仍按"保持直行"处理，不扫线。
 */
static const NoPidZone NO_SCAN_ZONES[] = {
    {  3,  150.0,  9999.0 },   /* 第3段 = 210cm，最后 60cm(150..210) 是虚线：不扫线 */
};
#define NOSCAN_N ((int)(sizeof(NO_SCAN_ZONES) / sizeof(NO_SCAN_ZONES[0])))

static uint8_t in_no_scan(int leg, float done)
{
    int k;
    for (k = 0; k < NOSCAN_N; k++) {
        if (NO_SCAN_ZONES[k].leg == (uint8_t)leg &&
            done >= NO_SCAN_ZONES[k].from && done < NO_SCAN_ZONES[k].to) return 1u;
    }
    return 0u;
}

/* ==================== 十二、主程序 ==================== */

/* ---- 分段调试：不动串口也能用，改这里重新编译即可 ---- */
#define START_LEG_DEFAULT   1     /* 从第几段开始跑（1..34）；也可以上电 1 秒内用串口发 s12 */
#define ONLY_LEG_DEFAULT    0     /* 1 = 只跑起始那一段就停，用来单独试某一段 */

/* 电机单独自检：每相 1.2s，分别报左右轮的里程增量。
 * 用来区分"左轮不转"是硬件（给了 PWM 也不动）还是软件（PWM 就是 0）。 */
static void motor_test(void)
{
    /* 每一相 1.2 秒：先右轮正转、再左轮正转、再各反转，最后双轮正转。
     * 左轮「正转不动、反转能动」= BIN1/BIN2 里有一根不通；
     * 正反都不动 = PWM(PWMB) 或电机/驱动那一路整条不通。 */
    static const char *tag[5] = { "RIGHT +   pwm=300", "LEFT  +   pwm=300",
                                  "RIGHT -   pwm=300", "LEFT  -   pwm=300",
                                  "BOTH  +   pwm=300" };
    static const int32_t pr[5] = { 300, 0, -300, 0, 300 };
    static const int32_t pl[5] = { 0, 300, 0, -300, 300 };
    int k;
    uart_str("\r\n=== MOTOR TEST (power cycle to exit) ===\r\n");
    for (k = 0; k < 5; k++) {
        float l = 0.0f, r = 0.0f;
        motor_set(pr[k], pl[k]);
        odom_take2(&l, &r);                 /* 丢掉切换瞬间的残留 */
        delay_ms(1200);
        odom_take2(&l, &r);
        uart_str(tag[k]);
        uart_str("  L mm=");
        uart_int((int32_t)(l * 10.0f));
        uart_str("  R mm=");
        uart_int((int32_t)(r * 10.0f));
        uart_str("\r\n");
        motor_stop();
        delay_ms(300);
    }
    uart_str("=== done: LEFT + should be POSITIVE, LEFT - NEGATIVE ===\r\n");
}

static int g_start_leg = START_LEG_DEFAULT;
static int g_only_leg  = ONLY_LEG_DEFAULT;

int main(void)
{
    sys_tick_init();
    cyc_init();
    uart_init();
    led_init();
    motor_gpio_dir();
    pwm_init();
    enc_init();
    adc_init();
    cal_default();
    motor_stop();

    /* 上电闪灯报版本：6 短闪 = v0.6.x。不接串口也能确认固件是新的 */
    for (int k = 0; k < 6; k++) { led_set(1); delay_ms(90); led_set(0); delay_ms(110); }

    uart_str("\r\n\r\n");
    uart_str("=== analog_route v1.4.1  rolling turn + next-leg trim ===\r\n");
    uart_num("legs=", LEG_N);
    uart_str("\r\n");

    int32_t total_cm = 0;
    for (int i = 0; i < LEG_N; i++) total_cm += LEGS[i].cm;
    uart_num("route total=", total_cm);
    uart_str("cm\r\n");
    /* 环岛这一段的长度直接从表里取出来打印：改表里那个数，这里必须跟着变 */
    uart_num("RING leg=", RING_LEG);
    uart_str("  len=");
    uart_int((int32_t)(LEGS[RING_LEG - 1].cm));
    uart_str("cm  turn=");
    uart_int((int32_t)(LEGS[RING_LEG - 1].turn));
    uart_str("  exit_lost=");
    uart_int((int32_t)RING_EXIT_LOST);
    uart_str("ms  use_nopid=");
    uart_int((int32_t)RING_USE_NOPID);
    uart_str("\r\n");
    uart_str("  (if len here does not change when you edit LEGS[], you are building another file)\r\n");
    /* 段中找线的禁用区间：只作用于 drive_pid 里的"段中丢线"分支。
     * 转弯后的 scan_find_line("after turn") 不受它影响，永远执行。 */
    for (int k = 0; k < NOSCAN_N; k++) {
        uart_str("no-scan(mid-leg only): leg=");
        uart_int(NO_SCAN_ZONES[k].leg);
        uart_str("  from=");
        uart_int((int32_t)(NO_SCAN_ZONES[k].from * 10.0f));
        uart_str("mm  to=");
        uart_int((int32_t)(NO_SCAN_ZONES[k].to * 10.0f));
        uart_str("mm\r\n");
    }

    uart_num("roll-turn=", TURN_ROLL ? 1 : 0);
    uart_num("  R(cm x10)=", (int32_t)(TURN_ROLL_R * 10.0f));
    uart_num("  adv(cm x10)=", (int32_t)(TURN_ROLL_ADV * 10.0f));
    uart_num("  pwm=", TURN_ROLL_PWM);
    uart_str("\r\n");
    uart_num("last-leg sprint=", FINAL_SPRINT ? SPEED_LAST : 0);
    uart_str("  (PWM, 0=off)\r\n");
    uart_num("KP=", (int32_t)(KP * 10.0f));
    uart_num("  KD=", (int32_t)(KD * 10.0f));
    uart_num("  TURN_SCALE=", (int32_t)(TURN_SCALE * 100.0f));
    uart_str("/100  SPEED=");
    uart_int(SPEED_LEG);
    uart_str("\r\n");

    /* adc_lab();  2026-11-13 已确认：校准后 DR 正常，EOC 不可用 */
    adc_scan();
    {
        int16_t vs0 = 0, n0 = 0;
        int16_t e0 = sense_err(&vs0, &n0);
        uart_num("e=", (e0 == 999 ? 999 : (int32_t)e0));
        uart_num("  v=", vs0);
        uart_num("  n=", n0);
        uart_str("   (e=999 means no line under sensors)\r\n");
    }
    uart_str("adc now:");
    for (int i = 0; i < SENSOR_N; i++) { uart_char(' '); uart_int(g_adc[i]); }
    uart_str("\r\n");

#if MUX_FT_PINS
    uart_str("mux(5V): AD0->PB10 AD1->PB11 AD2->PA15 (open-drain + 10k to 5V)\r\n");
    uart_str("         OUT->PA3 via 10k/20k divider (0..5V -> 0..3.3V)\r\n");
#else
    uart_str("mux(3V3): AD0->PA0 AD1->PA1 AD2->PA2, OUT->PA3 (ADC1_IN3)\r\n");
#endif
    if (!g_cal_ok) {
        uart_str("WARN: not calibrated, using defaults (press 'k' to calibrate)\r\n");
    }
    uart_str("AUTO START in 1s (no key needed).\r\n");
    uart_str("  'k' within 1s -> calibrate, then run automatically\r\n");
    uart_str("  'x' -> stop\r\n");
    uart_str("  't' -> motor self-test (LEFT/RIGHT/BOTH), then halt\r\n");
    uart_str("  's12' -> start from leg 12, run to the end\r\n");
    uart_str("  'o12' -> run ONLY leg 12, then stop\r\n");
    uart_str("  after every turn: if no sensor sees the line, scan in place\r\n");
    uart_str("  (+-30deg; if nothing, back 5cm and scan again)\r\n");
    uart_str("  (put the car at the start of that leg, facing forward)\r\n");

    /* 上电 1 秒倒计时：不按任何键就自动开跑 */
    {
        uint32_t t0 = millis(), wait = AUTO_START_MS;
        int num = 0, has_num = 0;
        char mode = 0;
        while ((uint32_t)(millis() - t0) < wait && !g_abort) {
            int c = uart_try_get();
            if (c >= 0) {
                t0 = millis(); wait = 1500u;      /* 收到一个字符就再等 1.5s，方便输多位数 */
                if (c >= '0' && c <= '9') {
                    num = num * 10 + (c - '0');
                    if (num > 99) num = 99;
                    has_num = 1;
                } else if (c == 's' || c == 'S') {
                    mode = 's';                   /* s12 = 从第 12 段开始，跑到最后 */
                } else if (c == 'o' || c == 'O') {
                    mode = 'o';                   /* o12 = 只跑第 12 段 */
                } else if (c == 'k' || c == 'K') {
                    motor_stop();
                    cal_do();                     /* 标定完直接进 GO，不用再按 g */
                    break;
                } else if (c == 't' || c == 'T') {
                    motor_stop(); motor_test();
                    uart_str("power cycle to run normally\r\n");
                    g_abort = 1; break;
                } else if (c == 'c' || c == 'C') {
                    motor_stop(); sensor_watch();
                } else if (c == 'x' || c == 'X') {
                    g_abort = 1; break;
                }
            }
            led_set(((millis() / 100u) & 1u) ? 0 : 1);
            delay_ms(5);
        }
        if (has_num) {
            if (num >= 1 && num <= LEG_N) {
                g_start_leg = num;
                g_only_leg  = (mode == 'o') ? 1 : 0;
            } else {
                uart_str("!! leg number out of range, keep start=1\r\n");
            }
        }
        led_set(0);
    }

    if (g_abort) {
        motor_stop();
        uart_str("aborted before start\r\n");
        while (1) { motor_stop(); delay_ms(500); }
    }

    uart_num("start leg=", g_start_leg);
    if (g_only_leg) uart_str("  (this leg only)\r\n");
    else            uart_str("  (to the end)\r\n");
    uart_str("--- GO ---\r\n");
    delay_ms(LEG_SETTLE_MS);

    float odom_total = 0.0f;
    uint32_t t_start = millis();
    int8_t rc = 1;

    int i_start = g_start_leg - 1;
    if (i_start < 0) i_start = 0;
    if (i_start > LEG_N - 1) i_start = LEG_N - 1;
    int i_end = g_only_leg ? (i_start + 1) : LEG_N;

    for (int i = i_start; i < i_end && rc > 0; i++) {
        uart_str("[L");
        uart_int(i + 1);
        uart_str("] cm=");
        uart_int(LEGS[i].cm);
        uart_str(" turn=");
        uart_int(LEGS[i].turn);
        uart_str("\r\n");

        g_sprint = (uint8_t)((FINAL_SPRINT && (i == LEG_N - 1) && LEGS[i].turn == 0) ? 1 : 0);
        if (g_sprint) uart_str("  SPRINT: full speed to the finish\r\n");

        /* 不停车转弯：这一段提前 TURN_ROLL_ADV 就开始转，末尾也不刹车；
           同时把上一次滚转已经走出去的那段从本段里程里扣掉 */
        float leg_target = (float)LEGS[i].cm - g_trim_cm;
        if (g_trim_cm > 0.0f) {
            uart_str("  (trim this leg by ");
            uart_int((int32_t)(g_trim_cm * 10.0f));
            uart_str("mm)\r\n");
        }
        g_trim_cm = 0.0f;

        uint8_t roll_now = (uint8_t)((TURN_ROLL && LEGS[i].turn != 0 && !g_sprint) ? 1 : 0);
        if (roll_now) leg_target -= TURN_ROLL_ADV;
        if (leg_target < 3.0f) {
            uart_str("  !! trim/adv exceeds this leg, clamped to 3cm\r\n");
            leg_target = 3.0f;
        }

        rc = drive_pid(leg_target, i + 1, (uint8_t)(!roll_now));
        if (rc <= 0) break;

        odom_total += (float)LEGS[i].cm;
        if (!roll_now) delay_ms(LEG_SETTLE_MS);

        if (LEGS[i].turn != 0) {
            rc = roll_now ? turn_roll(LEGS[i].turn) : turn_deg((float)LEGS[i].turn);
            if (rc <= 0) break;
            delay_ms(LEG_SETTLE_MS);

            /* 转完必须确认回到线上：不在线就原地找线，误差每段清零不累积。
             * 注意：这里不受 NO_SCAN_ZONES 影响 —— 禁用区间只管"段中丢线"，
             * 转向后无论落在哪一段都必须找线。 */
            rc = scan_find_line("after turn");
            if (rc <= 0) break;
            delay_ms(LEG_SETTLE_MS);
        }

        uart_str("  ok, total=");
        uart_int((int32_t)odom_total);
        uart_str("cm, t=");
        uart_int((int32_t)((millis() - t_start) / 1000u));
        uart_str("s\r\n");
    }

    motor_stop();

    if (rc > 0)       uart_str("\r\n=== ROUTE FINISHED ===\r\n");
    else if (rc == 0) uart_str("\r\n=== ABORTED by 'x' ===\r\n");
    else              uart_str("\r\n=== STOPPED (stall/timeout) ===\r\n");

    uart_num("total=", (int32_t)odom_total);
    uart_str("cm, time=");
    uart_int((int32_t)((millis() - t_start) / 1000u));
    uart_str("s\r\n");
    uart_str("=== halted, power cycle to run again ===\r\n");

    while (1) {
        motor_stop();
        led_set(1);
        delay_ms(300);
        led_set(0);
        delay_ms(700);
    }
}
