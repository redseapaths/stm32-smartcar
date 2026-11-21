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
#define SPEED_LEG       340       /* 直行基准速度（PID 的基准） */
#define SPEED_TURN      380       /* 原地转 */
#define SPEED_MIN       200       /* 快到位时降到这么慢 */
#define WHEEL_MIN       150       /* 内侧轮最低速，别让它停死被拖 */
#define DECEL_CM        5.0f      /* 差这么多厘米开始减速 */

/* ---------- 循迹 PID ---------- */
/* err 是按位加权算出来的位置，范围大约 -35..+35：
 *   全部在中间 → 0；偏一个传感器 → ±5；偏到边上 → ±35            */
#define KP              5.0f
#define KI              0.0f      /* 先不用积分，免得越积越歪 */
#define KD              6.0f
#define STEER_MAX       400       /* PID 输出限幅（PWM 单位） */
#define STEER_SIGN      (+1)      /* 车往反方向跑就改成 -1 */

/* 传感器：如果 PA0..PA7 与 X1..X8 是反着接的，设 1（2026-11-10 实测确认为 1） */
#define SENSOR_REVERSE  1
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
#define LEG_SETTLE_MS   300
#define LEG_TIMEOUT_MS  30000u
#define TURN_TIMEOUT_MS 15000u
#define STALL_MS        800u

/* 调试打印间隔 */
#define REPORT_MS       500u

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
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 8)) | (0x4u << 8);   /* PA10 浮空输入 */
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
    if (v < 0) { uart_char('-'); v = -v; }
    if (v == 0) { uart_char('0'); return; }
    while (v > 0 && i < 11) { buf[i++] = (char)('0' + v % 10); v /= 10; }
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
    GPIOB->BSRR = (1u << 8) | (1u << 9);

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
    uint8_t st = 0;
    st |= (GPIOB->IDR & (1u << 8)) ? 0x02u : 0;
    st |= (GPIOB->IDR & (1u << 9)) ? 0x01u : 0;
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

/* ==================== 七、八路循迹（数字量） ==================== */

static uint8_t g_lm_prev = 0;
static uint8_t g_lm_cnt[8] = { 0 };

static void track_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    /* CRL 这 32 位**全部**属于 PA0..PA7，没有别的位可保，所以直接整片写是对的；
       PA8 以上在 CRH 里，pwm_init 单独配置，互不影响 */
    GPIOA->CRL  = 0x88888888u;      /* PA0..PA7：上拉/下拉输入 */
    GPIOA->BSRR = 0x00FFu;          /* 置 ODR = 1 → 上拉 */
    delay_ms(1);
}

static uint8_t track_raw(void) { return (uint8_t)(GPIOA->IDR & 0xFFu); }

/* bit0<->bit7 对调（硬件接线是反的） */
static uint8_t rev8(uint8_t v)
{
    uint8_t r = v;
    r = (uint8_t)(((r & 0xAAu) >> 1) | ((r & 0x55u) << 1));
    r = (uint8_t)(((r & 0xCCu) >> 2) | ((r & 0x33u) << 2));
    r = (uint8_t)((r >> 4) | (r << 4));
    return r;
}

static uint8_t line_mask(uint8_t raw)
{
#if SENSOR_REVERSE
    raw = rev8(raw);
#endif
    raw ^= TRACK_INV;
    return (uint8_t)(raw & TRACK_ENABLE_MASK);
}

/* 每路 3 帧确认，滤掉毛毛刺刺的单帧抖动 */
static uint8_t track_stable(void)
{
    uint8_t lm = line_mask(track_raw());
    uint8_t i;
    for (i = 0; i < 8u; i++) {
        uint8_t bit = (uint8_t)((lm >> i) & 1u);
        uint8_t old = (uint8_t)((g_lm_prev >> i) & 1u);
        if (bit == old) { g_lm_cnt[i] = 0; continue; }
        if (g_lm_cnt[i] < 2u) { g_lm_cnt[i]++; }
        else {
            if (bit) g_lm_prev |=  (uint8_t)(1u << i);
            else     g_lm_prev &= (uint8_t)~(1u << i);
            g_lm_cnt[i] = 0;
        }
    }
    return g_lm_prev;
}

static int pop8(uint8_t v)
{
    int n = 0;
    while (v) { n += (v & 1u); v = (uint8_t)(v >> 1); }
    return n;
}

/* 位置误差：-35(最左) .. +35(最右)，err>0 = 线在右边 */
static const int8_t W8[8] = { -35, -25, -15, -5, 5, 15, 25, 35 };

static int16_t track_err(uint8_t lm)
{
    int32_t s = 0, n = 0;
    uint8_t i;
    if (lm == 0u) return 999;            /* 丢线 */
    for (i = 0; i < 8u; i++) {
        if (lm & (uint8_t)(1u << i)) { s += W8[i]; n++; }
    }
    if (n == 0) return 999;
    return (int16_t)(s / n);
}

/* 传感器观察模式：不跑车，只看读数，方便判断接线/极性 */
static void sensor_watch(void)
{
    uart_str("\r\n=== SENSOR WATCH (# = on line, . = blue floor) ===\r\n");
    uart_str("block a sensor by hand, the bit should move; x = quit\r\n\r\n");
    while (1) {
        int c = uart_try_get();
        if (c == 'x' || c == 'X') { uart_str("quit watch, back to countdown\r\n"); return; }

        uint8_t raw = track_raw();
        uint8_t lm  = line_mask(raw);
        int16_t e   = track_err(lm);

        uart_num("raw=", (int32_t)raw);
        uart_str("  X1..X8: ");
        for (int i = 0; i < 8; i++) {
            uart_char((lm & (1u << i)) ? '#' : '.');
            uart_char(' ');
        }
        uart_num("  err=", (e == 999 ? 999 : (int32_t)e));
        uart_str("\r\n");

        led_set(1); delay_ms(75); led_set(0); delay_ms(75);
    }
}

/* ==================== 八、公共状态 ==================== */

static uint8_t g_abort  = 0;
static uint8_t g_paused = 0;

static uint8_t poll_cmd(void)
{
    int c;
    while ((c = uart_try_get()) >= 0) {
        if (c == 'x' || c == 'X') { g_abort = 1; }
        else if (c == 'p' || c == 'P') { g_paused = !g_paused; }
    }
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
static int8_t drive_pid(float target_cm)
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
    int16_t last_e = 0;

    while (done < target_cm) {
        if (poll_cmd()) { motor_stop(); return 0; }
        wait_if_paused();

        /* --- 1. 里程 --- */
        float dl, dr;
        odom_take2(&dl, &dr);
        done += (dl + dr) * 0.5f;

        /* --- 2. 循迹误差 --- */
        uint8_t lm  = track_stable();
        int16_t e   = track_err(lm);

        if (e == 999) {                       /* 八路全蓝：丢线 */
            if (t_lost == 0) t_lost = millis();
            if ((uint32_t)(millis() - t_lost) > LOST_MS) e = 0;      /* 放弃找线，直着走 */
            else                                         e = last_e;/* 先按上次的方向修 */
        } else {
            t_lost = 0;
            last_e = e;
        }

        /* --- 3. 横向宽白（路口/横线）：PID 暂停，免得被带偏 --- */
        if (pop8(lm) >= JUNC_POP) t_junc = millis();
        uint8_t frozen = (t_junc && (uint32_t)(millis() - t_junc) < JUNC_HOLD_MS);

        /* --- 4. PID（直行只需要 P 和 D，积分先不用） --- */
        float pid = 0.0f;
        if (e != 999 && !frozen) {
            if (e_first) { e_prev = (float)e; e_first = 0; }
            float de = (float)e - e_prev;
            e_int += (float)e;
            pid = KP * (float)e + KI * e_int + KD * de;
            if (pid >  STEER_MAX) pid =  STEER_MAX;
            if (pid < -STEER_MAX) pid = -STEER_MAX;
            e_prev = (float)e;
        } else {
            /* 丢线或路口冻结：这一帧不算 D，恢复后第一帧也不砸，
               免得 200ms 的累积差一次性变成方向冲击 */
            e_first = 1;
        }

        last_e = e;
        last_pid = (int32_t)pid;

        /* --- 5. 速度 + 差速 --- */
        int32_t sp = speed_for(target_cm - done, SPEED_LEG);

        int32_t steer = (int32_t)(pid * STEER_SIGN);
        int32_t pr = sp - steer;      /* err>0（线在右）→ 右轮慢一点 → 车往右拐 */
        int32_t pl = sp + steer;

        /* 钳位不能白钳：一轮被顶到 WHEEL_MIN 时，另一轮要相应退回来，
           否则 (pr+pl)/2 会比 sp 大 —— 急弯时车反而加速，弯里更容易冲出去 */
        if (pr < WHEEL_MIN) { pr = WHEEL_MIN; pl = 2 * sp - pr; }
        if (pl < WHEEL_MIN) { pl = WHEEL_MIN; pr = 2 * sp - pl; }
        if (pr > 999)       { pr = 999;       pl = 2 * sp - pr; }
        if (pl > 999)       { pl = 999;       pr = 2 * sp - pl; }
        if (pr < 0) pr = 0;
        if (pl < 0) pl = 0;

        motor_set(pr, pl);

        /* --- 6. 保护 --- */
        if (done - stall_ref > 0.2f || stall_ref - done > 0.2f) {
            stall_ref = done;
            t_stall = millis();
        } else if ((uint32_t)(millis() - t_stall) > STALL_MS) {
            motor_stop();
            uart_str("  ** STALL: 编码器不动了（检查接线/电池/是否顶住）**\r\n");
            return -1;
        }
        if ((uint32_t)(millis() - t0) > leg_to) {
            motor_stop();
            uart_str("  ** TIMEOUT: 这一段走太久了 **\r\n");
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
            uart_num(" e=", (e == 999 ? 999 : (int32_t)e));
            uart_num(" pid=", last_pid);
            uart_str(" lm=");
            for (int i = 0; i < 8; i++) uart_char((lm & (1u << i)) ? '1' : '0');
            uart_str("\r\n");
        }

        led_set(1);
        delay_ms(2);
        led_set(0);
    }

    motor_stop();
    delay_ms(150);
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
            uart_str("  ** STALL: 转不动 **\r\n");
            return -1;
        }
        if ((uint32_t)(millis() - t0) > TURN_TIMEOUT_MS) {
            motor_stop();
            uart_str("  ** TIMEOUT: 转弯超时 **\r\n");
            return -1;
        }

        led_set(1);
        delay_ms(2);
        led_set(0);
    }

    motor_stop();
    delay_ms(200);
    uart_str("  turn done, walked=");
    uart_int((int32_t)(got * 10.0f));
    uart_str("mm\r\n");
    return 1;
}

/* ==================== 十一、路线表 ==================== */

typedef struct { uint16_t cm; int16_t turn; } Leg;

static const Leg LEGS[] = {
    {  69, +90 },   /*  1 N21→N1 */
    {  42, +90 },   /*  2 N1→N2 */
    { 190, -90 },   /*  3 N2→N59 */
    {  35, -90 },   /*  4 N59→N60 */
    {  40, +90 },   /*  5 N60→N52 */
    { 134, -90 },   /*  6 N52→N55 */
    {  38, -90 },   /*  7 N55→N41 */
    { 134, +90 },   /*  8 N41→N38 */
    {  74, +90 },   /*  9 N38→N13 */
    { 134, -90 },   /* 10 N13→N16 */
    {  38, +90 },   /* 11 N16→N6 */
    {  24, +90 },   /* 12 N6→N7 */
    {  38, -90 },   /* 13 N7→N17 */
    {  94, +90 },   /* 14 N17→N20 */
    {  55, +90 },   /* 15 N20→N35 */
    {  27, -90 },   /* 16 N35→N33 */
    {  50,  +0 },   /* 17 N33→N49 */
    { 172,  +0 },   /* 18 N49→RING */
    {  47, +90 },   /* 19 N49→N65 */
    {  91, +90 },   /* 20 N65→N63 */
    { 121, -90 },   /* 21 N63→N25 */
    {  72, +90 },   /* 22 N25→N24 */
    {  31, -90 },   /* 23 N24→N15 */
    {  62, +90 },   /* 24 N15→N13 */
    {  38, +90 },   /* 25 N13→N3 */
    { 252, +90 },   /* 26 N3→N10 */
    { 190, +90 },   /* 27 N10→N67 */
    { 339, +90 },   /* 28 N67→N58 */
    {  97,  +0 },   /* 29 N58→N28 */
};

#define LEG_N ((int)(sizeof(LEGS) / sizeof(LEGS[0])))

/* ==================== 十二、主程序 ==================== */

int main(void)
{
    sys_tick_init();
    uart_init();
    led_init();
    motor_gpio_dir();
    pwm_init();
    enc_init();
    track_init();
    motor_stop();
    g_lm_prev = line_mask(track_raw());

    uart_str("\r\n\r\n");
    uart_str("=== hybrid_route v0.5.1  PID follow + odometry turn ===\r\n");
    uart_num("legs=", LEG_N);
    uart_str("\r\n");

    int32_t total_cm = 0;
    for (int i = 0; i < LEG_N; i++) total_cm += LEGS[i].cm;
    uart_num("route total=", total_cm);
    uart_str("cm\r\n");

    uart_num("KP=", (int32_t)(KP * 10.0f));
    uart_num("  KD=", (int32_t)(KD * 10.0f));
    uart_num("  TURN_SCALE=", (int32_t)(TURN_SCALE * 100.0f));
    uart_str("/100  SPEED=");
    uart_int(SPEED_LEG);
    uart_str("\r\n");

    uart_str("track sensor now: raw=");
    uart_int((int32_t)track_raw());
    uart_str(" lm=");
    {
        uint8_t lm0 = line_mask(track_raw());
        for (int i = 0; i < 8; i++) uart_char((lm0 & (1u << i)) ? '1' : '0');
    }
    uart_str("\r\n");

    uart_str("'g' = start now, 'c' = sensor watch, 3s countdown otherwise\r\n");

    /* 3 秒窗口 */
    {
        uint8_t go = 0;
        while (!go && !g_abort) {
            uint32_t t0 = millis();
            while ((uint32_t)(millis() - t0) < 3000u && !go) {
                int c = uart_try_get();
                if (c == 'g' || c == 'G') go = 1;
                else if (c == 'c' || c == 'C') { sensor_watch(); t0 = millis(); }
                else if (c == 'x' || c == 'X') { g_abort = 1; break; }
                led_set(((millis() / 150u) & 1u) ? 0 : 1);
                delay_ms(5);
            }
            led_set(0);
            if (!go && !g_abort) {
                uart_str("countdown again ('g' to start, 'c' to watch again)\r\n");
            }
        }
    }

    if (g_abort) {
        motor_stop();
        uart_str("aborted before start\r\n");
        while (1) { motor_stop(); delay_ms(500); }
    }

    uart_str("--- GO ---\r\n");
    delay_ms(LEG_SETTLE_MS);

    float odom_total = 0.0f;
    uint32_t t_start = millis();
    int8_t rc = 1;

    for (int i = 0; i < LEG_N && rc > 0; i++) {
        uart_str("[L");
        uart_int(i + 1);
        uart_str("] cm=");
        uart_int(LEGS[i].cm);
        uart_str(" turn=");
        uart_int(LEGS[i].turn);
        uart_str("\r\n");

        rc = drive_pid((float)LEGS[i].cm);
        if (rc <= 0) break;

        odom_total += (float)LEGS[i].cm;
        delay_ms(LEG_SETTLE_MS);

        if (LEGS[i].turn != 0) {
            rc = turn_deg((float)LEGS[i].turn);
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
