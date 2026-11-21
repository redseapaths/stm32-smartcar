/* ============================================================
 *  blind_route.c —— 纯里程盲跑版（无 PID、完全不看传感器）  v0.4.0
 *
 *  目的只有一个：验证「路线表 + 里程 + 转弯」这套逻辑到底对不对。
 *  所以这一版**故意**不读 8 路循迹、不做任何 PID 修正：
 *    - 前进：两个轮子给同一个固定 PWM，跑到里程到了就停
 *    - 转弯：两个轮子反向给固定 PWM，按"轮子走过的弧长"算角度，到了就停
 *  如果这样能大致走完 29 段，说明路线表、编码器比例尺、转弯算法都是对的，
 *  以后加回循迹只是"锦上添花"；如果这样都走不准，那问题在里程/机械，不在算法。
 *
 *  引脚（与现有接线一致，别改）：
 *    PA8   TIM1_CH1  PWM 右轮        PA11  TIM1_CH4  PWM 左轮
 *    PB12/PB13 右轮方向 AIN1/AIN2    PB14/PB15 左轮方向 BIN1/BIN2
 *    PB6/PB7   左编码器 A/B（TIM4 硬件四倍频）
 *    PB8/PB9   右编码器 A/B（EXTI9_5 软件四倍频）
 *    PA9/PA10  USART1 115200，8N1
 *    PC13      板载 LED（低电平点亮）
 *
 *  串口命令：
 *    g  立刻开始（不等 3 秒倒计时）
 *    x  紧急停车，程序停在原地
 *    p  暂停 / 继续（按住不放的那种暂停，不丢里程）
 *
 *  转弯符号约定：turn = +90 表示**右转**（顺时针），-90 表示左转（逆时针）
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

/* 轮距（左右轮中心距离，厘米）。转 90° 需要每个轮子走 π*D/4 */
#define WHEEL_BASE_CM   16.75f

/* 转弯修正系数：车实际转得不够就加大（1.05、1.10…），转过头就减小（0.95…）。
 * 先按 1.00 跑一圈，量一下实际转了多少度，再用 期望角度/实际角度 去算这个值。 */
#define TURN_SCALE      0.68f

/* 速度（0~999 = 0~100% 占空比） */
#define SPEED_LEG       420       /* 直行 */
#define SPEED_TURN      380       /* 原地转 */
#define SPEED_MIN       240       /* 快到位时的最低速，再低电机带不动 */

/* 距离目标还有这么远就开始减速（厘米） */
#define DECEL_CM        5.0f

/* 每段走完停一下，让车稳一稳（毫秒） */
#define LEG_SETTLE_MS   300

/* 超时保护 */
#define LEG_TIMEOUT_MS  30000u    /* 单段最长 30 秒 */
#define TURN_TIMEOUT_MS 15000u    /* 单次转弯最长 15 秒 */

/* 堵转保护：给了 PWM 但编码器 800ms 没变化，就认为卡住了 */
#define STALL_MS        800u

/* ---- 调试模式（v0.4.1）：只做一次右转 90°，其他全部不执行 ----
 *   1 = 打开：上电 1 秒后右转 90°，转完就停住，路线/倒计时/循迹全部不跑
 *   0 = 关闭：回到正常的 29 段盲跑
 * 用途：单独验 TURN_SCALE 准不准。量出实际转了多少度 θ，新值 = 当前值 × 90/θ */
#define DEBUG_TURN_ONCE  0
#define DEBUG_TURN_DEG   90        /* 转多少度，正数=右转 */
#define DEBUG_DELAY_MS   1000u     /* 上电后等多久开始转 */

/* 每段最多 999cm（打印用），路线表里最长的 339cm 够用 */

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

    GPIOB->CRL = (GPIOB->CRL & ~(0xFFu << 24)) | (0x44u << 24);
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

/* ==================== 七、公共状态 ==================== */

static uint8_t g_abort  = 0;    /* 'x' */
static uint8_t g_paused = 0;    /* 'p' */

/* 处理串口命令；返回 1 = 这一轮需要立刻退出 */
static uint8_t poll_cmd(void)
{
    int c;
    while ((c = uart_try_get()) >= 0) {
        if (c == 'x' || c == 'X') { g_abort = 1; }
        else if (c == 'p' || c == 'P') { g_paused = !g_paused; }
    }
    return g_abort;
}

/* 暂停时原地等，不影响里程 */
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
        led_set(((millis() / 200u) & 1u) ? 0 : 1);
        delay_ms(5);
    }
    led_set(0);
    uart_str("  resumed\r\n");
}

/* 减速曲线：剩得越少给得越小，但不是 PID，只是查表 */
static int32_t speed_for(float remain, int32_t full)
{
    int32_t sp = full;
    if (remain < DECEL_CM) {
        sp = SPEED_MIN + (int32_t)((float)(full - SPEED_MIN) * remain / DECEL_CM);
        if (sp < SPEED_MIN) sp = SPEED_MIN;
    }
    return sp;
}

/* ==================== 八、走一段直线（纯里程，无修正） ==================== */

/* 返回 1 = 走完，0 = 被 x 打断，-1 = 堵转/超时 */
static int8_t drive_cm(float target_cm)
{
    float done = 0.0f;
    uint32_t t0 = millis(), t_stall = millis(), t_report = millis();
    float stall_ref = 0.0f;
    int32_t last_sp = SPEED_LEG;

    while (done < target_cm) {
        if (poll_cmd()) { motor_stop(); return 0; }
        wait_if_paused();

        float dl, dr;
        odom_take2(&dl, &dr);
        done += (dl + dr) * 0.5f;

        int32_t sp = speed_for(target_cm - done, SPEED_LEG);
        last_sp = sp;
        motor_set(sp, sp);

        /* 堵转检测 */
        if (done - stall_ref > 0.2f || stall_ref - done > 0.2f) {
            stall_ref = done;
            t_stall = millis();
        } else if ((uint32_t)(millis() - t_stall) > STALL_MS) {
            motor_stop();
            uart_str("  ** STALL: 编码器不动了（检查接线/电池/是否顶住）**\r\n");
            return -1;
        }

        /* 每 500ms 打一行进度（长段不至于刷屏） */
        if ((uint32_t)(millis() - t_report) >= 500u) {
            t_report = millis();
            uart_str("    ");
            uart_num("done=", (int32_t)(done * 10.0f)); uart_str("mm/");
            uart_num("", (int32_t)(target_cm * 10.0f)); uart_str("mm ");
            uart_num("sp=", last_sp);
            uart_str("\r\n");
        }

        if ((uint32_t)(millis() - t0) > LEG_TIMEOUT_MS) {
            motor_stop();
            uart_str("  ** TIMEOUT: 这一段走太久了 **\r\n");
            return -1;
        }

        led_set(1);
        delay_ms(5);
        led_set(0);
    }

    motor_stop();
    /* 刹车后溜一点，等它停稳 */
    delay_ms(150);
    return 1;
}

/* ==================== 九、原地转（纯里程，无修正） ==================== */

/* deg > 0 右转（顺时针），deg < 0 左转。返回同 drive_cm */
static int8_t turn_deg(float deg)
{
    if (deg == 0.0f) return 1;

    /* 每个轮子要走的弧长：圆的周长 × (角度/360) */
    float need = 3.14159265f * WHEEL_BASE_CM * (deg > 0 ? deg : -deg) / 360.0f;
    need *= TURN_SCALE;

    float got = 0.0f;
    uint32_t t0 = millis(), t_stall = millis();
    float stall_ref = 0.0f;
    int32_t dir = (deg > 0) ? 1 : -1;      /* +1 = 右转 */

    /* 起转前把编码器里残留的计数丢掉：
       上一段停车后车还会往前滑几厘米，那些计数不能让转弯"一开始就转好了" */
    {
        float jl, jr;
        odom_take2(&jl, &jr);
    }

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
        /* 转角的正确度量是「两轮的相对转动」，不是「两轮各走了多远」：
           车往前滑行时 dl ≈ dr（差值≈0，不计入转角）
           真正原地转时 dl ≈ -dr（差值最大，正好等于每轮走过的弧长）
           2026-11-12 实测：原来的 |dl|+|dr| 写法会把停车滑行当成已转角度，
           导致正式跑路线时只转了一点点（调试模式车是静止的，所以看不出来） */
        float d = (dir > 0) ? (dl - dr) : (dr - dl);
        if (d > 0.0f) got += d * 0.5f;

        int32_t sp = speed_for(need - got, SPEED_TURN);
        if (dir > 0) motor_set(-sp,  sp);   /* 右轮后退、左轮前进 → 右转 */
        else         motor_set( sp, -sp);   /* 左转 */

        if (got - stall_ref > 0.2f) { stall_ref = got; t_stall = millis(); }
        else if ((uint32_t)(millis() - t_stall) > STALL_MS) {
            motor_stop();
            uart_str("  ** STALL: 转不动（检查接线/电池）**\r\n");
            return -1;
        }

        if ((uint32_t)(millis() - t0) > TURN_TIMEOUT_MS) {
            motor_stop();
            uart_str("  ** TIMEOUT: 转弯超时 **\r\n");
            return -1;
        }

        led_set(1);
        delay_ms(5);
        led_set(0);
    }

    motor_stop();
    delay_ms(200);      /* 转完等一下，别让惯性带着继续转 */
    return 1;
}

/* ==================== 十、路线表（与 route_follow.c 完全一致） ==================== */

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
    { 172,  +0 },   /* 18 N49→RING（环岛段，先按直线处理） */
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

/* ==================== 十一、主程序 ==================== */

int main(void)
{
    sys_tick_init();
    uart_init();
    led_init();
    motor_gpio_dir();
    pwm_init();
    enc_init();
    motor_stop();

    uart_str("\r\n\r\n");
    uart_str("=== blind_route v0.4.2  pure odometry, NO PID, NO sensors ===\r\n");
    uart_num("legs=", LEG_N);
    uart_str("\r\n");

    int32_t total_cm = 0;
    for (int i = 0; i < LEG_N; i++) total_cm += LEGS[i].cm;
    uart_num("route total=", total_cm);
    uart_str("cm\r\n");

    /* ===== 调试模式：其他全部停下来，只转一次 ===== */
    if (DEBUG_TURN_ONCE) {
        uart_str("\r\n=== DEBUG MODE: one turn only, everything else disabled ===\r\n");
        uart_num("turn deg = ", DEBUG_TURN_DEG);
        uart_str("   (positive = right)");
        uart_str("\r\nTURN_SCALE = ");
        uart_int((int32_t)(TURN_SCALE * 100.0f));
        uart_str("/100\r\n");
        uart_str("waiting ");
        uart_int((int32_t)DEBUG_DELAY_MS);
        uart_str("ms after power on ...\r\n");
        delay_ms(DEBUG_DELAY_MS);

        int8_t rr = turn_deg((float)DEBUG_TURN_DEG);
        motor_stop();
        if (rr > 0) {
            uart_str("=== turn FINISHED --- measure the real angle now ===\r\n");
            uart_str("new TURN_SCALE = 90 / measured_deg   (e.g. 80deg -> 1.13)\r\n");
        } else {
            uart_str("=== turn FAILED (stall or timeout) ===\r\n");
        }
        uart_str("=== halted, power cycle to turn again ===\r\n");
        while (1) {
            motor_stop();
            led_set(1); delay_ms(300);
            led_set(0); delay_ms(700);
        }
    }

    uart_str("put the car on the track, press 'g' to start (auto in 3s), 'x' = stop\r\n");

    /* 3 秒倒计时，灯闪；按 g 立刻开始 */
    {
        uint32_t t0 = millis();
        uint8_t go = 0;
        while ((uint32_t)(millis() - t0) < 3000u && !go) {
            int c = uart_try_get();
            if (c == 'g' || c == 'G') go = 1;
            if (c == 'x' || c == 'X') { g_abort = 1; break; }
            led_set(((millis() / 150u) & 1u) ? 0 : 1);
            delay_ms(5);
        }
        led_set(0);
    }

    if (g_abort) { motor_stop(); uart_str("aborted before start\r\n"); while (1) { } }

    uart_str("--- GO ---\r\n");

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

        rc = drive_cm((float)LEGS[i].cm);
        if (rc <= 0) break;

        odom_total += (float)LEGS[i].cm;
        delay_ms(LEG_SETTLE_MS);

        if (LEGS[i].turn != 0) {
            rc = turn_deg((float)LEGS[i].turn);
            if (rc <= 0) break;
            delay_ms(LEG_SETTLE_MS);
        }

        uart_str("  ok, total=");
        uart_int((int32_t)(odom_total));
        uart_str("cm, t=");
        uart_int((int32_t)((millis() - t_start) / 1000u));
        uart_str("s\r\n");
    }

    motor_stop();

    if (rc > 0) {
        uart_str("\r\n=== ROUTE FINISHED ===\r\n");
    } else if (rc == 0) {
        uart_str("\r\n=== ABORTED by 'x' ===\r\n");
    } else {
        uart_str("\r\n=== STOPPED (stall/timeout) ===\r\n");
    }
    uart_str("total=");
    uart_int((int32_t)odom_total);
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
