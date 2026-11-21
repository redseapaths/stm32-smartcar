/* ============================================================
 *  simple_follow.c —— 最简版：只有「巡线 + 里程表」
 *
 *  刻意不带这些东西（都在 route_follow.c 里，那份已备份）：
 *    - Flash 黑匣子日志、x/m/c/t/e 命令
 *    - 29 段路线表、路口/环岛判定、原地转向、自动标定
 *  目的：先用最小代码确认「编码器读数对、传感器方向对、车能顺着线走」，
 *        这三件事确认之后再回到 route_follow.c 加功能。
 *
 *  引脚（与现有接线一致，别改）：
 *    PA8   TIM1_CH1  PWM 右轮        PA11  TIM1_CH4  PWM 左轮
 *    PB12/PB13 右轮方向 AIN1/AIN2    PB14/PB15 左轮方向 BIN1/BIN2
 *    PA0..PA7  8 路循迹（1 = 压白线，0 = 蓝底）
 *    PB6/PB7   左编码器 A/B（TIM4 硬件四倍频）
 *    PB8/PB9   右编码器 A/B（EXTI9_5 软件四倍频）
 *    PA9/PA10  USART1 115200，8N1
 *    PC13      板载 LED（低电平点亮）
 *
 *  串口每 200ms 打一行：
 *    t=12345  cm=573  v=31  err=-15  raw=00011000  R=380 L=320
 *      t   上电后的毫秒数
 *      cm  累计里程（厘米，可以按 'z' 清零）
 *      v   这 200ms 的速度（厘米/秒）
 *      err 循迹偏差（负=线偏左，正=线偏右，±35 满量程）
 *      raw 传感器原始字节（波形对照用）
 *      R/L 实际给两个轮子的 PWM
 * ============================================================ */

#include "stm32f10x.h"

static void delay_ms(uint32_t ms);
static void motor_set(int32_t right, int32_t left);
static void motor_stop(void);

/* ==================== 一、全部可调参数都在这里 ==================== */

/* 每厘米编码器计数（四倍频后）。2026-11-08 推车 100cm 标定得 2700 */
#define ENC_PER_CM      2700.0f

/* 电机方向：车往前跑时发现某个轮子倒转，就把对应的改成 0 */
#define RIGHT_INVERT    1
#define LEFT_INVERT     1

/* 编码器方向：手推车往前走，两个轮子的里程都应该是正数。
   2026-11-10 实测右轮读到 -34mm、左轮 +28mm，故右轮翻正。 */
#define ENC_RIGHT_SIGN  (+1)
#define ENC_LEFT_SIGN   (-1)

/* 传感器排线顺序：2026-11-10 实测，挡住物理最左的光电管，读回来的是最右一位。
   1 = 在软件里翻正（让 X1 真的对应最左边那一路） */
#define SENSOR_REVERSE  1

/* ---- 校准（v0.3.0）：上电 3 秒内发 'k' 走一遍 ----
 * 校准后每路两类信息：
 *   CAL_MASK_DEFAULT  哪几路"蓝底/白线读数不同"= 有效，1=有效
 *   CAL_INV_DEFAULT   哪几路极性和别人相反，1=要翻转
 * 手工填也行（校准时串口会把这两个值打出来），不填就是全有效、不翻转。 */
#define CAL_MASK_DEFAULT 0xFFu
#define CAL_INV_DEFAULT  0x00u

static uint8_t g_cal_mask = CAL_MASK_DEFAULT;
static uint8_t g_cal_inv  = CAL_INV_DEFAULT;

/* ---- 速度（0~999 对应 0~100% 占空比） ---- */
#define BASE_SPEED      300       /* 直道速度，先慢一点 */
#define TURN_MAX        400       /* 差速上限，防止一侧反转 */

/* ---- 控制：PD ---- */
#define KP              6.0f
#define KD              2.0f

/* ---- 丢线 ---- */
#define LOST_MS         250       /* 全蓝多久算丢线 */
#define LOST_STOP_MS    300       /* 丢线后先停这么久 */
#define LOST_BACK_MS    300       /* 再倒退这么久找线 */
#define LOST_TURN_MS    250       /* 再按"上次偏差方向"原地转这么久 */
#define LOST_GIVEUP     6         /* 连续丢线这么多次就彻底停下 */

/* ============================================================
 *  二、1ms 时基
 * ============================================================ */
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

/* ============================================================
 *  三、串口 USART1（PA9/PA10，115200）
 * ============================================================ */
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

/* 打 8 位二进制 */
static void uart_bits(uint8_t v)
{
    for (int8_t k = 7; k >= 0; k--) uart_char((v & (1u << k)) ? '1' : '0');
}

static int uart_try_get(void)
{
    if (USART1->SR & USART_SR_RXNE) return (int)(USART1->DR & 0xFFu);
    if (USART1->SR & USART_SR_ORE) (void)USART1->DR;
    return -1;
}

/* ============================================================
 *  四、板载 LED（PC13，低电平点亮）
 * ============================================================ */
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

/* ============================================================
 *  五、电机：TB6612，PWM 用 TIM1_CH1(PA8)=右 / CH4(PA11)=左
 * ============================================================ */
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

    TIM1->PSC  = 71;        /* 72MHz / 72 = 1MHz */
    TIM1->ARR  = 999;       /* 1kHz PWM */
    TIM1->CCR1 = 0;
    TIM1->CCR4 = 0;
    TIM1->CCMR1 = (TIM1->CCMR1 & ~(7u << 4))  | (6u << 4);   /* CH1 PWM1 */
    TIM1->CCMR2 = (TIM1->CCMR2 & ~(7u << 12)) | (6u << 12);  /* CH4 PWM1 */
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

/* ============================================================
 *  六、编码器：左 TIM4 硬件四倍频，右 EXTI 软件四倍频
 * ============================================================ */
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

    GPIOB->CRL = (GPIOB->CRL & ~(0xFFu << 24)) | (0x44u << 24);   /* PB6/PB7 浮空 */
    GPIOB->CRH = (GPIOB->CRH & ~(0xFFu << 0))  | (0x88u << 0);    /* PB8/PB9 上拉 */
    GPIOB->BSRR = (1u << 8) | (1u << 9);

    AFIO->EXTICR[2] = (AFIO->EXTICR[2] & ~0x00FFu) | 0x0011u;     /* EXTI8/9 → PB */
    EXTI->IMR  |= (1u << 8) | (1u << 9);
    EXTI->RTSR |= (1u << 8) | (1u << 9);
    EXTI->FTSR |= (1u << 8) | (1u << 9);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    TIM4->PSC   = 0;
    TIM4->ARR   = 0xFFFF;
    TIM4->CCMR1 = (TIM4->CCMR1 & ~0xFFFFu) | 0x0101u;
    TIM4->SMCR  = (TIM4->SMCR & ~7u) | 3u;                        /* 编码器模式 */
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

/* 两轮平均里程（cm）：取走并清零。正数 = 车往前走 */
static float odom_take_cm(void)
{
    NVIC_DisableIRQ(EXTI9_5_IRQn);
    int16_t cl = (int16_t)TIM4->CNT;
    int16_t cr = (int16_t)enc_r;
    TIM4->CNT = 0;
    enc_r = 0;
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    int32_t pl = (int32_t)cl * ENC_LEFT_SIGN;
    int32_t pr = (int32_t)cr * ENC_RIGHT_SIGN;
    return ((float)pl + (float)pr) * 0.5f / ENC_PER_CM;
}

/* ============================================================
 *  七、循迹：PA0..PA7
 * ============================================================ */
static void track_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRL = 0x88888888u;      /* PA0..PA7 上拉输入 */
    GPIOA->BSRR = 0x00FFu;
}

static uint8_t track_raw(void) { return (uint8_t)(GPIOA->IDR & 0xFFu); }

static uint8_t rev8(uint8_t v)
{
    v = (uint8_t)(((v & 0xF0u) >> 4) | ((v & 0x0Fu) << 4));
    v = (uint8_t)(((v & 0xCCu) >> 2) | ((v & 0x33u) << 2));
    v = (uint8_t)(((v & 0xAAu) >> 1) | ((v & 0x55u) << 1));
    return v;
}

/* 白线掩码：1 = 这一路压在白线上 */
static uint8_t line_mask(uint8_t raw)
{
#if SENSOR_REVERSE
    raw = rev8(raw);
#endif
    raw = (uint8_t)(raw ^ g_cal_inv);      /* 极性不一致的通道翻正 */
    return (uint8_t)(raw & g_cal_mask);    /* 屏蔽在这个高度瞎掉的通道 */
}

/* 逐路 3 帧去抖：同一路要连续 2 帧变才认，压一下抖动的毛刺 */
static uint8_t g_lm_prev = 0;
static uint8_t g_lm_cnt[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

static uint8_t track_stable(void)
{
    uint8_t m = line_mask(track_raw());
    uint8_t out = 0;
    for (uint8_t k = 0; k < 8; k++) {
        uint8_t bit  = (uint8_t)((m >> k) & 1u);
        uint8_t prev = (uint8_t)((g_lm_prev >> k) & 1u);
        if (bit == prev) {
            g_lm_cnt[k] = 0;
            if (prev) out |= (uint8_t)(1u << k);
        } else {
            if (g_lm_cnt[k] < 2) g_lm_cnt[k]++;
            if (g_lm_cnt[k] >= 2) {
                if (bit) out |= (uint8_t)(1u << k);
            } else {
                if (prev) out |= (uint8_t)(1u << k);
            }
        }
    }
    g_lm_prev = out;
    return out;
}

static uint8_t pop8(uint8_t v)
{
    uint8_t n = 0;
    while (v) { n += (uint8_t)(v & 1u); v >>= 1; }
    return n;
}

/* 偏差：线偏左为负、偏右为正，范围 ±35 */
static int16_t track_err(uint8_t lm)
{
    static const int8_t W[8] = { -35, -25, -15, -5, 5, 15, 25, 35 };
    int16_t sum = 0;
    uint8_t n = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (lm & (1u << i)) { sum += W[i]; n++; }
    }
    if (n == 0) return 999;
    return (int16_t)(sum / (int16_t)n);
}

/* ============================================================
 *  八、主循环：PD 巡线 + 里程表
 * ============================================================ */
/* ---------------- 校准 ---------------- */

static void cal_hex2(uint8_t v)
{
    const char *H = "0123456789ABCDEF";
    uart_char(H[(v >> 4) & 0xFu]);
    uart_char(H[v & 0xFu]);
}

static void cal_bits(const char *tag, uint8_t v)
{
    uart_str(tag);
    for (int8_t k = 0; k < 8; k++) uart_char((v & (1u << k)) ? '1' : '0');
    uart_str("   (X1..X8)\r\n");
}

/* 边等按键边刷实时读数：调电位器时盯着这一行，看哪一路在动 */
static void cal_wait(void)
{
    uint32_t last = 0;
    while (1) {
        uint32_t now = millis();
        if ((uint32_t)(now - last) >= 200u) {
            uint8_t m = line_mask(track_raw());
            last = now;
            uart_str("   live: ");
            for (int8_t k = 0; k < 8; k++) {
                uart_char((m & (1u << k)) ? '#' : '.');
                uart_char(' ');
            }
            uart_num(" n=", (int32_t)pop8(m));
            uart_str("\r\n");
        }
        if (uart_try_get() >= 0) break;
        delay_ms(2);
    }
}

/* 采 0.3 秒，每路按多数表决取一个稳定的 8 位读数 */
static uint8_t cal_sample(void)
{
    uint8_t cnt[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t out = 0;
    for (uint16_t i = 0; i < 60; i++) {
        uint8_t m = line_mask(track_raw());
        for (uint8_t k = 0; k < 8; k++) if (m & (1u << k)) cnt[k]++;
        delay_ms(5);
    }
    for (uint8_t k = 0; k < 8; k++) if (cnt[k] >= 30u) out |= (uint8_t)(1u << k);
    return out;
}

static void cal_do(void)
{
    uint8_t blue, white, mask = 0, inv = 0;

    g_cal_mask = 0xFFu;                 /* 校准时先按"全有效、不翻转"读原始值 */
    g_cal_inv  = 0x00u;

    uart_str("\r\n=== CALIBRATION (6 steps of nothing, just 2 key presses) ===\r\n");
    uart_str("1) put the car on the BLUE floor, away from any white line\r\n");
    uart_str("   then press any key\r\n");
    cal_wait();
    blue = cal_sample();
    cal_bits("   BLUE  = ", blue);

    uart_str("2) now put the car so the white line sits under the middle sensors\r\n");
    uart_str("   then press any key\r\n");
    cal_wait();
    white = cal_sample();
    cal_bits("   WHITE = ", white);

    for (uint8_t k = 0; k < 8; k++) {
        uint8_t b = (uint8_t)((blue  >> k) & 1u);
        uint8_t w = (uint8_t)((white >> k) & 1u);
        if (b != w) {
            mask |= (uint8_t)(1u << k);
            if (w == 0) inv |= (uint8_t)(1u << k);   /* 白线读 0 = 极性反 */
        }
    }

    uart_str("\r\n--- result ---\r\n");
    cal_bits("MASK  = ", mask);
    cal_bits("INV   = ", inv);
    uart_num("valid = ", (int32_t)pop8(mask));
    uart_num(" / 8    inverted = ", (int32_t)pop8(inv));
    uart_str("\r\n");

    if (mask == 0u) {
        uart_str("!! BLUE and WHITE read exactly the same on all 8 channels.\r\n");
        uart_str("!! at this height there is NO contrast at all -- no software can fix that.\r\n");
        uart_str("!! fix hardware first: 3~8mm height, lock it, then turn each pot\r\n");
        uart_str("!! until its channel flips only on white. re-run 'k' to check.\r\n");
    } else if (mask != 0xFFu) {
        uart_str("!! some channels are blind at this height (see MASK) -- they are ignored.\r\n");
    } else {
        uart_str("all 8 channels see a difference. good.\r\n");
    }

    g_cal_mask = mask;
    g_cal_inv  = inv;

    uart_str("to hard-code it:  #define CAL_MASK_DEFAULT 0x");
    cal_hex2(mask);
    uart_str("   #define CAL_INV_DEFAULT 0x");
    cal_hex2(inv);
    uart_str("\r\npress any key to start running\r\n\r\n");
    cal_wait();
}

int main(void)
{
    sys_tick_init();
    uart_init();
    motor_gpio_dir();
    pwm_init();
    enc_init();
    track_init();
    led_init();
    motor_stop();

    uart_str("\r\n=== simple_follow v0.3.0 ===\r\n");
    uart_str("send 'k' within 3s to CALIBRATE the 8 sensors, otherwise it just runs\r\n");
    {
        uint32_t t0 = millis();
        int hit = -1;
        while ((uint32_t)(millis() - t0) < 3000u) {
            int c = uart_try_get();
            if (c == 'k' || c == 'K') { hit = c; break; }
            delay_ms(10);
        }
        if (hit == 'k' || hit == 'K') cal_do();
    }
    uart_str("t=ms  cm=odometer  v=cm/s  err  raw  R L\r\n");
    uart_str("press 'z' to zero odometer\r\n\r\n");

    float total_cm = 0.0f;
    float win_cm   = 0.0f;
    int16_t last_e = 0;
    uint32_t lost_ms = 0;
    uint8_t  lost_n  = 0;
    uint32_t prev = millis();
    uint32_t last_print = prev;
    float last_v = 0.0f;

    while (1) {
        /* ---------- 串口命令（只有清零） ---------- */
        int c = uart_try_get();
        if (c == 'z' || c == 'Z') { total_cm = 0.0f; uart_str("odom zeroed\r\n"); }

        /* ---------- 读传感器 ---------- */
        uint32_t now = millis();
        uint32_t dt = now - prev;
        prev = now;
        if (dt == 0) dt = 1;
        if (dt > 50) dt = 50;

        uint8_t raw = track_raw();
        uint8_t lm  = track_stable();
        uint8_t n   = pop8(lm);
        int16_t e   = track_err(lm);

        /* ---------- 控制 ---------- */
        int32_t sp = BASE_SPEED;
        int32_t pid = 0;

        if (lm == 0x00u) {
            /* 全蓝：一路都没压线 = 丢线 */
            lost_ms += dt;
            if (lost_ms > LOST_MS) {
                lost_n++;
                motor_stop();
                led_set(0);
                uart_num("[LOST #", lost_n);
                uart_num("] cm=", (int32_t)total_cm);
                uart_num(" e=", last_e);
                uart_str("\r\n");

                if (lost_n >= LOST_GIVEUP) {
                    uart_str("!! give up, halted\r\n");
                    while (1) { led_set(0); delay_ms(500); }
                }

                delay_ms(LOST_STOP_MS);
                motor_set(-200, -200);        /* 后退找线 */
                delay_ms(LOST_BACK_MS);
                if (last_e < 0) motor_set(-180, 180);   /* 线在左边 → 原地左转 */
                else            motor_set(180, -180);
                delay_ms(LOST_TURN_MS);
                motor_stop();
                (void)odom_take_cm();         /* 恢复动作的里程不计入总里程 */
                lost_ms = 0;
            } else {
                pid = (int32_t)last_e * 3;
                motor_set(sp - pid, sp + pid);
                led_set(1);
            }
        } else if (n >= 7) {
            /* 7~8 路压白 = 宽白块/横向线：直着过去 */
            motor_set(sp, sp);
            lost_ms = 0;
            lost_n = 0;
            led_set(1);
        } else {
            int32_t pp = (int32_t)(KP * (float)e + KD * (float)(e - last_e));
            if (pp >  TURN_MAX) pp =  TURN_MAX;
            if (pp < -TURN_MAX) pp = -TURN_MAX;
            pid = pp;
            motor_set(sp - pid, sp + pid);
            last_e = e;
            lost_ms = 0;
            lost_n = 0;
            led_set(1);
        }

        /* ---------- 里程表 ---------- */
        float d = odom_take_cm();
        total_cm += d;
        win_cm   += d;

        /* ---------- 每 200ms 打一行 ---------- */
        if ((uint32_t)(now - last_print) >= 200u) {
            uint32_t span = now - last_print;
            last_print = now;
            last_v = win_cm * 1000.0f / (float)span;
            win_cm = 0.0f;

            uart_num("t=", (int32_t)now);
            uart_num("  cm=", (int32_t)total_cm);
            uart_num("  v=", (int32_t)last_v);
            uart_num("  err=", (e == 999 ? 999 : (int32_t)e));
            uart_str("  raw=");
            uart_bits(raw);
            uart_num("  R=", sp - pid);
            uart_num(" L=", sp + pid);
            uart_str("\r\n");
        }

        delay_ms(2);
    }
}
