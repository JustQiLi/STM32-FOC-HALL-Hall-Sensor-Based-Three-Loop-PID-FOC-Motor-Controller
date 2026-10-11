/**
  * @file    bsp_encoder.c
  * @brief   霍尔电角度估算 + 零位标定 + 力矩方向自检
  * ① 锚点：每跨过一个霍尔边沿，转子必定正好落在 60° 扇区边界，用它钉死不漂。
  * ② 插值：边沿之间按上一扇区测得的转速线性外推。
  * ③ 平滑：边沿处的残余误差折算成下一扇区的速度偏置，一个扇区吃完（死拍）。
  * ④ 兜底：角度夹回当前霍尔扇区，最坏误差 ≤60°，转矩方向永不反转。
  * 上电做一次零位标定（Id 拖动细分），紧接着做方向自检（判掉模 π 二义性）。
  * @note   TIM5 霍尔接口 PH10/11/12，200kHz（5µs/tick），20kHz FOC 中断里调用。
  */

#include "bsp_encoder.h"
#include <math.h>

/*=========================== 内部状态 ==============================*/
static HALLParm_TypeDef     *s_pHall   = NULL;

/* 边沿信息：TIM5 中断写、FOC 中断读。s_edge_cnt 最后写，作为"本条信息已完整"的
 * 提交标志；读取侧统一走 HALL_ReadEdge() 的临界区快照，不依赖 NVIC 优先级配置。 */
static volatile uint32_t s_edge_cnt    = 0;   /* 已确认边沿计数（提交标志） */
static uint32_t         s_last_ccr    = 0;   /* 上次有效边沿的 TIM5 捕获值(5us/tick)，毛刺去抖用 */
static uint32_t s_debounce_ticks = HALL_EDGE_DEBOUNCE_TICKS;   /* 去抖窗(tick)，位置模式会动态加大 */
static volatile uint8_t  s_edge_phase  = 0;   /* 本次边沿后的霍尔扇区 1~6 */
static volatile int8_t   s_edge_dir    = 0;   /* 霍尔坐标系方向 +1/-1 */
static volatile uint32_t s_edge_period = 0;
static volatile uint32_t s_edge_ms     = 0;   /* 本次有效边沿的 1ms 时间戳 */
static volatile uint32_t s_ms_tick     = 0;   /* SysTick 1ms 自由计数 */

/* 最近 6 个同方向有效 Hall 周期：平均周期用于扇区角度插值，降低 Hall 安装误差和冲转噪声 */
#define HALL_PERIOD_RING_SIZE   6U
static volatile uint32_t s_period_ring[HALL_PERIOD_RING_SIZE];
static volatile uint8_t  s_ring_head;
static volatile uint8_t  s_ring_count;
static volatile int8_t   s_ring_dir;
static volatile int32_t  s_signed_edges;
static volatile uint8_t  s_breakaway_latch;   /* 1 = 刚发生"长时间无边沿后重新起转" */

/* 边沿信息快照 */
typedef struct
{
    uint32_t period;    /* 相邻两个霍尔边沿之间的 tick 数 */
    uint8_t  phase;     /* 边沿后的霍尔扇区 1~6 */
    int8_t   dir;       /* 霍尔坐标系方向 +1/-1 */
} HALL_Edge_TypeDef;

/* ---- 角度估算器状态（只在 FOC 中断里访问） ---- */
static float    s_hall_ang        = 0.0f;   /* 霍尔坐标系电角度估算值 [0,2π) */
static float    s_hall_spd        = 0.0f;   /* 最近单边沿电角速度（备用）rad/s */
static float    s_angle_spd       = 0.0f;   /* 角度插值用电角速度，来自六边沿平均 rad/s */
static float    s_phase_trim         = 0.0f;   /* 相位校正速度 rad/s（每个扇区更新一次） */
static float    s_zero_ofs           = 0.0f;   /* 电角度为 0 时的霍尔角（零位） */
static float    s_hall_dir        = 1.0f;   /* θe = σ·(h - h0) 的 σ */
static uint8_t  s_est_valid       = 0;      /* 估算器已锁定到霍尔 */
static uint8_t  s_phase_cur    = 0;      /* 估算器当前使用的霍尔扇区（与 s_hall_ang 同源） */
static int8_t   s_dir_cur      = 0;      /* 估算器当前使用的方向（与 s_hall_ang 同源） */
static uint32_t s_edge_seen    = 0;      /* 已处理的边沿计数 */
static uint16_t s_lost_limit   = 0;      /* 丢边沿判定门限 ms */

/* ---- 连续机械角合成（有符号边沿计数 + 扇区内分数） ---- */
static float   s_mech_zero       = 0.0f;  /* 标定结束时的机械角零点基准 */
static volatile float s_mech_raw_out = 0.0f; /* FOC中断内计算的原始机械角快照 */
static volatile float s_mech_angle_out = 0.0f; /* 对外机械角快照 */

/* ---- 低速测速观察器（只在 FOC 中断里更新） ----
 * 低速用连续机械角的固定窗口净位移测速，避免同一 Hall 边界往返时把局部振荡
 * 当成有效转速；中高速再平滑切回单边沿 T 法。状态封装在 Hall 模块内，
 * 上层状态机只读取最终速度。 */
typedef struct
{
    uint16_t window_cnt;
    uint8_t  initialized;
    uint8_t  edge_oscillating;
    float    mech_last;
    float    mech_rpm;
    float    fused_rpm;
    uint32_t edge_last;
    int32_t  signed_edge_last;
} HALLSpeedObserver_TypeDef;

static HALLSpeedObserver_TypeDef s_speed_obs;

/* ---- 兜底零位（两次对齐的扇区中点平均） ---- */
static uint8_t  s_calib_sec1     = 0;    /* 0°对齐时所在扇区 */
static uint8_t  s_calib_sec2     = 0;    /* 180°对齐时所在扇区 */
static float    s_calib_h0_fb    = 0.0f; /* 兜底零位 */
static int8_t   s_calib_sigma_fb = 1;    /* 兜底符号 */

/* ---- 拖动标定状态 ---- */
static float    s_calib_theta  = 0.0f;   /* 当前施加的电角度 */
static float    s_calib_start  = 0.0f;   /* 起始电角度 */
static uint32_t s_calib_seen   = 0;      /* 已处理的边沿计数 */
static uint8_t  s_calib_hits   = 0;      /* 已捕获的有效边沿数 */
static int8_t   s_calib_sigma  = 1;      /* 本次标定得到的符号 */
static float    s_calib_h0ref  = 0.0f;   /* 第一个边沿算出的零位 */
static float    s_calib_theta1 = 0.0f;   /* 第一个边沿时的电角度 */
static float    s_calib_h0     = 0.0f;   /* 最终零位 */
static uint8_t  s_calib_ok     = 0;      /* 标定成功标志 */
static uint8_t  s_calib_fail   = 0;      /* 标定失败标志（用兜底） */

/*=========================== BLDC方向状态表 ==============================*/
/* 正转 6->2->3->1->5->4->6；dir_tab[from][to] = +1 正转 / -1 反转 / 0 异常。
 * 必须严格互逆：dir_tab[a][b] == -dir_tab[b][a]，且 +1 连成一个 6 状态环。
 * 改动时务必保持互逆关系。 */
static const int8_t dir_tab[8][8] = {
/*to:  0   1   2   3   4   5   6   7  */
  {   0,  0,  0,  0,  0,  0,  0,  0 },  /* 0 */
  {   0,  0,  0, -1,  0,  1,  0,  0 },  /* 1: 1->5 正(+1), 1->3 反(-1) */
  {   0,  0,  0,  1,  0,  0, -1,  0 },  /* 2: 2->3 正(+1), 2->6 反(-1) */
  {   0,  1, -1,  0,  0,  0,  0,  0 },  /* 3: 3->1 正(+1), 3->2 反(-1) */
  {   0,  0,  0,  0,  0, -1,  1,  0 },  /* 4: 4->6 正(+1), 4->5 反(-1) */
  {   0, -1,  0,  0,  1,  0,  0,  0 },  /* 5: 5->4 正(+1), 5->1 反(-1) */
  {   0,  0,  1,  0, -1,  0,  0,  0 },  /* 6: 6->2 正(+1), 6->4 反(-1) */
  {   0,  0,  0,  0,  0,  0,  0,  0 },  /* 7 */
};

/* 正转方向的下一个扇区，用于兜底标定时判断霍尔角符号 */
static const uint8_t sect_next[8] = { 0, 5, 3, 1, 6, 4, 2, 0 };

/* 正转扇区环，与 dir_tab 的 +1 方向、phase_start_angle 同序：6->2->3->1->5->4->6 */
static const uint8_t sect_fwd[6] = { 6U, 2U, 3U, 1U, 5U, 4U };

/** @brief  两个霍尔状态之间的有向步数（正 = 正转方向）
  * @return  +1/-1 相邻过渡；+2/-2 合并过渡（跨两个扇区）；0 = 同状态、跨 3 步
  *          （无法判向）或含非法状态
  * @note    去抖窗内丢了中间那个边沿时，GPIO 会一次跳两个状态（dir_tab 得 d==0）。
  *          这时用本函数在扇区环上量出跨了几步，才能把边沿计数补回来。 */
static int8_t HALL_RingSpan(uint8_t from, uint8_t to)
{
    int8_t i_from = -1;
    int8_t i_to   = -1;
    int8_t k;
    int8_t span;

    for (k = 0; k < 6; k++)
    {
        if (sect_fwd[k] == from) i_from = k;
        if (sect_fwd[k] == to)   i_to   = k;
    }
    if ((i_from < 0) || (i_to < 0)) return 0;
    span = (int8_t)(((i_to - i_from) + 6) % 6);   /* 0..5 */
    if (span > 3) span = (int8_t)(span - 6);      /* 4 -> -2, 5 -> -1 */
    if ((span == 3) || (span == -3)) return 0;    /* 跨 3 步无法判方向 */
    return span;
}

/*=========================== 霍尔位置角度表 =======================*/
/* 每个霍尔位置占一个 60° 区间，下列是区间起始角：
 * 6(110):0~60  2(010):60~120  3(011):120~180  1(001):180~240  5(101):240~300  4(100):300~360 */
static const float phase_start_angle[8] = {
    0.0f,                       /* 0 无效 */
    _PI_3 * 3.0f,               /* 1 = 180° */
    _PI_3 * 1.0f,               /* 2 = 60°  */
    _PI_3 * 2.0f,               /* 3 = 120° */
    _PI_3 * 5.0f,               /* 4 = 300° */
    _PI_3 * 4.0f,               /* 5 = 240° */
    0.0f,                       /* 6 = 0°   */
    0.0f,                       /* 7 无效 */
};

/*=========================== 数学小工具 ==============================*/
static float HALL_Wrap0To2Pi(float angle)
{
    while (angle >= _2PI) angle -= _2PI;
    while (angle < 0.0f)  angle += _2PI;
    return angle;
}

static float HALL_WrapMinusPiToPi(float angle)
{
    while (angle >  _PI) angle -= _2PI;
    while (angle < -_PI) angle += _2PI;
    return angle;
}

/* 沿正转方向走 n 个扇区 */
static uint8_t HALL_SectorStep(uint8_t s, uint8_t n)
{
    while (n-- > 0U) s = sect_next[s & 0x07U];
    return s;
}

/** @brief  取一条完整一致的霍尔边沿信息
  * @return  该快照对应的边沿序号；与上次相同说明没有新边沿
  * @note   只用十来个时钟周期的关中断把 period/phase/dir 一次性拷出，即使被 20kHz
  *         的 FOC 中断打断也不会读到"改了一半"的数据；开销约 0.02%。 */
static uint32_t HALL_ReadEdge(HALL_Edge_TypeDef *out)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t cnt;

    __disable_irq();
    out->period = s_edge_period;
    out->phase  = s_edge_phase;
    out->dir    = s_edge_dir;
    cnt         = s_edge_cnt;
    __set_PRIMASK(primask);
    return cnt;
}

/** @brief 一致读取总边沿计数和有符号净边沿计数 */
static void HALL_ReadMotionCounts(uint32_t *edge_count, int32_t *signed_edges)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *edge_count  = s_edge_cnt;
    *signed_edges = s_signed_edges;
    __set_PRIMASK(primask);
}

/** @brief 临界区读取六边沿周期缓存摘要 */
static uint8_t HALL_ReadRing(uint32_t *sum, uint8_t *count, int8_t *dir)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t i;

    __disable_irq();
    *sum = 0U;
    for (i = 0U; i < s_ring_count; i++)
    {
        *sum += s_period_ring[i];
    }
    *count = s_ring_count;
    *dir   = s_ring_dir;
    __set_PRIMASK(primask);
    return *count;
}

/** @brief 最新 Hall 速度是否仍在有效期；过期则认为停转 */
static uint8_t HALL_SpeedIsFresh(void)
{
    uint32_t age = s_ms_tick - s_edge_ms;
    uint32_t timeout_ms;

    if (s_edge_period == 0U)
    {
        timeout_ms = HALL_EDGE_LOST_MS_MIN;
    }
    else
    {
        timeout_ms = (2U * s_edge_period) / (uint32_t)HALL_TICK_PER_MS;
        if (timeout_ms < HALL_EDGE_LOST_MS_MIN) timeout_ms = HALL_EDGE_LOST_MS_MIN;
        if (timeout_ms > HALL_EDGE_LOST_MS_MAX) timeout_ms = HALL_EDGE_LOST_MS_MAX;
    }

    return (age <= timeout_ms) ? 1U : 0U;
}

/** @brief 根据有符号边沿和当前扇区内插值角计算未减零点的原始机械角 */
static float HALL_CalcRawMech(void)
{
    float edge_m = (float)s_signed_edges * _PI_3 / HALL_POLE_PAIR;
    float start;
    float frac;

    if (s_phase_cur == 0U || s_phase_cur == 7U || s_dir_cur == 0)
    {
        return edge_m;
    }

    start = phase_start_angle[s_phase_cur];
    /* frac 必须始终相对"本扇区起点"量取，与转向无关：经过 HALL_GetElecAngle() 的
     * 扇区钳位后，s_hall_ang 一定落在 [start, start+60°] 内，所以两种转向都用
     * s_hall_ang - start 才对；换向时改量到 start+60° 会让机械角瞬间少一个扇区。 */
    frac = s_hall_ang - start;
    if (frac > _PI_3) frac = _PI_3;
    if (frac < 0.0f)  frac = 0.0f;
    return edge_m + frac / HALL_POLE_PAIR;
}

static void HALL_SpeedObserver_Reset(float mech)
{
    uint32_t edge_count;
    int32_t signed_edges;

    HALL_ReadMotionCounts(&edge_count, &signed_edges);
    s_speed_obs.window_cnt        = 0U;
    s_speed_obs.initialized       = 1U;
    s_speed_obs.edge_oscillating  = 0U;
    s_speed_obs.mech_last         = mech;
    s_speed_obs.mech_rpm          = 0.0f;
    s_speed_obs.fused_rpm         = 0.0f;
    s_speed_obs.edge_last         = edge_count;
    s_speed_obs.signed_edge_last  = signed_edges;
}

/** @brief 更新低速净位移测速，并与单边沿 T 法按速度区间融合 */
static void HALL_SpeedObserver_Update(float mech)
{
    float rpm_hall;
    float speed_abs;

    if (s_speed_obs.initialized == 0U)
    {
        HALL_SpeedObserver_Reset(mech);
        return;
    }

    if (++s_speed_obs.window_cnt >= SPEED_MEAS_WINDOW_DIV)
    {
        uint32_t edge_now;
        int32_t signed_now;
        uint32_t edge_delta;
        int32_t signed_delta;
        uint32_t net_edges;
        float window_s = (float)SPEED_MEAS_WINDOW_DIV * ts;
        float mech_delta = s_hall_dir * (mech - s_speed_obs.mech_last);

        HALL_ReadMotionCounts(&edge_now, &signed_now);
        edge_delta = edge_now - s_speed_obs.edge_last;
        signed_delta = signed_now - s_speed_obs.signed_edge_last;
        net_edges = (signed_delta < 0)
                  ? (uint32_t)(-signed_delta)
                  : (uint32_t)signed_delta;

        s_speed_obs.window_cnt = 0U;
        s_speed_obs.mech_rpm = mech_delta * 60.0f / (_2PI * window_s);
        if (s_speed_obs.mech_rpm >  SPEED_FB_RPM_MAX) s_speed_obs.mech_rpm =  SPEED_FB_RPM_MAX;
        if (s_speed_obs.mech_rpm < -SPEED_FB_RPM_MAX) s_speed_obs.mech_rpm = -SPEED_FB_RPM_MAX;

        /* 同一窗口出现正反向边沿时，总边沿数大于净边沿数。此时局部往返不应
         * 交给单边沿 T 法作为有效速度，融合器强制采用净位移结果。 */
        s_speed_obs.edge_oscillating =
            ((edge_delta >= 2U) && (net_edges < edge_delta)) ? 1U : 0U;

        s_speed_obs.mech_last        = mech;
        s_speed_obs.edge_last        = edge_now;
        s_speed_obs.signed_edge_last = signed_now;
    }

    rpm_hall = RPM_FROM_WE(HALL_GetElecOmega());
    speed_abs = fabsf(s_speed_obs.mech_rpm);

    if ((s_speed_obs.edge_oscillating != 0U) ||
        (speed_abs <= SPEED_MEAS_LOW_RPM))
    {
        s_speed_obs.fused_rpm = s_speed_obs.mech_rpm;
    }
    else if (speed_abs >= SPEED_MEAS_HIGH_RPM)
    {
        s_speed_obs.fused_rpm = rpm_hall;
    }
    else
    {
        float ratio = (speed_abs - SPEED_MEAS_LOW_RPM)
                    / (SPEED_MEAS_HIGH_RPM - SPEED_MEAS_LOW_RPM);
        s_speed_obs.fused_rpm = s_speed_obs.mech_rpm
                              + ratio * (rpm_hall - s_speed_obs.mech_rpm);
    }

    if (s_speed_obs.fused_rpm >  SPEED_FB_RPM_MAX) s_speed_obs.fused_rpm =  SPEED_FB_RPM_MAX;
    if (s_speed_obs.fused_rpm < -SPEED_FB_RPM_MAX) s_speed_obs.fused_rpm = -SPEED_FB_RPM_MAX;
}

/** @brief  清空同方向周期滑动缓存 */
static void HALL_ResetPeriodRing(int8_t dir)
{
    uint8_t i;
    for (i = 0U; i < HALL_PERIOD_RING_SIZE; i++)
    {
        s_period_ring[i] = 0U;
    }
    s_ring_head  = 0U;
    s_ring_count = 0U;
    s_ring_dir   = dir;
}

/** @brief  写入一个有效 Hall 周期；方向变化时丢弃旧方向周期，避免混合测速
  * @note   往返边沿由 IRQ 层去抖/回退判据/方向跟随处理，不靠 ring 过滤。 */
static void HALL_PushPeriod(uint32_t period, int8_t dir)
{
    if ((s_ring_count == 0U) || (s_ring_dir != dir))
    {
        HALL_ResetPeriodRing(dir);
    }

    s_period_ring[s_ring_head] = period;
    s_ring_head = (uint8_t)((s_ring_head + 1U) % HALL_PERIOD_RING_SIZE);
    if (s_ring_count < HALL_PERIOD_RING_SIZE)
    {
        s_ring_count++;
    }
}

/** @brief  获取当前霍尔电平
  * @return  0~7 霍尔编码值，CH1/A 为 bit2，CH3/C 为 bit0（每次实时读 GPIO） */
int32_t HALL_GetPhase(void)
{
    int32_t p = 0;
    p  = HAL_GPIO_ReadPin(HALL_CH1_PORT, HALL_CH1_PIN) << 2;   /* A(U) */
    p |= HAL_GPIO_ReadPin(HALL_CH2_PORT, HALL_CH2_PIN) << 1;   /* B(V) */
    p |= HAL_GPIO_ReadPin(HALL_CH3_PORT, HALL_CH3_PIN);        /* C(W) */
    return p & 0x07;
}

/** @brief  霍尔模块初始化
  * @note   配置 TIM5 时间基准与中断优先级，清零全部内部状态，再启动霍尔接口并开中断 */
void HALL_Speed_Init(HALLParm_TypeDef *state_hall)
{
    uint8_t phase;

    s_pHall   =  state_hall ;

    /* 时间基准：CubeMX 默认 PSC=83(1MHz)，16bit 最多表示 65.5ms 边沿间隔，4 极对
     * 电机低于约 38rpm 就会回绕 → 转速虚高 → 失步、必须手推才能起动。故取 PSC=419
     * → 200kHz（5us/tick），可测到 327ms ≈ 7.6rpm，分辨率仍够；写在用户代码里，
     * CubeMX 重新生成工程也不会丢（.ioc 里同步改亦可）。
     * 不修改 NVIC 优先级 —— 交给 CubeMX 管；边沿信息用临界区快照读（HALL_ReadEdge），
     * 优先级怎么配都不会读到半成品。 */
    state_hall->first        =  1;
    state_hall->last_period  =  0;
    phase = (uint8_t)HALL_GetPhase();
    state_hall->last_phase = phase;
    s_edge_cnt    = 0;
    s_last_ccr    = 0;
    s_edge_phase  = (phase > 0U && phase < 7U) ? phase : 0U;
    s_edge_dir    = 0;
    s_edge_period = 0;
    s_edge_ms     = 0;
    s_ms_tick     = 0;
    HALL_ResetPeriodRing(0);
    s_signed_edges       = 0;
    s_breakaway_latch    = 0U;
    s_hall_ang       = (phase > 0U && phase < 7U)
                  ? HALL_Wrap0To2Pi(phase_start_angle[phase] + _PI_3 * 0.5f)
                  : 0.0f;
    s_hall_spd       = 0.0f;
    s_angle_spd      = 0.0f;
    s_phase_trim        = 0.0f;
    s_zero_ofs          = 0.0f;
    s_hall_dir       = 1.0f;
    s_est_valid      = 0U;
    s_phase_cur   = s_edge_phase;
    s_dir_cur     = 0;
    s_edge_seen   = 0;
    s_lost_limit  = HALL_EDGE_LOST_MS_MIN;
    /* 机械角零点在标定结束时根据当前位置建立 */
    s_mech_zero       = 0.0f;
    s_mech_raw_out    = 0.0f;
    s_mech_angle_out  = 0.0f;
    HALL_SpeedObserver_Reset(0.0f);
    s_calib_sec1     = (phase > 0U && phase < 7U) ? phase : 0U;
    s_calib_sec2     = s_calib_sec1;
    s_calib_h0_fb    = 0.0f;
    s_calib_sigma_fb = 1;
    s_calib_theta    = 0.0f;
    s_calib_start    = 0.0f;
    s_calib_seen     = 0;
    s_calib_hits     = 0;
    s_calib_sigma    = 1;
    s_calib_h0ref    = 0.0f;
    s_calib_theta1   = 0.0f;
    s_calib_h0       = 0.0f;
    s_calib_ok       = 0;
    s_calib_fail     = 0;
    HAL_TIMEx_HallSensor_Start_IT(&htim5);
}

/** @brief  TIM5 霍尔接口捕获中断回调
  * @note   只做方向判断/周期读取/边沿计数/时间戳（硬件已清零 CNT、CCR1 锁存间隔），
  *         角度更新放在 20kHz FOC 中断里做。 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint8_t ph;
    uint8_t last;
    int8_t  d;
    int8_t  span;
    uint32_t ccr;

    if (htim->Instance != TIM5) return;
    if (s_pHall == NULL) return;              /* 未初始化退出 */
    ph   = (uint8_t)HALL_GetPhase();
    if (ph == 0U || ph == 7U)
    {
        /* 非法 Hall 状态：不更新相位、不刷新有效边沿时间，防止延迟停转判定 */
        return;
    }

    /* 先取本次捕获的间隔（TIM5 计数被霍尔硬件在边沿上清零，CCR1 = 距上一边沿的 tick 数）。
     * 必须在去抖判据之前取，否则去抖读到的是未初始化值、形同未生效。 */
    ccr = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if ((s_last_ccr != 0U) && (ccr < s_debounce_ticks))
    {
        /* 毛刺：完全忽略（不计数、不算速、不同步相位）。
         * 注意：绝不能把毛刺时刻的霍尔相位同步给 s_phase_cur —— EMI 毛刺读到
         * 的 GPIO 状态不可信，同步后会污染 FOC 电角度、力矩方向错电机锁死。
         * last_phase 也不更新：毛刺后真实边沿到来时方向表仍按旧状态判，不会错。 */
        return;
    }
    s_last_ccr = ccr;

    last = s_pHall->last_phase;
    /* 第一次有效中断只记录相位，不算速度 */
    if (s_pHall->first) {
        s_pHall->last_phase = ph;
        s_pHall->first      = 0;
        s_edge_phase        = ph;
        return;
    }
    d = dir_tab[last][ph];
    /* last_phase 无条件更新（自同步）：高速/大电流下霍尔中断可能合并（一次跨两个
     * 扇区，如 1->3->5 直读成 1->5，dir_tab 得 d==0），若不更新则后续真实边沿全部
     * 判 d==0，换相链断。
     * 合并过渡的成因是"转速高 + 转矩脉动大 ⇒ 两个真边沿的间隔短于去抖窗"，不是
     * EMI 毛刺：必须按环上实际跨过的步数补计数（span=±2），否则 s_signed_edges
     * （机械角/PLL/SpeedFB 的唯一来源）会随转速与电流抖动系统性少算近一半。
     * 无法判向的跳变（同状态、跨 3 步、含非法状态）仍只同步扇区、不计数、
     * 不伪造速度。 */
    s_pHall->last_phase = ph;
    span = (d != 0) ? d : HALL_RingSpan(last, ph);
    if (span == 0) {
        s_edge_phase = ph;
        return;
    }
    if (ccr == 0U) return;
    if ((span == 2) || (span == -2)) {
        ccr >>= 1;   /* 两步的间隔折算成单扇区等效周期，供测速/插值使用 */
    }
    d = (span > 0) ? (int8_t)1 : (int8_t)-1;   /* 周期/方向语义统一成"一步" */
    s_pHall->last_period = ccr;
    /* 停转/长时间没有边沿后重新起转：滑窗里残留的是卡滞之前的旧周期，已不代表当前
     * 转速。若继续拿它做角度外推，估算角会在脱开瞬间一次冲出去（低速冲转的成因之一），
     * 所以先清空滑窗，让本次周期单独作为速度基准。 */
    if ((s_edge_ms != 0U) && ((s_ms_tick - s_edge_ms) > (uint32_t)s_lost_limit))
    {
        HALL_ResetPeriodRing(d);
    }
    /* 起转(脱开静摩擦/齿槽卡滞)检测：边沿间隔明显长于正常周期 → 转子刚刚从静止
     * 开始动。上层状态机靠它立刻砍掉摩擦前馈；若等 10ms 测速窗 + 30Hz 低通去发现
     * 起转要 15~20ms，低速下这段时间足够转子冲到几百 rpm。 */
    if ((s_edge_ms != 0U) && ((s_ms_tick - s_edge_ms) > HALL_BREAKAWAY_MS))
    {
        s_breakaway_latch = 1U;
    }
    HALL_PushPeriod(ccr, d);
    s_signed_edges += (int32_t)span;
    /* 提交本次边沿信息，s_edge_cnt 最后写 */
    s_edge_period = ccr;
    s_edge_dir    = d;
    s_edge_phase  = ph;
    s_edge_ms     = s_ms_tick;
    s_edge_cnt++;
}

/*=========================== 零位标定 ==============================*/

/** @brief  第一次对齐(0°电角度)结束：记录当前霍尔扇区 */
void HALL_CalibAlign1(void)
{
    uint8_t ph = (uint8_t)HALL_GetPhase();
    if (ph > 0U && ph < 7U) s_calib_sec1 = ph;
}

/** @brief  第二次对齐(180°电角度)结束：记录扇区并算出"兜底零位"
  * @note   兜底零位只取两次对齐的扇区中点（约 ±30°），仅在拖动标定失败时使用 */
void HALL_CalibAlign2(void)
{
    uint8_t ph = (uint8_t)HALL_GetPhase();
    float   a1;
    float   a2;
    float   h0b;
    float   diff;
    int8_t  sg = 1;

    if (ph > 0U && ph < 7U) s_calib_sec2 = ph;
    a1 = phase_start_angle[s_calib_sec1] + _PI_3 * 0.5f;
    a2 = phase_start_angle[s_calib_sec2] + _PI_3 * 0.5f;
    /* 0°→180° 时扇区序号应正转 3 步；判不出（转子没停准）就保持 +1 */
    if (s_calib_sec1 > 0U && s_calib_sec1 < 7U &&
        s_calib_sec2 > 0U && s_calib_sec2 < 7U)
    {
        if (HALL_SectorStep(s_calib_sec1, 3U) != s_calib_sec2)
        {
            if (HALL_SectorStep(s_calib_sec2, 3U) == s_calib_sec1)
            {
                sg = -1;
            }
        }
    }
    /* 两次对齐把 h0 夹在一个区间里，取区间中点 */
    h0b  = HALL_Wrap0To2Pi(a2 - (float)sg * _PI);
    diff = HALL_WrapMinusPiToPi(h0b - a1);
    s_calib_h0_fb    = HALL_Wrap0To2Pi(a1 + diff * 0.5f);
    s_calib_sigma_fb = sg;
}

/** @brief  开始零位细分标定（缓慢拖动转子跨越霍尔边沿）
  * @param[in]  theta_start 起始电角度（= 当前实际施加的 FOC 电角度） */
void HALL_CalibRampBegin(float theta_start)
{
    /* 先和当前霍尔状态对齐：last_phase 必须最新，否则拖动后的第一个边沿查表得到
     * dir=0 被丢掉；'first' 也要清掉，否则对齐期间没动过时会吞掉一个标定样本。 */
    if (s_pHall != NULL)
    {
        uint8_t ph = (uint8_t)HALL_GetPhase();
        if (ph > 0U && ph < 7U)
        {
            s_pHall->last_phase = ph;
            s_edge_phase        = ph;
        }
        s_pHall->first = 0U;
    }
    s_calib_theta  = theta_start;
    s_calib_start  = theta_start;
    s_calib_seen   = s_edge_cnt;
    s_calib_hits   = 0;
    s_calib_sigma  = 1;
    s_calib_h0ref  = 0.0f;
    s_calib_theta1 = 0.0f;
    s_calib_h0     = 0.0f;
    s_calib_ok     = 0;
    s_calib_fail   = 0;
}

/** @brief  零位细分标定：每拍推进一点电角度；返回 1 = 继续拖动，0 = 结束
  * @param[out] theta_cmd 本拍应施加给 FOC 的电角度
  * @note   用 Id 把转子"粘"在 d 轴上跟着施加角走，边沿瞬间 θe = 当前施加角、
  *         h = 扇区边界 B，由 θe = σ·(h - h0) 反算 h0；两个边沿间隔必须恰好 60°，
  *         否则谐波/抖动/转子没跟上都会被判失败并退回兜底值。 */
uint8_t HALL_CalibRampRun(float *theta_cmd)
{
#if (HALL_CALIB_ENABLE == 0)
    s_calib_fail = 1U;
    *theta_cmd   = ALIGN2_ANGLE;
    return 0U;
#else
    float swept;

    if (s_calib_ok != 0U || s_calib_fail != 0U) return 0U;
    s_calib_theta += HALL_CALIB_RATE * ts;
    if (s_calib_theta >= _2PI) s_calib_theta -= _2PI;
    *theta_cmd = s_calib_theta;
    /* 拖过一个整圈还没标定成功 → 放弃 */
    swept = HALL_Wrap0To2Pi(s_calib_theta - s_calib_start);
    if (swept > HALL_CALIB_MAX_SWEEP) { s_calib_fail = 1U; return 0U; }
    if (s_edge_cnt != s_calib_seen)
    {
        HALL_Edge_TypeDef e;
        uint8_t ph;
        int8_t  d;

        s_calib_seen = HALL_ReadEdge(&e);      /* 临界区快照 */
        ph           = e.phase;
        d            = e.dir;
        if (ph > 0U && ph < 7U && d != 0)
        {
            /* 边界取决于行进方向：d>0 取扇区起点，d<0 取起点+60°。漏掉这个区分，
             * σ=-1 时会整整偏一个扇区，表现是标定"成功"但运行角度恒定偏 60°。 */
            float B  = HALL_Wrap0To2Pi(phase_start_angle[ph] + ((d > 0) ? 0.0f : _PI_3));
            float sg = (float)d;
            float h0 = HALL_Wrap0To2Pi(B - sg * s_calib_theta);
            if (s_calib_hits == 0U)
            {
                s_calib_hits   = 1U;
                s_calib_sigma  = d;
                s_calib_h0ref  = h0;
                s_calib_theta1 = s_calib_theta;
            }
            else
            {
                float dth;

                if (d != s_calib_sigma) { s_calib_fail = 1U; return 0U; }
                /* 两次边沿之间转子应恰好转过 60° 电角度 */
                dth = HALL_Wrap0To2Pi(s_calib_theta - s_calib_theta1);
                if (fabsf(dth - _PI_3) > HALL_CALIB_TOL) { s_calib_fail = 1U; return 0U; }
                /* 两次算出的零位取圆周平均，降噪 */
                s_calib_h0 = HALL_Wrap0To2Pi(
                                 s_calib_h0ref
                               + HALL_WrapMinusPiToPi(h0 - s_calib_h0ref) * 0.5f);
                s_calib_ok = 1U;
                return 0U;
            }
        }
    }
    return 1U;
#endif
}

/** @brief  标定结束落地：锁定零位/符号，把估算器复位到已知角度
  * @note   成功用细分零位（约几度），失败自动退回扇区中点零位（±30°） */
void HALL_CalibFinish(void)
{
    if (s_calib_ok != 0U)
    {
        s_zero_ofs    = s_calib_h0;
        s_hall_dir = (float)s_calib_sigma;
        /* θe = σ·(h - h0)  =>  h = h0 + σ·θe */
        s_hall_ang = HALL_Wrap0To2Pi(s_zero_ofs + s_hall_dir * s_calib_theta);
        /* 拖动时转子正以 HALL_CALIB_RATE 转动，用它初始化转速，进 RUN 第一拍就能外推 */
        s_hall_spd = s_hall_dir * HALL_CALIB_RATE;
    }
    else
    {
        uint8_t ph = (uint8_t)HALL_GetPhase();
        s_zero_ofs    = s_calib_h0_fb;
        s_hall_dir = (float)s_calib_sigma_fb;
        if (ph == 0U || ph == 7U) ph = s_edge_phase;
        if (ph == 0U || ph == 7U) ph = s_calib_sec2;
        s_hall_ang = HALL_Wrap0To2Pi(phase_start_angle[ph] + _PI_3 * 0.5f);
        s_hall_spd = 0.0f;
    }
    s_phase_trim      = 0.0f;
    s_angle_spd       = 0.0f;
    HALL_ResetPeriodRing(0);
    s_signed_edges    = 0;
    s_breakaway_latch = 0U;
    s_est_valid    = 1U;
    s_edge_seen = s_edge_cnt;
    s_lost_limit = HALL_EDGE_LOST_MS_MIN;
    if (s_pHall != NULL)
    {
        uint8_t ph = (uint8_t)HALL_GetPhase();
        if (ph == 0U || ph == 7U) ph = s_calib_sec2;
        if (ph == 0U || ph == 7U) ph = s_edge_phase;
        s_edge_phase         = ph;
        s_phase_cur          = ph;          /* 钳位用的扇区必须一起同步 */
        s_dir_cur            = (int8_t)s_hall_dir;
        s_pHall->last_phase  = ph;
        s_pHall->last_period = 0U;
        /* 当前霍尔状态已知，运行后第一个状态变化即可判断方向并接管角度 */
        s_pHall->first       = 0U;
        s_mech_raw_out       = HALL_CalcRawMech();
        s_mech_zero          = s_mech_raw_out;
        s_mech_angle_out     = 0.0f;
        HALL_SpeedObserver_Reset(s_mech_angle_out);
    }
    __HAL_TIM_SET_COUNTER(&htim5, 0);
}

/** @brief  零位细分标定是否成功（0 = 用了兜底零位） */
uint8_t HALL_IsCalibrated(void)
{
    return s_calib_ok;
}

/*=================== 力矩方向自检（上电必做，防全速飞车） ===================*/
/* 拖动标定的零位只能确定到"模 π"（0° 与 180° 两个对齐点数学上分不出来）。保持施加
 * 角不动、给一个小 +Iq：转子往 +Theta 走则极性正确，往 -Theta 走则零位差 π，补上 π。
 * 少了这步，转子停在反相位置时会闭环极性反 → 上电全速飞车。
 * 结束条件取"出现新霍尔边沿"：低速段估算角只靠边沿更新，必须真实跨过 60° 边界。 */
static float    s_dirchk_hold;      /* 自检期间保持的电角度 */
static float    s_dirchk_acc;       /* 相对保持角的总偏移（带符号） */
static uint32_t s_dirchk_seen;      /* 开始时的边沿计数 */
static uint16_t s_dirchk_cnt;       /* 计时 */
static uint8_t  s_dirchk_done;

void HALL_DirCheckBegin(void)
{
    s_dirchk_hold = HALL_Wrap0To2Pi(s_hall_dir * (s_hall_ang - s_zero_ofs));
    s_dirchk_seen = s_edge_cnt;
    s_dirchk_acc  = 0.0f;
    s_dirchk_cnt  = 0;
    s_dirchk_done = 0U;
}

uint8_t HALL_DirCheckRun(float *theta_cmd)
{
    float now;

    *theta_cmd = s_dirchk_hold;          /* 角度保持不动，转子才会被"推着"走 */
    if (s_dirchk_done != 0U) return 0U;
    now = HALL_GetElecAngle(s_pHall);    /* 让估算器继续跟踪转子 */
    s_dirchk_acc = HALL_WrapMinusPiToPi(now - s_dirchk_hold);
    if (s_dirchk_cnt < 0xFFFFU) s_dirchk_cnt++;
    if ((s_edge_cnt != s_dirchk_seen) ||
        (s_dirchk_cnt >= (uint16_t)(HALL_DIRCHK_MS * (Tim_Hz / 1000U))))
    {
        s_dirchk_done = 1U;
    }
    return 1U;
}

void HALL_DirCheckFinish(void)
{
    /* 只有真的等到边沿（观察到位移）且方向相反时才补 π；超时（没动/堵转）不改 */
    if ((s_dirchk_cnt < (uint16_t)(HALL_DIRCHK_MS * (Tim_Hz / 1000U))) &&
        (s_dirchk_acc < 0.0f))
    {
        s_zero_ofs = HALL_Wrap0To2Pi(s_zero_ofs + _PI);
    }
    /* 方向自检会主动推动转子，进入 RUN 前丢弃这段运动产生的测速历史。 */
    HALL_SpeedObserver_Reset(s_mech_angle_out);
}

/*=========================== 电角度估算 ==============================*/

/* ① 边沿处理：同步扇区/方向、边沿测速、算相位误差 s_phase_trim、更新丢边沿门限 */
static void Hall_StepOnEdge(void)
{
    if (s_edge_cnt != s_edge_seen)
    {
        HALL_Edge_TypeDef e;
        uint32_t period;
        uint8_t  eph;
        int8_t   d;
        uint32_t ring_sum;
        uint8_t  ring_n;
        int8_t   ring_dir;
        float    Tavg;

        s_edge_seen = HALL_ReadEdge(&e);       /* 临界区快照，一次拿全 */
        period      = e.period;
        eph         = e.phase;
        d           = e.dir;
        if (period > 0U && d != 0 && eph > 0U && eph < 7U)
        {
            float Tp = (float)period * HALL_TICK_S;   /* 本扇区耗时 s */
            float w  = (float)d * _PI_3 / Tp;         /* 本扇区平均电角速度 */
            /* 这一步是否发生了换向：s_dir_cur 还保持着上一次的有效方向 */
            uint8_t dir_flip = (s_dir_cur != 0 && s_dir_cur != d) ? 1U : 0U;
            if (w >  HALL_OMEGA_E_MAX) w =  HALL_OMEGA_E_MAX;
            if (w < -HALL_OMEGA_E_MAX) w = -HALL_OMEGA_E_MAX;
            if (s_est_valid == 0U)
            {
                /* 首次锁定：扇区已知，先放到扇区中点，边沿会被慢慢校正 */
                s_hall_ang  = HALL_Wrap0To2Pi(phase_start_angle[eph] + _PI_3 * 0.5f);
                s_hall_spd  = w;
                s_angle_spd = w;
                s_phase_trim   = 0.0f;
                s_est_valid = 1U;
            }
            else
            {
                float edge_theta = HALL_Wrap0To2Pi(
                                       phase_start_angle[eph] + ((d > 0) ? 0.0f : _PI_3));
                float err;
                float lim;

                /* 边沿测速只在"小范围波动"时走低通；符号相反/过零、方向翻转(dir_flip)、
                 * 幅度超 100% 这三种必须直接跟随新值 —— 换向时若还慢慢低通，估算转速会
                 * 在零附近磨蹭甚至保持旧符号，估算角顶着反方向走、最后卡在 90°，
                 * 电流全落在 d 轴上没有转矩。 */
                /* ---- 回退型假边沿拒绝 ----
                 * 方向未变但边沿目标角相对当前估算角"回退 >5°"：转子瞬时回退
                 * 60° 电不可能，判假边沿整体忽略（不 snap/不相位校正/不更新相位
                 * 方向/不污染测速）。助推期间电角度开环爬升，真边沿必然同向前进
                 * 不误伤；漏边沿后的补校同为前进不误伤。 */
                if ((dir_flip == 0U) && (s_dir_cur != 0))
                {
                    /* 只在估算角尚未超前本扇区终点(rel<=60°)时执行回退判据：
                     * 高速真边沿到达时估算角往往已外推超前(rel>60°)，edge_theta=扇区
                     * 起点相对回退属正常校正，执行判据会误伤真边沿。 */
                    float _rel = HALL_Wrap0To2Pi(s_hall_ang - phase_start_angle[s_phase_cur]);
                    if (_rel <= _PI_3)
                    {
                        float _delta = HALL_WrapMinusPiToPi(edge_theta - s_hall_ang);
                        if (((s_dir_cur > 0) && (_delta < -HALL_EDGE_BACK_LIM)) ||
                            ((s_dir_cur < 0) && (_delta >  HALL_EDGE_BACK_LIM)))
                        {
                            s_phase_trim = 0.0f;
                            return;   /* 假边沿：整体忽略（不更新相位/方向/不 snap） */
                        }
                    }
                }
                if (dir_flip ||
                    ((w * s_hall_spd) <= 0.0f) ||
                    (fabsf(w - s_hall_spd) > HALL_SPEED_JUMP_RATIO * (fabsf(s_hall_spd) + 1.0f)))
                {
                    s_hall_spd = w;
                }
                else
                {
                    s_hall_spd += HALL_SPEED_ALPHA * (w - s_hall_spd);
                }
                if (dir_flip)
                {
                    /* 反向边沿无条件跟随霍尔：刚换向时旧方向与旧相位校正全部作废，
                     * 在霍尔边界上干净地重新对齐；假反向边沿由非相邻忽略/回退判据/
                     * 外推限幅前置拦截。 */
                    s_hall_ang = edge_theta;
                    s_phase_trim  = 0.0f;
                }
                else
                {
                    /* 必须用"还没被钳位"的 s_hall_ang 算误差；把它折算成下一扇区内的
                     * 一段速度偏置，一个扇区正好吃完（死拍），角度连续、速度也连续。 */
                    err = HALL_WrapMinusPiToPi(edge_theta - s_hall_ang);
                    if (fabsf(err) > HALL_SNAP_ERR)
                    {
                        /* 偏差过大（漏边沿/测速偏差）：直接在边界上 snap 重新对齐，
                         * 比慢慢校正干净，避免错位持续导致电流饱和。 */
                        s_hall_ang = edge_theta;
                        s_phase_trim  = 0.0f;
                    }
                    else
                    {
                        /* 残余误差折算成下一扇区内的相位校正速度（限幅），一个扇区吃完 */
                        s_phase_trim = HALL_PHASE_GAIN * err / Tp;
                        lim = fabsf(s_angle_spd) * HALL_PHASE_CORR_RATIO;
                        if (lim < HALL_CORR_MIN_OMEGA) lim = HALL_CORR_MIN_OMEGA;
                        if (s_phase_trim >  lim) s_phase_trim =  lim;
                        if (s_phase_trim < -lim) s_phase_trim = -lim;
                    }
                }
            }
            /* 六边沿平均周期用于角度插值，避免上一次冲转的高速度让估算角提前跑到扇区终点 */
            ring_n = HALL_ReadRing(&ring_sum, &ring_n, &ring_dir);
            if (ring_n > 0U && ring_dir != 0 && ring_sum > 0U)
            {
                float _w6;
                Tavg = ((float)ring_sum / (float)ring_n) * HALL_TICK_S;
                _w6  = (float)ring_dir * _PI_3 / Tavg;
                /* 外推速度始终跟随最新边沿周期，避免旧值冻结导致估算角锁死 */
                s_angle_spd = _w6;
                if (s_angle_spd >  HALL_OMEGA_E_MAX) s_angle_spd =  HALL_OMEGA_E_MAX;
                if (s_angle_spd < -HALL_OMEGA_E_MAX) s_angle_spd = -HALL_OMEGA_E_MAX;
            }
            /* 丢边沿判定门限：按上次周期自适应，40~400ms */
            {
                uint32_t lim_ms = (uint32_t)(((float)period * (float)HALL_EDGE_LOST_RATIO)
                                             / HALL_TICK_PER_MS);
                if (lim_ms < HALL_EDGE_LOST_MS_MIN) lim_ms = HALL_EDGE_LOST_MS_MIN;
                if (lim_ms > HALL_EDGE_LOST_MS_MAX) lim_ms = HALL_EDGE_LOST_MS_MAX;
                s_lost_limit = (uint16_t)lim_ms;
            }
        }
        /* 扇区始终以最新霍尔状态为准（异常边沿也要同步，否则钳位会用错区间）；
         * 方向只在方向判断有效时才更新，免得被噪声边沿带偏。 */
        if (eph > 0U && eph < 7U)
        {
            s_phase_cur = eph;
        }
        if (d != 0)
        {
            s_dir_cur = d;
        }
    }
}

/* ② 电角度输出：边沿之间做外推，并把它夹在扇区内 */

/* 低速角度助推（I/F 硬拖）请求速度：由 StateMachine 每拍写入。
 * 0     = 不助推：丢边沿时按 HALL_LOST_DECAY_RS 线性衰减外推速度
 * 非 0  = 助推：丢边沿时给外推速度一个同方向的蠕动下限，并放开 60° 扇区钳位，
 *         让电流矢量持续旋转 —— 否则矢量静止在定子上会把转子吸死。 */
static float   s_creep_omega = 0.0f;
static uint8_t s_force_creep = 0U;   /* 堵转强制蠕动：即使有边沿也放开钳位开环拖（位置环堵转破盲区） */
static uint8_t s_lost_creep  = 0U;   /* 本拍是否处于"丢边沿 + 助推"状态 */

/** @brief  启动/切模式时按当前霍尔状态重置估算角到扇区中点，消除上次失步残留 */
void HALL_ResetEstimateForStart(void)
{
    uint8_t ph = (uint8_t)HALL_GetPhase();
    if (ph > 0U && ph < 7U)
    {
        s_phase_cur  = ph;
        s_hall_ang   = HALL_Wrap0To2Pi(phase_start_angle[ph] + _PI_3 * 0.5f);
        s_hall_spd   = 0.0f;
        s_angle_spd  = 0.0f;
        s_phase_trim = 0.0f;
        s_est_valid  = 1U;
    }
}

void HALL_SetCreepOmega(float w)
{
    if (w >  ANGLE_ASSIST_W_CREEP) w =  ANGLE_ASSIST_W_CREEP;
    if (w < -ANGLE_ASSIST_W_CREEP) w = -ANGLE_ASSIST_W_CREEP;
    s_creep_omega = w;
}

/* 堵转强制蠕动开关：1 时即使没丢边沿也按 s_creep_omega 蠕动（放开 60° 钳位），
 * 由位置环堵转检测调用（见 StateMachine.c Run_PositionMode）。0 = 默认只丢边沿才蠕动。 */
void HALL_SetForceCreep(uint8_t on)
{
    s_force_creep = (on != 0U) ? 1U : 0U;
}

/** @brief  获取当前电角度（FOC 直接使用，20kHz 每拍调用）；范围 [0, 2π)
  * @note   主入口：① Hall_StepOnEdge() 先更新边沿状态；② 按转速外推并夹回当前
  *         霍尔扇区；③ 末尾折算成 FOC 坐标系输出，并顺带累加机械角
  *         （电角度每拍增量 / HALL_POLE_PAIR，供 HALL_GetMechAngle 用）。 */
float HALL_GetElecAngle(HALLParm_TypeDef * state)
{
    float    rel;
    float    start;
    uint32_t lost;
    float    omega_cmd;
    uint8_t  ph;
    float    now;

    (void)state;
    s_lost_creep = 0U;
    /*---------- 1. 有边沿就把边沿相关的状态先更新掉 ----------*/
    Hall_StepOnEdge();
    /*---------- 2. 扇区内连续外推 + 兜底钳位 ----------*/
    ph = s_phase_cur;
    if (s_est_valid != 0U && ph > 0U && ph < 7U)
    {
        start = phase_start_angle[ph];
        lost  = s_ms_tick - s_edge_ms;
        if ((lost > (uint32_t)s_lost_limit) ||
            (s_force_creep != 0U && s_creep_omega != 0.0f))
        {
            /* 长时间没有新边沿。旧写法在这里把 s_angle_spd 直接清零 —— 清零点就是低速
             * 自锁的根因：估算角冻死 → 电流矢量静止在定子上 → 转子被吸到 Δ=0（转矩 0）
             * → 转子不动 → 永远没有新边沿。现在分两种处理： */
            s_phase_trim = 0.0f;
            if (s_creep_omega != 0.0f)
            {
                /* 有助推请求：给外推速度一个"同方向的蠕动下限"，让电角度继续转。
                 * 只提高不降低，避免正常高速运行中偶发丢边沿时把转速硬拽下来。 */
                if (s_force_creep != 0U)
                {
                    s_angle_spd = s_creep_omega;   /* 强制蠕动: 速度与方向均=请求值 */
                }
                else if (s_creep_omega > 0.0f)
                {
                    if (s_angle_spd <  ANGLE_ASSIST_W_CREEP) s_angle_spd =  ANGLE_ASSIST_W_CREEP;
                }
                else
                {
                    if (s_angle_spd > -ANGLE_ASSIST_W_CREEP) s_angle_spd = -ANGLE_ASSIST_W_CREEP;
                }
                s_lost_creep = 1U;
            }
            else
            {
                /* 无助推请求：按 HALL_LOST_DECAY_RS(rad/s^2) 线性衰减到 0，不再硬清零 */
                float dw = HALL_LOST_DECAY_RS * ts;
                if      (s_angle_spd >  dw) s_angle_spd -= dw;
                else if (s_angle_spd < -dw) s_angle_spd += dw;
                else                        s_angle_spd  = 0.0f;
                s_lost_creep = 0U;
            }
        }
        /* 方向守卫只限制校正量 s_phase_trim；绝不能拿它限制转速反馈 s_hall_spd：
         * 否则换向瞬间会把整条积分清零，估算角度冻死，电机随后卡住。 */
        if (s_dir_cur > 0 && s_phase_trim < 0.0f) s_phase_trim = 0.0f;
        if (s_dir_cur < 0 && s_phase_trim > 0.0f) s_phase_trim = 0.0f;
        omega_cmd = s_angle_spd + s_phase_trim;
        s_hall_ang = HALL_Wrap0To2Pi(s_hall_ang + omega_cmd * ts);
        /* 关键保护：估算角度必须始终落在 [start, start+60°] 内。霍尔是绝对位置传感器，
         * 转子此刻必定在这个扇区里，所以该窗口不引入误差，却把最坏角度误差锁在 60°
         * 以内 —— cos60°=0.5，转矩方向永不反转，不会被顶到 d 轴卡死。
         * ！！不要再放宽 ！！放宽到"落后一个扇区"会让换向/低速段估算角凭上次测速多跑
         * 100° 以上，电流全落到 d 轴 → 转矩为零、停下起不来。窄窗口是在拽真值。 */
        rel = HALL_Wrap0To2Pi(s_hall_ang - start);
        if ((rel > _PI_3) && (s_lost_creep == 0U))
        {
            /* rel 略大于 60°：向前超了 → 钳到终点；rel 接近 2π：落后 → 钳到起点 */
            rel = (rel > _PI) ? 0.0f : _PI_3;
            s_hall_ang = HALL_Wrap0To2Pi(start + rel);
            s_phase_trim  = 0.0f;
        }
        /* 助推期间(s_lost_creep=1)故意不夹：让估算角越过扇区边界继续前进 = 开环强制换相。
         * 转子跟不上会失步，但电流矢量一直在转，不会把转子吸住；一旦真转子转过 15°
         * 机械角产生新边沿，Hall_StepOnEdge() 会把 s_hall_ang 拉回真值并自动退出该状态。 */
    }
    /*---------- 3. 折算到 FOC 坐标系输出 ----------*/
    now = HALL_Wrap0To2Pi(s_hall_dir * (s_hall_ang - s_zero_ofs));
    s_mech_raw_out   = HALL_CalcRawMech();
    s_mech_angle_out = s_mech_raw_out - s_mech_zero;
    HALL_SpeedObserver_Update(s_mech_angle_out);
    return now;
}

/** @brief 获取最新单边沿电角速度（FOC 坐标系 rad/s，正 = FOC 正转）
  * @note  供高低速融合使用；速度过期后返回 0。 */
float HALL_GetElecOmega(void)
{
    float T;
    float foc_sign;

    if (HALL_SpeedIsFresh() == 0U) return 0.0f;
    T = (float)s_edge_period * HALL_TICK_S;
    foc_sign = s_hall_dir * (float)s_edge_dir;
    return foc_sign * _PI_3 / T;
}

/** @brief 获取低速净位移与单边沿 T 法融合后的机械转速(rpm) */
float HALL_GetSpeedRpm(void)
{
    return s_speed_obs.fused_rpm;
}

/** @brief 获取固定窗口机械角净位移测速(rpm)，供上层摩擦前馈/堵转判定使用 */
float HALL_GetMechSpeedRpm(void)
{
    return s_speed_obs.mech_rpm;
}

/** @brief  取出并清除"刚脱开卡滞"标志（捕获中断置位，上层状态机消费）
  * @return  1 = 自上次读取以来出现过一次"长时间无边沿后重新起转" */
uint8_t HALL_TakeBreakawayFlag(void)
{
    uint8_t f = s_breakaway_latch;
    s_breakaway_latch = 0U;
    return f;
}

/** @brief  获取机械角（rad，连续多圈，可正可负；标定结束时刻为 0）
  * @return 机械角 rad，一转 = 2π，越圈继续累加不回绕
  * @note   由有符号 Hall 边沿计数和扇区内插值分数合成，应在 HALL_GetElecAngle() 后调用 */
float HALL_GetMechAngle(void)
{
    return s_mech_angle_out;
}

/** @brief  把当前机械角置 0（供上层建立位置零点）
  * @note   只更新零点基准，不影响电角度估算 */
void HALL_SetMechAngleZero(void)
{
    s_mech_zero      = s_mech_raw_out;
    s_mech_angle_out = 0.0f;
    HALL_SpeedObserver_Reset(s_mech_angle_out);
}

/** @brief  仅重置速度观测器（不动机械角基准/零位）。
  * @note   上位机切入位置模式时调用：清掉上次运行残留的假速度，避免速度环在
  *         启动瞬间反向制动。机械角/电角度估算不动。 */
void HALL_ResetSpeedObserver(void)
{
    HALL_SpeedObserver_Reset(s_mech_angle_out);
}

/** @brief  动态设置霍尔边沿去抖窗（tick 数，HALL_TICK_HZ=200kHz → 200=1ms）。
  * @note   位置模式切入时调大(如 1000=5ms)抑制堵转大电流 EMI 假边沿；退出恢复默认。
  *         真边沿间隔：40rpm 位置环 ≈62ms、100rpm ≈25ms、200rpm ≈12.5ms，
  *         5ms 窗在任何运行速度下都不误伤真边沿。 */
void HALL_SetDebounceTicks(uint32_t ticks)
{
    if (ticks < 50U)   ticks = 50U;     /* 下限 0.25ms：防误设把真边沿全滤掉 */
    if (ticks > 5000U) ticks = 5000U;   /* 上限 25ms */
    s_debounce_ticks = ticks;
}

/** @brief  SysTick 中断回调，每 1ms 执行一次
  * @note   维护 1ms 自由计数（估算器用它判断"多久没有霍尔边沿"） */
void HAL_SYSTICK_Callback(void)
{
    s_ms_tick++;
}
