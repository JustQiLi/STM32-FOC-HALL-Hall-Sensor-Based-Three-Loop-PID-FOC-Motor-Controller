#include "PID.h"
#include <math.h>

/* 三个环（电流/速度/位置）的全部实现都在本文件。
 * 本文件不认识 StateMachine，也不认识 Hall 模块：每个环只接收自己的状态结构体，
 * 反馈由调用者传进来。这样"谁的状态谁维护"，改一个环不会牵动另一个环。 */

/**
 * @brief PID初始化（清零全部状态，参数保持不变）
 * @param[in] pid PID结构体指针
 * @param[out] 清零后的PID结构体
 * @param[inout] 无
 * @return 无
 * @note 需先调用PID_SetParm/PID_SetLimit设置参数，再调用本函数清零状态
 */
void PID_Init(PIDParm_TypeDef *pid)
{
    pid->Target    = 0.0f;
    pid->Feedback  = 0.0f;
    pid->Error     = 0.0f;
    pid->ErrorLast = 0.0f;
    pid->Integral  = 0.0f;

    pid->OutP = 0.0f;
    pid->OutI = 0.0f;
    pid->OutD = 0.0f;
    pid->Out  = 0.0f;
}

/**
 * @brief 设置PID参数（采样周期T、比例Kp、积分Ki、微分Kd）
 * @param[in] pid PID结构体指针
 * @param[in] t   采样周期（秒）
 * @param[in] kp  比例系数
 * @param[in] ki  积分系数
 * @param[in] kd  微分系数
 * @param[out] 无
 * @param[inout] 无
 * @return 无
 * @note 修改参数后建议重新调用PID_Init清零积分，避免积分跳变
 */
void PID_SetParm(PIDParm_TypeDef *pid, float t, float kp, float ki, float kd)
{
    pid->T  = t;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

/* 在线修改单个PID增益，保持周期和其他增益不变 */
void PID_SetGain(PIDParm_TypeDef *pid, uint8_t which, float value)
{
    switch (which)
    {
        case PID_GAIN_KP: pid->Kp = value; break;
        case PID_GAIN_KI: pid->Ki = value; break;
        case PID_GAIN_KD: pid->Kd = value; break;
        default: break;
    }
}

/**
 * @brief 设置PID输出限幅
 * @param[in] pid     PID结构体指针
 * @param[in] out_max 输出上限
 * @param[in] out_min 输出下限
 * @param[out] 无
 * @param[inout] 无
 * @return 无
 * @note 若out_min > out_max会自动交换，保证限幅区间有效
 */
void PID_SetLimit(PIDParm_TypeDef *pid, float out_max, float out_min)
{
    /* 保证 out_min <= out_max */
    if (out_min > out_max)
    {
        float t = out_max;
        out_max = out_min;
        out_min = t;
    }

    pid->OutMax = out_max;
    pid->OutMin = out_min;
}

/**
 * @brief 单次PID计算（位置式PID + 抗积分饱和 + 输出限幅）
 * @param[in] pid      PID结构体指针
 * @param[in] target   目标值
 * @param[in] feedback 反馈值
 * @param[out] 更新后的PID结构体（Out为计算结果）
 * @param[inout] pid 内部状态（积分/历史误差）被更新
 * @return 无（结果存入 pid->Out）
 * @note 输出 u = Kp*e + Ki*∫e dt + Kd*de/dt
 *       输出被限幅到[OutMin, OutMax]，限幅时回退积分以抗饱和
 */
void PID_Loop(PIDParm_TypeDef *pid, float target, float feedback)
{
    float integral_prev;
    float output;

    /* 1. 更新输入与误差 */
    pid->Target   = target;
    pid->Feedback = feedback;
    pid->Error    = pid->Target - pid->Feedback;

    /* 2. 积分项（保存上次积分用于退饱和） */
    integral_prev = pid->Integral;
    pid->Integral = integral_prev + pid->Error * pid->T;

    /* 3. 微分项（对误差求导，防止T过小除零） */
    if (pid->T > 1e-6f)
        pid->OutD = pid->Kd * (pid->Error - pid->ErrorLast) / pid->T;
    else
        pid->OutD = 0.0f;

    /* 4. 三项求和 */
    pid->OutP = pid->Kp * pid->Error;
    pid->OutI = pid->Ki * pid->Integral;
    output    = pid->OutP + pid->OutI + pid->OutD;

    /* 5. 输出限幅 + 抗积分饱和（限幅时回退本次积分） */
    if (output > pid->OutMax)
    {
        output = pid->OutMax;
        pid->Integral = integral_prev;
        pid->OutI     = pid->Ki * pid->Integral;
    }
    else if (output < pid->OutMin)
    {
        output = pid->OutMin;
        pid->Integral = integral_prev;
        pid->OutI     = pid->Ki * pid->Integral;
    }

    pid->Out = output;

    /* 6. 保存历史误差 */
    pid->ErrorLast = pid->Error;
}

/*=========================== 电流环 ==============================*/
/**
 * @brief 电流环参数初始化：对 Id、Iq 的 PI 结构设置参数并清零状态
 * @note Id、Iq 共用同一组 Kp/Ki/Kd 与限幅，周期就是 FOC 节拍 ts（20kHz）
 */
void PI_Current_Init(CurrentCtrl_TypeDef *cur)
{
    /* Id、Iq状态清零初始化 */
    PID_Init(&cur->d);
    PID_Init(&cur->q);

    /* Id、Iq参数设置 */
    PID_SetParm(&cur->d, ts, CURRENT_KP, CURRENT_KI, CURRENT_KD);
    PID_SetParm(&cur->q, ts, CURRENT_KP, CURRENT_KI, CURRENT_KD);

    /* Id、Iq输出限幅 */
    PID_SetLimit(&cur->d, CURRENT_OUT_MAX, CURRENT_OUT_MIN);
    PID_SetLimit(&cur->q, CURRENT_OUT_MAX, CURRENT_OUT_MIN);

    /* Id、Iq初始目标值设置 */
    cur->d.Target = CURRENT_ID_TARGET;
    cur->q.Target = CURRENT_IQ_TARGET;
}

/**
 * @brief 电流环运行：执行 Id、Iq 两个 PI，结果写入 FOCParm 的 Vd/Vq
 * @param[in,out] cur 电流环状态（Target 由上层给定，如转矩模式/速度环输出）
 * @param[in,out] foc FOC 参数：读 Id/Iq，写 Vd/Vq
 */
void PI_Current_Loop(CurrentCtrl_TypeDef *cur, FOCParm_TypeDef *foc)
{
    /* 进行PI计算 */
    PID_Loop(&cur->d, cur->d.Target, foc->Id);
    PID_Loop(&cur->q, cur->q.Target, foc->Iq);

    /* PI输出Vd、Vq */
    foc->Vd = cur->d.Out;
    foc->Vq = cur->q.Out;
    /* 矢量幅值圆限幅：保证 sqrt(Vd^2+Vq^2) <= CURRENT_OUT_MAX，即线性调制圆半径 Udc/sqrt(3)。
     * 两条单轴限幅(±CURRENT_OUT_MAX)挡不住两轴同时触限，矢量最大可达 sqrt(2)*CURRENT_OUT_MAX。
     * 越圆部分会被 SVPWM 按 ts/(T1+T2) 等比例缩小，即实际施加的电压比这里以为的小，
     * 方向不变。
     * 放在电流环出口，因为 Vd/Vq 就是本环输出；其后的死区补偿每轴最多再叠加 ±0.0855V，
     * 这点余量由 SVPWM 的过调制按比例缩小吸收。常规只多 2 乘 1 加 1 比较，sqrtf 仅越圆时执行。*/
    {
        float vmax = CURRENT_OUT_MAX;
        float m2 = foc->Vd * foc->Vd + foc->Vq * foc->Vq;
        if (m2 > vmax * vmax)
        {
            float k = vmax / sqrtf(m2);
            foc->Vd *= k;
            foc->Vq *= k;
        }
    }
}

/*=========================== 速度环 ==============================*/
/**
 * @brief 速度环参数初始化
 * @note 速度环周期 = SPEED_LOOP_DIV * ts = 1ms（1kHz），输出为Iq目标(A)
 */
void PID_Speed_Init(SpeedCtrl_TypeDef *sc)
{
    PID_Init(&sc->pid);

    PID_SetParm(&sc->pid,
                (float)SPEED_LOOP_DIV * ts,
                SPEED_KP, SPEED_KI, SPEED_KD);

    PID_SetLimit(&sc->pid, SPEED_IQ_MAX, -SPEED_IQ_MAX);

    sc->pid.Target = SPEED_TARGET_RPM;
}

#if SPEED_FRICTION_ENABLE
/* 脱开电流包络：从释放电流平滑过渡到动摩擦电流 */
static float PID_FrictionReleaseCurrent(uint16_t elapsed_ms)
{
    float ratio;

    if (elapsed_ms >= SPEED_FRICTION_RELEASE_MS)
    {
        return SPEED_FRICTION_KINETIC_A;
    }

    ratio = (float)elapsed_ms / (float)SPEED_FRICTION_RELEASE_MS;
    return SPEED_FRICTION_RELEASE_A
         + ratio * (SPEED_FRICTION_KINETIC_A - SPEED_FRICTION_RELEASE_A);
}

/** @brief 低速摩擦补偿：静止时平滑爬升；脱开后平滑释放电流。
  * @param[in] fb_rpm Hall 净位移机械转速(rpm)，由调用者传入。
  *        用它判定运动，避免单边沿瞬时测速把局部振荡误当作持续转动。 */
static float PID_FrictionFeedforward(SpeedCtrl_TypeDef *sc, float rpm_target, float fb_rpm)
{
    static uint16_t call_div;
    float sign;
    float speed_abs;

    /* 低速目标段（位置环/低速速度模式）：按"速度比例"给破静摩擦前馈。
     * 静止时 START_A=0.5A(>静摩擦)保证电机马上启动；随速度线性衰减，
     * 到目标速度归零，由 PI 积分负责稳态保持。恒定前馈在目标附近死区
     * 持续正向推，会形成过冲-刹回循环、转速大幅瞬变导致估算角跟不上
     * 而齿槽回弹，故必须衰减。目标≈0 时前馈为 0。 */
    if (fabsf(rpm_target) <= SPEED_FRICTION_CMD_RPM_MAX)
    {
        if (fabsf(rpm_target) >= SPEED_FRICTION_MIN_CMD_RPM)
        {
            float s_along = (rpm_target >= 0.0f) ? fb_rpm : -fb_rpm;
            if (s_along < 0.0f)
            {
                return 0.0f;   /* 反向: 撤前馈 */
            }
            {
                float taper = 1.0f - s_along / fabsf(rpm_target);
                if (taper < 0.0f) taper = 0.0f;
                return (rpm_target >= 0.0f)
                     ?  SPEED_FRICTION_START_A * taper
                     : -SPEED_FRICTION_START_A * taper;
            }
        }
        return 0.0f;
    }

    if (++call_div >= SPEED_LOOP_DIV)
    {
        call_div = 0U;
        if (sc->friction_ms < 0xFFFFU) sc->friction_ms++;
    }

    if (fabsf(rpm_target) < SPEED_FRICTION_MIN_CMD_RPM)
    {
        sc->friction_state = FRICTION_OFF;
        sc->friction_ms    = 0U;
        return 0.0f;
    }

    sign = (rpm_target >= 0.0f) ? 1.0f : -1.0f;
    speed_abs = sign * fb_rpm;

    /* 反向或明显过冲时取消补偿。低速环路不再主动反向制动，由PID_Speed_Loop限制Iq符号。 */
    if ((speed_abs < 0.0f) ||
        (speed_abs > fabsf(rpm_target) + SPEED_FRICTION_OVERSHOOT_RPM))
    {
        sc->friction_state = FRICTION_BRAKE;
        sc->friction_ms    = 0U;
        return 0.0f;
    }

    if (speed_abs >= SPEED_FRICTION_MOVE_RPM)
    {
        if (sc->friction_state != FRICTION_MOVING)
        {
            /* 静摩擦已脱开：保留速度环刚形成的输出，避免转矩突然掉落。 */
            sc->friction_state = FRICTION_MOVING;
            sc->friction_ms    = 0U;
        }
        return sign * PID_FrictionReleaseCurrent(sc->friction_ms);
    }

    if (speed_abs <= SPEED_FRICTION_REST_RPM)
    {
        /* 脱开后短时速度回落，仍维持释放电流；避免刚跨过阈值又重新进入静摩擦爬升。 */
        if ((sc->friction_state == FRICTION_MOVING) &&
            (sc->friction_ms < SPEED_FRICTION_RELEASE_MS))
        {
            return sign * PID_FrictionReleaseCurrent(sc->friction_ms);
        }

        if (sc->friction_state != FRICTION_REST)
        {
            sc->friction_state   = FRICTION_REST;
            sc->friction_ms      = 0U;
            /* 注意：这里不能清 Integral/Error/ErrorLast/Out —— 清零会让速度环
             * 永远建不起稳态力矩（只能靠前馈"猛冲-掉回"循环，即低速一转一停的根因）。
             * 只重置时间计数，积分由 PID_Speed_Loop 里的积分限幅管住。 */
        }

        /* REST期间前馈负责快速跨过静摩擦；PI积分不再被清零，负责稳态力矩。 */
        if (sc->friction_ms < SPEED_FRICTION_STATIC_MS)
        {
            float ratio = (sc->friction_ms >= SPEED_FRICTION_RAMP_MS)
                        ? 1.0f
                        : (float)sc->friction_ms / (float)SPEED_FRICTION_RAMP_MS;
            float mag = SPEED_FRICTION_KINETIC_A
                      + ratio * (SPEED_FRICTION_STATIC_A - SPEED_FRICTION_KINETIC_A);
            return sign * mag;
        }
        sc->friction_state = FRICTION_BRAKE;
        sc->friction_ms    = 0U;
        return 0.0f;
    }

    /* 10~20rpm迟滞区：保留当前状态，避免在阈值附近反复切换。 */
    if (sc->friction_state == FRICTION_MOVING)
    {
        return sign * PID_FrictionReleaseCurrent(sc->friction_ms);
    }
    return 0.0f;
}
#endif

/* 速度环：分频到1kHz跑PI，输出Iq经斜率限制后由调用者写入电流环 q 轴目标。
 * rpm_target 是唯一入口(速度模式传 speed_target，位置模式传位置环输出)，
 * 只读一次，死区判断与 PID_Loop 共用同一个值 */
float PID_Speed_Loop(SpeedCtrl_TypeDef *sc, float rpm_target, float mech_rpm, uint8_t from_position)
{
    float ff = 0.0f;
    float iq_raw;
    float iq_step;
    float di;

#if SPEED_FRICTION_ENABLE
    ff = PID_FrictionFeedforward(sc, rpm_target, mech_rpm);
#else
    (void)mech_rpm;
#endif

    /* 速度环：分频到1kHz，反馈用滤波后转速；前馈先占用输出额度，避免PI积分饱和 */
    if (++sc->div_cnt >= SPEED_LOOP_DIV)
    {
        sc->div_cnt = 0;

        if ((from_position == 0U) &&
            fabsf(rpm_target) < 0.5f && fabsf(sc->f_speed_fb) < SPEED_DEADBAND_RPM)
        {
            sc->pid.Integral  = 0.0f;
            sc->pid.Error     = 0.0f;
            sc->pid.ErrorLast = 0.0f;
            sc->pid.Out       = 0.0f;
            sc->last_cmd_iq   = 0.0f;
        }
        else
        {
            /* 不再按前馈占用额度去裁剪 PI 限幅：限幅一旦被裁剪，PID_Loop 的"饱和回退积分"
             * 会反复打断积分器，配合原来的积分清零就彻底没有稳态力矩了。
             * 现在 PI 用完整限幅，前馈只作为一个偏置在下面叠加。 */
            PID_Loop(&sc->pid, rpm_target, sc->f_speed_fb);

            /* 积分限幅（替代原来的"每拍清零"）：保证 Ki*Integral 不超出输出上限，
             * 既防积分饱和，又保留低速稳态力矩。 */
            if (sc->pid.Ki != 0.0f)
            {
                /* 非对称钳制：正向放开(±1.25A)保证启动/升档力矩；负向钳制到
                 * -SPEED_IQ_NEG_LIM_A(-0.50A)，抵消前馈后仍有正向保持力矩、
                 * 无转矩真空，防止低速反转/跑高停转。 */
                float i_lim = SPEED_IQ_MAX / sc->pid.Ki;
                float i_lim_neg = SPEED_IQ_NEG_LIM_A / sc->pid.Ki;
                if (sc->pid.Integral >  i_lim) sc->pid.Integral =  i_lim;
                if (sc->pid.Integral < -i_lim_neg) sc->pid.Integral = -i_lim_neg;
                /* 低速目标(≤200rpm)正积分按目标比例限幅：滤掉低速段反馈振铃对
                 * 积分的整流抬升，稳态 Iq 贴近各速摩擦需求；负向仍 -0.50A 保证刹得住。 */
                if (fabsf(rpm_target) <= SPEED_FRICTION_CMD_RPM_MAX)
                {
                    float i_lim_pos = (SPEED_IQ_POS_LIM_LOW_BASE_A
                                       + SPEED_IQ_POS_LIM_LOW_K_A * fabsf(rpm_target))
                                      / sc->pid.Ki;
                    if (sc->pid.Integral > i_lim_pos) sc->pid.Integral = i_lim_pos;
                }
                sc->pid.OutI = sc->pid.Ki * sc->pid.Integral;
            }
        }
    }

    iq_raw = sc->pid.Out + ff;
    /* 低速段的制动权限：只在"沿目标方向但还没到目标速度"时禁止反向制动
     * （别刹在下一个霍尔边界）；到达/超过目标速度、或正在反向旋转时，
     * 减速权交回速度环。 */
    if ((fabsf(rpm_target) >= SPEED_FRICTION_MIN_CMD_RPM) &&
        (fabsf(rpm_target) <= SPEED_NO_BRAKE_RPM) &&
        (iq_raw * rpm_target < 0.0f))
    {
        float speed_along_target = (rpm_target >= 0.0f)
                                 ? sc->f_speed_fb
                                 : -sc->f_speed_fb;
        float target_abs = fabsf(rpm_target);

        if ((speed_along_target >= 0.0f) &&
            (speed_along_target < target_abs) &&
            (speed_along_target < SPEED_NO_BRAKE_RPM))
        {
            /* 还没到目标速度：不给反向制动，避免刹在下一个霍尔边界。
             * 反转靠非对称积分钳制解决，不靠清零积分（清零会毁掉稳态保持力矩）。 */
            iq_raw = 0.0f;
        }
        else if (speed_along_target > target_abs + SPEED_FRICTION_OVERSHOOT_RPM)
        {
            /* PLL 判停保护：用未低通的原始 PLL 转速（边沿真相）判断——低通反馈
             * 滞后约 10~20ms，用它判"该不该刹"会把转子刹进停转点，引发齿槽/磁定位
             * 回弹。原始转速回落到目标附近即撤制动，给最小正向保持电流，无转矩阶跃。 */
            float raw_along = (rpm_target >= 0.0f) ? sc->f_speed_raw : -sc->f_speed_raw;
            if (raw_along < target_abs + SPEED_BRAKE_STOP_RPM_MARGIN)
            {
                iq_raw = (rpm_target >= 0.0f)
                       ?  SPEED_BRAKE_STOP_HOLD_A
                       : -SPEED_BRAKE_STOP_HOLD_A;
            }
            else
            {
                /* 明显超速：允许制动，但限制幅值 */
                if (rpm_target >= 0.0f)
                {
                    if (iq_raw < -SPEED_OVERSPEED_BRAKE_A) iq_raw = -SPEED_OVERSPEED_BRAKE_A;
                }
                else if (iq_raw > SPEED_OVERSPEED_BRAKE_A)
                {
                    iq_raw = SPEED_OVERSPEED_BRAKE_A;
                }
            }
        }
    }
    if (iq_raw >  SPEED_IQ_MAX) iq_raw =  SPEED_IQ_MAX;
    if (iq_raw < -SPEED_IQ_MAX) iq_raw = -SPEED_IQ_MAX;

    /* 位置模式破静摩擦启动补偿（仅位置模式）：指令较大且转子静止时，
     * 把 Iq 抬底到 POS_BOOST_A 破障，转速起来后撤除交回速度环。
     * 静止/启动判定用低通反馈 f_speed_fb（净位移测速在齿槽峰临界微动时
     * 会击穿静止窗口导致补偿反复撤除、力矩不足）。 */
    if ((from_position != 0U) &&
        (fabsf(rpm_target) >= POS_BOOST_CMD_RPM) &&
        (sc->f_speed_fb > -POS_BOOST_RPM_IN) && (sc->f_speed_fb < POS_BOOST_RPM_IN))
    {
        if (rpm_target >= 0.0f)
        {
            if (iq_raw < POS_BOOST_A) iq_raw = POS_BOOST_A;
        }
        else
        {
            if (iq_raw > -POS_BOOST_A) iq_raw = -POS_BOOST_A;
        }
    }

    /* Iq指令斜率限制：本函数每个 FOC 节拍(50us)都会被调用一次，所以步长必须按 ts 算。 */
    iq_step = SPEED_IQ_SLEW_A_S * ts;
    di = iq_raw - sc->last_cmd_iq;
    if (di >  iq_step) di =  iq_step;
    if (di < -iq_step) di = -iq_step;
    sc->last_cmd_iq += di;

    return sc->last_cmd_iq;
}

/*=========================== 位置环 ==============================*/
/**
 * @brief 位置环参数初始化
 * @note 位置环周期 = POS_LOOP_DIV * ts = 5ms（200Hz），输出为速度环目标(rpm)
 */
void PID_Position_Init(PositionCtrl_TypeDef *pc)
{
    /* 位置环状态清零初始化 */
    PID_Init(&pc->pid);

    /* 位置环参数设置：输入单位rad，输出单位rpm */
    PID_SetParm(&pc->pid,
                (float)POS_LOOP_DIV * ts,
                POS_KP, POS_KI, POS_KD);

    /* 位置环输出限幅：限制速度环目标(rpm) */
    PID_SetLimit(&pc->pid, POS_RPM_MAX, -POS_RPM_MAX);

    /* 位置环初始值设置 */
    pc->pid.Target = POS_TARGET_RAD;
    pc->div_cnt    = 0;
}

/**
 * @brief 位置环运行：分频到 200Hz 跑一次 PI，输出速度指令存入 pc->pid.Out
 * @param[in,out] pc   位置环状态，pc->pid.Target 为目标机械角(rad)
 * @param[in]     mech 当前机械角反馈(rad)，连续多圈、与目标同一坐标
 */
void PID_Position_Loop(PositionCtrl_TypeDef *pc, float mech)
{
    /* 位置环分频：20kHz / POS_LOOP_DIV = 200Hz */
    if (++pc->div_cnt < POS_LOOP_DIV) return;
    pc->div_cnt = 0;

    if ((POS_DEADBAND_RAD > 0.0f) &&
        fabsf(pc->pid.Target - mech) < POS_DEADBAND_RAD)
    {
        /* 进入死区：不再继续扩大误差，但保留已有积分和保持输出，
         * 避免每次进入死区都把保持力矩清零。 */
        PID_Loop(&pc->pid, mech, mech);
    }
    else
    {
        /* 位置误差(rad) -> 速度指令(rpm)，结果存入pc->pid.Out */
        PID_Loop(&pc->pid, pc->pid.Target, mech);
    }
}

/*=========================== 环状态复位 ==============================*/
/**
 * @brief 复位位置环状态：清积分、历史误差与输出，避免带旧转速指令冲击速度环
 * @note  不清分频计数，保证目标突变后位置环下一拍立刻按新目标计算
 */
void PID_PositionState_Reset(PositionCtrl_TypeDef *pc)
{
    pc->pid.Integral  = 0.0f;
    pc->pid.Error     = 0.0f;
    pc->pid.ErrorLast = 0.0f;
    pc->pid.Out       = 0.0f;
}

/**
 * @brief 复位速度环与低速摩擦前馈状态
 * @note  清速度环积分、分频计数、Iq斜率缓存，并把摩擦前馈状态机归零；
 *        切换运行模式时调用，避免上一模式的残留状态造成冲击。
 *        电流环 q 轴目标由调用者负责归零（本模块不认识电流环的持有者）。
 */
void PID_SpeedState_Reset(SpeedCtrl_TypeDef *sc)
{
    /* 复位速度环积分与中间量，防止旧积分残留 */
    sc->pid.Integral  = 0.0f;
    sc->pid.Error     = 0.0f;
    sc->pid.ErrorLast = 0.0f;
    sc->pid.Out       = 0.0f;

    sc->div_cnt        = 0;
    sc->last_cmd_iq    = 0.0f;
    sc->friction_state = FRICTION_OFF;
    sc->friction_ms    = 0U;
}
