/*==============================================================================
 * 文件: Users/Src/StateMachine.c
 * 职责: 唯一的电机状态机调度层 —— 只做"初始化 / 状态判断 / 按状态运行"。
 *
 * 分层约定(本工程功能逻辑的切分方式):
 *   Algorithm/Src/PID.c      : 三个环(电流/速度/位置)的算法、参数与内部状态量
 *   BSP/Src/bsp_encoder.c    : 霍尔测速、电角度解算、上电零点标定、方向自检
 *   BSP/Src/bsp_adc.c        : 三相电流采样与零点校准
 *   APP/Src/VOFA.c           : 上位机显示与在线命令
 *   Users/Src/StateMachine.c : 本文件 —— 只负责"什么时候调用谁"
 *
 * 中断节拍(20kHz, 见 Core/Src/stm32f4xx_it.c 的注入转换完成回调):
 *   ADC采样 -> 电流低通 -> Motor_StateMachine_Run()
 *     |- Clarke -> Park -> 电流环(PID.c)
 *     |- switch(状态): ALIGN / ALIGN2 / CALIB / DIRCHK / RUN
 *     |    RUN 内再按 run_mode 分派: 转矩 / 速度 / 位置
 *     |- SVPWM 调制
 *     '- 写 TIM8 CCR
 *============================================================================*/
#include "StateMachine.h"
MotorState_TypeDef MotorBLDC;

/* 运行时目标转速(rpm)，初值取宏，可被VOFA命令覆盖 */
float speed_target = SPEED_TARGET_RPM;

/* 位置目标是否已被上位机命令显式设定过。0 = 还没设过 → 进 RUN 时把当前转子位置
 * 当作目标（上电零点），避免用初值造成"上电就蹿一下"。 */
static volatile uint8_t s_pos_target_cmd = 0;
static uint8_t s_prev_in_pos = 0U;   /* 上一拍是否处于位置模式(供 State_Run 清除) */

/* 转矩模式运行目标：初值取配置宏，可由VOFA命令在线修改 */
static float s_torque_id_target = CURRENT_ID_TARGET;
static float s_torque_iq_target = CURRENT_IQ_TARGET;

/* VOFA命令接收初始化（定义在VOFA.c），避免头文件循环包含 */
extern void VOFA_CommandInit(void);

/* 本文件只做三件事：初始化、状态判断、状态运行。
 * 三个环的算法与状态都在 Algorithm/Src/PID.c，
 * 零位标定/方向自检的算法与状态都在 BSP/Src/bsp_encoder.c，
 * 这里只按状态机节拍依次调用它们，并从 Hall 取出反馈喂给对应的环。 */

void Motor_StateMachine_Init(MotorState_TypeDef * state)
{   
    /* 硬件初始化：PWM 使能 + TIM8 中断 */
    Motor_BspInit();
    /* 设置母线电压与 PWM 的 ARR */
    SetParm_Udc_ARR(&state->FOCParm,_Udc,_ARR);
    /* VF 开环强拖初始化（本工程 acc/target_speed 均为 0，未启用） */
    VFControl_Init(&state->VFParm,acc,target_speed,dead_zone,ts);
    /* 霍尔测速 + 电角度估算初始化：配置 TIM5 时间基准、启动霍尔接口中断 */
    HALL_Speed_Init(&state->HALLParm);
    /* PLL 初始化：本工程只当转速观测器用，不参与 FOC 角度 */
    PLL_SetParm(&state->PLLParm,ts,kp_theta,ki_theta);
    PLL_Init(&state->PLLParm);
    /* ADC 注入采样初始化：采样电阻、运放增益、参考电压 */
    ADC_SampInit(&state->ADCSamp, &state->FOCParm.Sector, rs , gain, vref);
    /* ADC 零点校准：电机静止时采 1000 次取平均做零漂基准 */
    ADC_ZeroCalibrate_Run(&state->ADCSamp);
    /* 滤波器初始化：转速与三相电流的一阶低通系数 */
    Filtering_Init();
    /* 电流环 PI 初始化：d/q 轴参数与输出限幅 */
    PI_Current_Init(&state->current);
    /* 速度环 PI 初始化：周期 = SPEED_LOOP_DIV / 20kHz = 1kHz */
    PID_Speed_Init(&state->speed);
    /* 位置环 PI 初始化：周期 = POS_LOOP_DIV / 20kHz = 200Hz */
    PID_Position_Init(&state->position);
    /* 启动 VOFA 命令接收（RX 空闲中断），用于运行时修改目标转速 */
    VOFA_CommandInit();
    /* 预定位初始化：进入 ALIGN 状态，把转子拉到已知电角度 */
    Motor_AlignState_Init(state);
    /* 运行模式选择：默认位置模式 —— 上电/复位即进位置环，切入时自动锚定当前
     * 转子位置并清零位置环状态，多圈机械角从上电零位起计。 */
    Motor_RunMode_Set(&MotorBLDC,MOTOR_RUN_POSITION);

}

/*==================== 状态/模式 内部函数声明 ====================*/
static void State_Align   (MotorState_TypeDef * state);
static void State_Align2  (MotorState_TypeDef * state);
static void State_Calib   (MotorState_TypeDef * state);
static void State_DirCheck(MotorState_TypeDef * state);
static void State_Run     (MotorState_TypeDef * state);
static void Run_UpdateObserver(MotorState_TypeDef * state);
static void Run_TorqueMode (MotorState_TypeDef * state);
static void Run_SpeedMode  (MotorState_TypeDef * state);
static void Run_PositionMode(MotorState_TypeDef * state);
static void Deadtime_Compensate(FOCParm_TypeDef * foc);
static float AngleAssist_Creep(float iq_cmd, float mech_rpm);


void Motor_StateMachine_Run(MotorState_TypeDef * state)
{
    /*对采集三相电流进行Clarke Park坐标变换*/
    Clarke_Transf(&state->FOCParm);
    Park_Transf(&state->FOCParm);
    /*执行电流环*/
    PI_Current_Loop(&state->current, &state->FOCParm);
    /*死区电压补偿：PI输出Vd/Vq之后、SVPWM之前，按相电流符号补偿逆变器死区
     * 造成的平均电压损失（见 Motor_Config.h「死区电压补偿」） */
    Deadtime_Compensate(&state->FOCParm);
    /*电机状态机*/
    switch (state->state)
    {
        case MOTOR_STATE_ALIGN:    State_Align(state);   break;
        case MOTOR_STATE_ALIGN2:   State_Align2(state);  break;
        case MOTOR_STATE_CALIB:    State_Calib(state);   break;
        case MOTOR_STATE_DIRCHK:   State_DirCheck(state);break;
        case MOTOR_STATE_RUN:      State_Run(state);     break;
        default:                   break;
    }
    /*执行SVPWM调制并实际输出CCR*/
    FOC_Algorithm_SVPWM(&state->FOCParm);
    Motor_SetPWMCompare(state->FOCParm.CCR_U,
                        state->FOCParm.CCR_V,
                        state->FOCParm.CCR_W);
}

/* 运行函数：更新观测器后按当前运行模式分派到 PID.c 里的控制律 */
static void State_Run(MotorState_TypeDef * state)
{
    Run_UpdateObserver(state);

    if (state->run_mode == MOTOR_RUN_POSITION)
    {
        Run_PositionMode(state);              /* 切入重置由函数内 s_prev_in_pos 沿检测处理 */
    }
    else
    {
        s_prev_in_pos = 0U;                   /* 离开位置模式，下次切入重新初始化 */
        HALL_SetDebounceTicks(HALL_EDGE_DEBOUNCE_TICKS);  /* 恢复默认去抖窗(100us，速度/扭矩模式高速运行) */
        /* 清位置模式蠕动残留（s_force_creep / s_creep_omega）：残留会让 Hall 角度外推
         * 每拍进蠕动分支、扇区钳位被豁免，与边沿 snap 互相拉扯导致高速转速抖动。
         * 与位置模式每拍强制设置对称，这里每拍强制清，杜绝残留劫持内环。 */
        HALL_SetForceCreep(0U);
        HALL_SetCreepOmega(0.0f);
        switch (state->run_mode)
        {
            case MOTOR_RUN_TORQUE:    Run_TorqueMode(state);    break;
            case MOTOR_RUN_SPEED:     Run_SpeedMode(state);     break;
            default:                  Run_SpeedMode(state);     break;
        }
    }
}

/*==================== 状态函数 ====================*/
/* 第一次对齐：固定0度强拉 */
static void State_Align(MotorState_TypeDef * state)
{
    state->FOCParm.Theta = ALIGN_ANGLE;
    if (--state->align_cnt == 0)
    {
        HALL_CalibAlign1();                 /* 记录第一次霍尔角 */
        state->state = MOTOR_STATE_ALIGN2;
        state->align_cnt = (uint32_t)(ALIGN_TIME_MS * Tim_Hz / 1000);
    }
}

/* 第二次对齐：固定180度强拉，结束后进入零位细分标定 */
static void State_Align2(MotorState_TypeDef * state)
{
    state->FOCParm.Theta = ALIGN2_ANGLE;
    if (--state->align_cnt == 0)
    {
        HALL_CalibAlign2();                 /* 记录第二次并算出兜底零位 */
        HALL_CalibRampBegin(state->FOCParm.Theta);   /* 从当前实际施加的电角度开始拖动 */
        state->state     = MOTOR_STATE_CALIB;
        state->align_cnt = 0;
    }
}

/* 零位细分标定：用 Id 把转子粘在 d 轴上缓慢正转，每跨过一个霍尔边沿就由当前施加
 * 电角度反算零位 h0 与符号 σ，精度从兜底的 ±30° 提升到几度。 */
static void State_Calib(MotorState_TypeDef * state)
{
    float theta_cmd;

    if (HALL_CalibRampRun(&theta_cmd) != 0U)
    {
        /* 拖动中：开环施加电角度，Id 由电流环给定 */
        state->FOCParm.Theta = theta_cmd;
    }
    else
    {
        HALL_CalibFinish();                 /* 锁定零位/符号并复位估算器 */
        HALL_DirCheckBegin();               /* 接着做一次力矩方向自检（判 π） */
        state->state = MOTOR_STATE_DIRCHK;
    }
}

/* 力矩方向自检：保持施加电角度不动只给小的 +Iq，看转子往哪边动；往 -Theta 动说明
 * 零位差 π（拖动标定的固有二义性），补 π。不做这步，转子停在反相位置时闭环极性
 * 会反过来，一给正指令就是正反馈 —— 表现就是"上电全速飞车"。 */
static void State_DirCheck(MotorState_TypeDef * state)
{
    float theta_cmd;

    /* 自检电流：Id=0、Iq=+HALL_DIRCHK_IQ */
    state->current.d.Target = 0.0f;
    state->current.q.Target = HALL_DIRCHK_IQ;

    if (HALL_DirCheckRun(&theta_cmd) != 0U)
    {
        state->FOCParm.Theta = theta_cmd;   /* 角度保持不动 */
    }
    else
    {
        float omega0;

        HALL_DirCheckFinish();              /* 极性反了就补 π */
        state->FOCParm.Theta = HALL_GetElecAngle(&state->HALLParm);

        /* PLL 只作为转速观测器（FOC 角度不再用它的输出）。用当前角度和估算电角速度
         * 把 PLL 状态对齐，避免进 RUN 时速度反馈从 0 跳起、速度环猛给 Iq 造成冲击。 */
        omega0 = HALL_GetElecOmega();
        state->PLLParm.OutRe = state->FOCParm.Theta;
        state->PLLParm.Yo    = state->FOCParm.Theta;
        state->PLLParm.OutWe = omega0;
        state->PLLParm.Xo    = omega0;
        state->PLLParm.i     = 0.0f;
        state->PLLParm.Error = 0.0f;

        /* 位置模式：把"进 RUN 这一刻的转子位置"记为目标（上电零点）。
         * 标定/方向自检过程中转子已经被拖动过，若还沿用上电前的 0，位置环一开始
         * 就带着几十度的误差、有了力矩权限后会猛地纠正一下。上位机若已下过位置
         * 命令则以命令为准（s_pos_target_cmd）。 */
        if (state->run_mode == MOTOR_RUN_POSITION && s_pos_target_cmd == 0U)
        {
            state->position.pos_target_cmd    = Motor_GetPosFB();   /* 单圈绝对角 [0,2π) */
            state->position.pid.Target        = state->position.pos_target_cmd;
            state->position.pid.Integral      = 0.0f;
            state->position.pid.Error         = 0.0f;
            state->position.pid.ErrorLast     = 0.0f;
            state->position.pid.Out           = 0.0f;
        }

        /* 进入RUN；清零内环目标与斜率缓存（运行模式由 Motor_RunMode_Set 决定，不在这里覆盖） */
        state->state            = MOTOR_STATE_RUN;
        state->current.d.Target = CURRENT_ID_TARGET;
        state->current.q.Target = CURRENT_IQ_TARGET;
        state->speed.div_cnt    = 0;
        state->speed.last_cmd_iq = 0.0f;
    }
}


/*==================== 低速角度助推判断 ====================*/
/* 死区带线性符号函数：|v|<=band 时按比例过渡，避免零电流附近补偿量突变造成正反馈抖动 */
static float lin_sign(float v, float band)
{
    if (v >  band) return  1.0f;
    if (v < -band) return -1.0f;
    return v / band;
}

/* 死区电压补偿：把三相电流符号投影到 dq 轴，补偿逆变器死区+管压降的电压损失。
 * 必须在 PI_Current_Loop 之后、FOC_Algorithm_SVPWM 之前调用。
 * 符号/幅值标定：见 Motor_Config.h「死区电压补偿」的说明。 */
static void Deadtime_Compensate(FOCParm_TypeDef *foc)
{
#if DEADTIME_COMP_ENABLE
    float cu = lin_sign(foc->Iu, DEAD_IZERO_A);
    float cv = lin_sign(foc->Iv, DEAD_IZERO_A);
    float cw = lin_sign(foc->Iw, DEAD_IZERO_A);
    float t  = foc->Theta;
    float s, c;
    float cm, sm, hc, hs;
    float cd, cq;
    const float SQRT3_2 = 0.86602540f;      /* √3/2 */
    FOC_SinCos(t, &s, &c);
    /* 相符号 → dq（等幅值 Clarke+Park，2/3 系数；0.5 为标定系数）。
     * 原式对 t、t∓120° 共调用 6 次 cosf/sinf；改用角度和公式把 ±120° 展开，
     * 每个周期只算一次 sinf/cosf(t)，数学上完全等价：
     *   cos(t∓120°) = -0.5c ± (√3/2)s        sin(t∓120°) = -0.5s ∓ (√3/2)c */
    cm = -0.5f * c;                         /* cos(t±120°) 的公共项 */
    sm = -0.5f * s;                         /* sin(t±120°) 的公共项 */
    hc = SQRT3_2 * c;
    hs = SQRT3_2 * s;
    cd = (2.0f/3.0f) * (cu*c + cv*(cm + hs) + cw*(cm - hs));
    cq = (2.0f/3.0f) * (cu*s + cv*(sm - hc) + cw*(sm + hc));

    foc->Vd += DEADTIME_COMP_SIGN * DEAD_TIME_V * cd * 0.5f;
    foc->Vq += DEADTIME_COMP_SIGN * DEAD_TIME_V * cq * 0.5f;
#else
    (void)foc;
#endif
}

/*==================== 低速角度助推判断 ====================*/
/* 返回本拍要写入 Hall 的助推电角速度(rad/s 电角度)，0 = 不助推。
 * 只在命令了电流(|Iq|>ANGLE_ASSIST_IQ_MIN)且实际转速很低时助推：
 *  - 位置模式绝不助推：位置环的保持力矩需要角度稳定，被拖走就变成自己往外跑；
 *  - 速度模式禁止助推：启动瞬间转子"还没动"≠堵转，空转电角度反而阻止启动；
 *  - 转速已经起来(>EXIT)：霍尔边沿正常，交回闭环，带 ENTER/EXIT 迟滞防抖。 */
static float AngleAssist_Creep(float iq_cmd, float mech_rpm)
{
#if ANGLE_ASSIST_ENABLE
    static uint8_t on = 0U;
    float mag  = (iq_cmd   >= 0.0f) ? iq_cmd   : -iq_cmd;
    float rabs = (mech_rpm >= 0.0f) ? mech_rpm : -mech_rpm;
    float sign;

    if (MotorBLDC.run_mode == MOTOR_RUN_POSITION) { on = 0U; return 0.0f; }
    if (mag <= ANGLE_ASSIST_IQ_MIN)               { on = 0U; return 0.0f; }

    /* 速度模式不在此处提前退出：低速段由下方 RPM 迟滞(25/45rpm)与 Iq 门限决定助推开关。 */

    if ((MotorBLDC.run_mode == MOTOR_RUN_SPEED) &&
        (fabsf(speed_target) < SPEED_FRICTION_MIN_CMD_RPM))
    {
        on = 0U;
        return 0.0f;
    }

    if (on != 0U)
    {
        if (rabs > ANGLE_ASSIST_RPM_EXIT)  on = 0U;
    }
    else
    {
        if (rabs < ANGLE_ASSIST_RPM_ENTER) on = 1U;
    }
    if (on == 0U) return 0.0f;

    sign = (iq_cmd >= 0.0f) ? 1.0f : -1.0f;
    return sign * ANGLE_ASSIST_W_CREEP;
#else
    (void)iq_cmd; (void)mech_rpm;
    return 0.0f;
#endif
}

/*==================== 更新当前实时角度和速度 ====================*/
static void Run_UpdateObserver(MotorState_TypeDef * state)
{
    float rpm_raw;

    /* 低速角度助推请求：必须在取电角度之前写入（见 Motor_Config.h「4b 低速角度助推」）。
     * 用上一拍算出的 Iq 指令判断：本拍的模式分派还没跑，滞后一拍 = 50us，无影响。 */
    HALL_SetCreepOmega(AngleAssist_Creep(state->current.q.Target, HALL_GetMechSpeedRpm()));

    /* Hall 角度必须先更新；Hall 模块在同一节拍内同步更新低速净位移测速与速度融合 */
    state->FOCParm.Theta = HALL_GetElecAngle(&state->HALLParm);

    rpm_raw = HALL_GetSpeedRpm();

#if SPEED_OBS_PLL_ENABLE
    (void)rpm_raw;   /* PLL 路径不使用 Hall 融合转速 */
    /* PLL 转速观测器：跟踪连续机械角，输出连续平滑的机械角速度，
     * 无固定窗口测速的阶梯/延迟，直接作速度环 PI 反馈。 */
    PLL_Loop(&state->PLLParm, HALL_GetMechAngle());
    /* PLL 输出接一阶低通，滤掉霍尔换相"角度台阶"产生的转速尖峰（速度环不再
     * 对台阶过激响应）；低通前先做每帧速率钳位 ±300rpm，只钳观测器尖峰，
     * 中高速真实动态（每帧变化物理上限约 100rpm）完全不受影响。 */
    {
        float rpm_pll = RPM_FROM_WM(state->PLLParm.OutWe);
        float d = rpm_pll - state->speed.f_speed_raw_prev;
        if      (d >  300.0f) rpm_pll = state->speed.f_speed_raw_prev + 300.0f;
        else if (d < -300.0f) rpm_pll = state->speed.f_speed_raw_prev - 300.0f;
        state->speed.f_speed_raw_prev = rpm_pll;
        state->speed.f_speed_fb = filter_lag(&state->FilterMethod.f_pll_lag, rpm_pll);
        state->speed.f_speed_raw = rpm_pll; /* 原始 PLL 转速(已钳位) */
    }
#else
    /* Hall 层已完成高低速融合；此处只做统一限幅和低通，供速度环与位置环使用 */
    if (rpm_raw >  SPEED_FB_RPM_MAX) rpm_raw =  SPEED_FB_RPM_MAX;
    if (rpm_raw < -SPEED_FB_RPM_MAX) rpm_raw = -SPEED_FB_RPM_MAX;
    state->speed.f_speed_fb = filter_lag(&state->FilterMethod.f_rpm_fb_lag, rpm_raw);
    state->speed.f_speed_raw = rpm_raw; /* 非 PLL 路径: 原始融合转速 */
#endif
}

/*==================== 对外公开接口（VOFA 上位机读写） ====================*/
/* 读取速度环实际使用的滤波后转速(rpm)，供上位机显示 */
float Motor_GetSpeedFB(void)
{
    return MotorBLDC.speed.f_speed_fb;
}

/* 读取位置模式实际使用的机械角反馈(rad)：绝对式单圈 [0,2π)，供上位机直接显示 0~360°。
 * 原始累计机械角(HALL_GetMechAngle)仍保留给 PLL 测速/堵转位移判断使用 */
float Motor_GetPosFB(void)
{
    float a = fmodf(HALL_GetMechAngle(), _2PI);
    if (a < 0.0f) a += _2PI;
    return a;
}

/* 读取位置目标机械角(度)，返回的是上位机下发的原始命令值，供显示/对比 */
float Motor_GetPosTarget(void)
{
    return RAD2DEG(MotorBLDC.position.pos_target_cmd);
}

/* 读取当前运行模式(0=转矩 1=速度 2=位置) */
MotorRunModeEnum Motor_GetRunMode(void)
{
    return MotorBLDC.run_mode;
}

/* 设置位置模式目标机械角。!! 入参单位是"度"(°) !!，内部换算成 rad 并归一化到 [0,2π)（绝对式单圈）。
 * 基准是上电标定结束时的转子位置(HALL_GetMechAngle()==0)，正值=正转方向；
 * 目标在 0~360° 之外时自动回绕到该区间（如 -10°→350°、370°→10°）。
 * 目标没变（上位机周期性下发同一个目标）时不动环内状态：否则每个命令周期都把位置积分
 * 清一次，稳态误差永远消不掉，轴会软绵绵地停在偏差上。
 * 目标突变时清位置环状态：若留着上一个目标的转速指令(position.pid.Out)，速度环会先按
 * 旧指令跑最长一个位置环周期(5ms)才纠正 */
void Motor_PosTarget_Set(float deg)
{
    float rad = fmodf(DEG2RAD(deg), _2PI);   /* 度 → rad，归一化到 [0,2π) 绝对式单圈 */
    if (rad < 0.0f) rad += _2PI;

    s_pos_target_cmd = 1U;      /* 上位机显式下过目标，进 RUN 时不再用当前位置覆盖 */

    if (fabsf(rad - MotorBLDC.position.pos_target_cmd) < 0.001f) return;   /* 目标未变，别动积分 */

    MotorBLDC.position.pos_target_cmd = rad;    /* 内部一律存 rad */
    MotorBLDC.position.pid.Target     = rad;    /* 下一拍 Run_PositionMode 会按最短路径重算 */

    PID_PositionState_Reset(&MotorBLDC.position);
}

/*==================== 转矩模式 ====================*/
static float clamp_current(float value)
{
    if (value > TORQUE_CURRENT_MAX)  return TORQUE_CURRENT_MAX;
    if (value < -TORQUE_CURRENT_MAX) return -TORQUE_CURRENT_MAX;
    return value;
}

void Motor_TorqueIdTarget_Set(float id_a)
{
    s_torque_id_target = clamp_current(id_a);
}

void Motor_TorqueIqTarget_Set(float iq_a)
{
    s_torque_iq_target = clamp_current(iq_a);
}

/* 转矩模式：直接给定目标Id/Iq，不跑速度环。
 * 静摩擦启动补偿（kick-boost）：堵转(|rpm|<IN)且目标|iq|<FLOOR 时，实际 iq 抬到
 * FLOOR 冲开静摩擦；转子启动(|rpm|>OUT)后回到用户目标电流维持。迟滞防抖，仅
 * 扭矩模式生效；电流环在模式分派之前执行，本拍改下一拍生效(50us)。 */
static void Run_TorqueMode(MotorState_TypeDef * state)
{
    static uint8_t s_boost = 0U;               /* 1=启动补偿中 */
    float rpm = HALL_GetMechSpeedRpm();
    float iq  = s_torque_iq_target;
    float mag = (iq >= 0.0f) ? iq : -iq;
    float iq_eff = iq;

    if (s_boost != 0U)
    {
        if ((rpm >  CURR_BOOST_RPM_OUT) || (rpm < -CURR_BOOST_RPM_OUT))
        {
            s_boost = 0U;                      /* 已启动：撤补偿回用户目标 */
        }
    }
    else
    {
        if ((rpm > -CURR_BOOST_RPM_IN) && (rpm < CURR_BOOST_RPM_IN))
        {
            if (mag < CURR_BOOST_FLOOR_A)
            {
                s_boost = 1U;                  /* 静止且目标<静摩擦：进补偿 */
            }
        }
    }

    if (s_boost != 0U)
    {
        if (mag < CURR_BOOST_FLOOR_A)
        {
            iq_eff = (iq >= 0.0f) ?  CURR_BOOST_FLOOR_A : -CURR_BOOST_FLOOR_A;
        }
    }

    state->current.d.Target = s_torque_id_target;
    state->current.q.Target = clamp_current(iq_eff);
}

/*==================== 速度/位置模式入口 ====================*/
/* 速度模式：目标转速取自全局 speed_target，机械转速取自 Hall，其余全交给 PID.c 的速度环 */
static void Run_SpeedMode(MotorState_TypeDef * state)
{
    /* 速度环返回的 Iq 指令直接写进电流环 q 轴目标（from_position=0：允许静止切断输出） */
    state->current.q.Target = PID_Speed_Loop(&state->speed,
                                             speed_target,
                                             HALL_GetMechSpeedRpm(),
                                             0U);
}

/* 位置模式：位置环(200Hz)输出速度指令(rpm)交给共用速度环 → 电流环(20kHz)。
 * 位置环反馈/目标均为"绝对式单圈 [0,2π)"（0~360°），误差走最短路径，
 * 不使用 HALL 累计机械角（可到几万度），避免目标 60° 对反馈几万度产生
 * 巨大误差 → 位置环长时间饱和乱推。 */
static void Run_PositionMode(MotorState_TypeDef * state)
{
    static float   s_pos_fb_lp   = 0.0f;
    static uint8_t s_pos_fb_init = 0U;
    static float   s_pos_out_prev = 0.0f;   /* 输出斜坡上次值(函数级，供切入重置) */
    float mech_full = HALL_GetMechAngle();  /* 连续累计机械角（可正可负）：仅供低通增量/堵转
                                             * 位移判断用（连续量不跨 360° 边界，判断最稳），
                                             * 不作为位置环反馈。PLL 测速也用它。 */
    float mech      = Motor_GetPosFB();     /* 单圈绝对角 [0,2π)：位置环反馈（绝对式） */
    float tgt       = state->position.pos_target_cmd; /* 目标绝对角 [0,2π)，由 Motor_PosTarget_Set 归一化 */
    float err;
    float dmech;

    /* 切入位置模式（从扭矩/速度切来，或烧录后首次运行）：清 RAM/上次运行残留。
     * ST-LINK 烧录后的复位不清 SRAM，static 全保留：位置反馈低通、输出斜坡、
     * PID 积分、速度观测器残留假速度，会让位置环启动瞬间沿错误方向猛冲。
     * 这里：低通对齐当前真值、斜坡/PID/速度观测器全部归零，从干净状态重新起环。 */
    if (s_prev_in_pos == 0U)
    {
        s_pos_fb_lp   = mech;                 /* 丢弃残留低通，对齐当前真实位置 */
        s_pos_fb_init = 1U;
        s_pos_out_prev = 0.0f;                /* 输出斜坡从 0 重新爬升 */
        state->position.pid.Target     = mech;
        state->position.pid.Out        = 0.0f;
        state->position.pid.Integral   = 0.0f;
        state->position.pid.Error      = 0.0f;
        state->position.pid.ErrorLast  = 0.0f;
        state->position.pid.OutI       = 0.0f;
        HALL_ResetSpeedObserver();            /* 清速度观测器残留假速度 */
        HALL_SetDebounceTicks(1000U);         /* 位置模式去抖窗 5ms：堵转大电流 EMI 假边沿间隔多在 1~5ms，真边沿(40rpm≈62ms)不误伤 */
        s_prev_in_pos = 1U;
    }

    /* 位置反馈低通（增量式，跨 360° 边界无尖峰）：霍尔只有 60°电=15°机械分辨率，
     * 换相瞬间反馈跳 15°，位置环 KP 足够克服静摩擦时误差跳 15° → 输出饱和猛冲 →
     * 过冲 → 反向饱和，形成极限环振荡。低通后位置环看到平滑反馈，高 KP 才能用。 */
    if (s_pos_fb_init == 0U)
    {
        s_pos_fb_lp  = mech;
        s_pos_fb_init = 1U;
    }
    dmech = mech - s_pos_fb_lp;
    dmech = fmodf(dmech + _PI, _2PI);
    if (dmech < 0.0f) dmech += _2PI;
    dmech -= _PI;

    /* 大跳变处理分两种（堵转大电流 EMI 会让位置反馈一跳 60°=4 个霍尔扇区）：
     * ① 跳变伴随真实运动（|速度|>30rpm，如手动转动/蠕动拖转）→ 真跳变，低通直接对齐；
     * ② 跳变但电机实际没动（|速度|<=30rpm）→ EMI 假跳变，低通保持旧值拒绝污染。 */
    {
        float _spd = fabsf(HALL_GetMechSpeedRpm());
        if ((fabsf(dmech) > 0.5236f) && (_spd > 30.0f))   /* 30° 且确有运动 */
        {
            s_pos_fb_lp = mech;
        }
        else if (fabsf(dmech) > 0.5236f)                  /* 30° 但电机没动 = 假跳变 */
        {
            /* 拒绝：不跟随，位置环按旧低通闭环；但 mech 本身也不可全信，
             * 只更新 err 来源时用旧值更稳 —— 保持 s_pos_fb_lp 不变即可 */
        }
        else
        {
            /* 低通增量滤波；s_pos_fb_lp 保持单圈绝对式，滤波后回绕到 [0,2π)，
             * 跨 360° 边界（359°→1°）不产生累计漂移 */
            s_pos_fb_lp += POS_FB_LP_ALPHA * dmech;
            s_pos_fb_lp = fmodf(s_pos_fb_lp, _2PI);
            if (s_pos_fb_lp < 0.0f) s_pos_fb_lp += _2PI;
        }
    }

    /* 绝对式单圈最短路径误差：目标/低通反馈均 [0,2π)，误差回绕到 [-π,π]。
     * 跨 360° 边界走最短路径（如 350°→10° 误差 -20°，不会反向转 340°）。 */
    err = tgt - s_pos_fb_lp;
    err = fmodf(err + _PI, _2PI);
    if (err < 0.0f) err += _2PI;
    err -= _PI;

    /* pid.Target 是与反馈同一圈的工作值（PID.h 约定），保证 PID_Loop 内部
     * 误差 = Target - Feedback = err（最短路径），积分方向正确 */
    state->position.pid.Target = s_pos_fb_lp + err;

    /* 位置环 200Hz → position.pid.Out（机械角反馈为单圈绝对角，低通后入环） */
    PID_Position_Loop(&state->position, s_pos_fb_lp);

    /* 位置环输出斜坡限制：KP 较大时任何非零误差都会把输出打到 ±120rpm 饱和，
     * 速度环饱和 → 电机加速失控超速 → 过冲 → 反向饱和 → 目标附近 bang-bang 极限环。
     * 每 5ms 拍限最大变化率后输出平滑爬升/回落，速度环始终可跟踪，无过冲无振荡。
     * 前置方向自检：烧录复位不清 SRAM，若上次运行结束于位置模式，模式切换沿检测
     * 会被残留标志跳过。这里：输出与误差方向相反且 |err|>20° → 判定残留异常，
     * 斜坡立即归零，从正确方向重新爬升（正常超调时 err 与输出同号，不受影响）。 */
    {
        float out = state->position.pid.Out;
        if ((err * out) < 0.0f && fabsf(err) > 0.3491f)  /* 20° */
        {
            s_pos_out_prev = 0.0f;
            state->position.pid.Out = 0.0f;
            out = 0.0f;
        }
        float dout = out - s_pos_out_prev;
        if (dout >  POS_OUT_RATE_RPM) out = s_pos_out_prev + POS_OUT_RATE_RPM;
        if (dout < -POS_OUT_RATE_RPM) out = s_pos_out_prev - POS_OUT_RATE_RPM;
        state->position.pid.Out = out;
        s_pos_out_prev = out;
    }

    /* ===== 堵转检测 + 蠕动（破霍尔边界力矩盲区）=====
     * 霍尔只有 60°电=15°机械分辨率，电机恰好停在扇区边界时 FOC 电角度有 ≤60° 歧义，
     * 力矩方向偏 → 推不动。解法：堵转检测（误差>15° + 位置反馈近300ms变化<3°）
     * → 强制电角度开环蠕动（方向由误差符号引导朝目标拖，见下方每拍设置）→ 越过
     * 边界后霍尔边沿对齐真值 → 位置环恢复推动 → 误差<10° 退出。 */
    {
        static float    s_stall_last = 0.0f;
        static uint16_t s_stall_cnt  = 0U;
        static uint8_t  s_creeping   = 0U;
        static uint16_t s_creep_t    = 0U;

        if ((fabsf(err) > 0.2618f) &&                          /* 15° */
            (fabsf(mech_full - s_stall_last) < 0.0524f))       /* 3°：用连续累计角，跨 0°/360° 边界不断 */
        {
            if (s_stall_cnt < 0xFFFFU) s_stall_cnt++;
        }
        else
        {
            s_stall_cnt  = 0U;
            s_stall_last = mech_full;
        }

        if (s_stall_cnt >= 60U)                     /* 300ms @200Hz */
        {
            if (s_creeping == 0U)
            {
                s_creeping = 1U;
                s_creep_t  = 0U;
            }
            /* 蠕动开关与方向每拍无条件设置，不依赖易残留的 static 判断（烧录复位
             * 不清 SRAM 会导致蠕动标志残留、HALL 侧蠕动分支永不进入）。方向由误差
             * 符号引导朝目标拖，|err|>20° 才换向（防边界抖动来回闪），拖到
             * |err|<10° 由下方退出条件收手。 */
            HALL_SetForceCreep(1U);
            if (fabsf(err) > 0.3491f)               /* 20° */
            {
                int8_t want = (err >= 0.0f) ? 1 : -1;
                HALL_SetCreepOmega((float)want * HALL_CREEP_W_POS);
            }
        }
        else if (s_creeping != 0U)
        {
            if (fabsf(err) < 0.1745f)               /* 10°：已接近目标，退出蠕动 */
            {
                s_creeping = 0U;
                HALL_SetForceCreep(0U);
                HALL_SetCreepOmega(0.0f);
                s_stall_cnt  = 0U;
                s_stall_last = mech_full;
            }
        }
    }

    /* 位置环输出＝速度环目标。from_position=1：位置模式不做"静止切断输出"，
     * 否则位置环维持位置所需的保持力矩会被自己清掉 */
    state->current.q.Target = PID_Speed_Loop(&state->speed,
                                             state->position.pid.Out,
                                             HALL_GetMechSpeedRpm(),
                                             1U);
}

void Motor_RunMode_Set(MotorState_TypeDef * state, MotorRunModeEnum mode)
{
    state->run_mode = mode;

    /* 切模式即重置霍尔估算角到当前扇区中点，消除上次失步残留 */
    HALL_ResetEstimateForStart();

    /* 复位速度环积分/分频计数/Iq斜率缓存/摩擦前馈状态（PID.c） */
    PID_SpeedState_Reset(&state->speed);
    /* 电流环 q 轴目标归零：位置/速度环上一模式留下的指令不能带进新模式 */
    state->current.q.Target = 0.0f;

    if (mode == MOTOR_RUN_TORQUE)
    {   /* 切入转矩模式时恢复配置宏中的安全默认目标，之后可用VOFA命令在线修改 */
        s_torque_id_target = CURRENT_ID_TARGET;
        s_torque_iq_target = CURRENT_IQ_TARGET;
        state->current.d.Target = s_torque_id_target;
        state->current.q.Target = s_torque_iq_target;
    }
    else if (mode == MOTOR_RUN_POSITION)
    {   /* 无扰切入位置模式：以当前机械角为目标，误差为0，不会一进环就猛冲。
         * 未收到上位机位置命令时，转子走到哪就以哪为上电零点（进 RUN 时还会再抓一次）。 */
        if (s_pos_target_cmd == 0U)
        {
            state->position.pos_target_cmd = Motor_GetPosFB();   /* 单圈绝对角 [0,2π) */
        }
        state->position.pid.Target = state->position.pos_target_cmd;

        /* 清位置环积分/历史误差/输出；分频计数也一并清掉，与位置环初始化一致 */
        PID_PositionState_Reset(&state->position);
        state->position.div_cnt = 0;
    }
    else if (mode == MOTOR_RUN_SPEED)
    {   /* 切回速度模式：丢掉位置环留下的速度指令，避免被残留指令带跑 */
        state->position.pid.Out = 0.0f;
    }
}

/** @brief 预定位初始化：固定角度、清零电压、赋值预定位电流目标并进入ALIGN状态 */
void Motor_AlignState_Init(MotorState_TypeDef * state)
{
    /* 固定预定位角度，清零Vd/Vq */
    state->FOCParm.Theta = ALIGN_ANGLE;
    state->FOCParm.Vd    = 0.0f;
    state->FOCParm.Vq    = 0.0f;

    state->current.d.Target = ALIGN_CURRENT_ID;
    state->current.q.Target = 0.0f;

    state->state     = MOTOR_STATE_ALIGN;
    state->align_cnt = (uint32_t)(ALIGN_TIME_MS * Tim_Hz / 1000);
}
