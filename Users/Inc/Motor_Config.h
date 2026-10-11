/*
 * Motor_Config.h —— 全工程可调参数集中于此（V1.0）
 * 修改记录 日期 | 作者 | 版本 | 修改描述
 */
#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__  
#include "QiLiFOC.h"
/*=========================== MCU系统常量 ==============================*/
#define Tim_Hz          20000   
/*=========================== 数学常量 ==============================*/
#define _PI             3.141592653f
#define _PI_3           1.047197551f
#define _2PI            6.283185307f
#define SQRT3           1.7320508f
/* 角度单位换算：位置环内部统一用 rad，对外的位置接口用度(°) */
#define DEG2RAD(x)      ((x) * _PI / 180.0f)
#define RAD2DEG(x)      ((x) * 180.0f / _PI)
/*=========================== FOC实际参数设定 ==============================*/
#define _Udc    24.0f             //母线电压(根据实际电路修改)
#define _ARR    4200.0f           //定时器ARR值(根据实际定时器修改)
/*=========================== VF强拖参数设定 ==============================*/
#define acc                  0.0f
#define target_speed         0.0f
#define dead_zone            0.00005f
#define ts                   (1.0f/Tim_Hz)
/*=========================== HALL测速参数设定 ==============================*/
#define HALL_POLE_PAIR      4.0f        /* 极对数，按实际电机改 */
/* TIM5 预分频：84MHz/(419+1)=200kHz；不取 200kHz 低速会回绕，见 bsp_encoder.c */
#define HALL_TIM_PSC        419U
/* 计数频率 200kHz(5us/tick)：16bit 可表示 327ms 边沿间隔，覆盖启动低速段 */
#define HALL_TICK_HZ        200000.0f
#define HALL_GATE_MS        20

/*---- 1. TIM5 时间基准 ----*/
#define HALL_TICK_S             (1.0f / HALL_TICK_HZ)
#define HALL_EDGE_DEBOUNCE_TICKS  20U /* 100us@200kHz：只挡"同一机械跳变引起的重复捕获/EMI 尖峰"。
                                        * 抖动/尖峰由 TIM5 的 IC1Filter=0x0F 数字滤波 + 本窗共同挡；
                                        * 100us 相对最小扇区时间(3500rpm≈1.4ms)有 14 倍余量。 */
#define POS_FB_LP_ALPHA        0.20f   /* 位置反馈低通(增量式回绕)：滤霍尔15°换相台阶，位置环才不会被台阶逼出饱和极限环；200Hz下≈6Hz */
#define POS_OUT_RATE_RPM      2.0f     /* 位置环输出(速度目标rpm)每5ms最大变化率：防高KP饱和猛冲→目标附近bang-bang极限环 */
#define HALL_TICK_PER_MS        (HALL_TICK_HZ / 1000.0f)

/*---- 2. 边沿测速平滑 ----*/
/* 边沿测速一阶低通系数（每个霍尔边沿更新一次） */
#define HALL_SPEED_ALPHA        0.25f
/* 突变判据：差值 > 比例×(估算+1rad/s) 时直接跳新值；换向绝不能低通，霍尔 60° 才测一次速跟不快 */
#define HALL_SPEED_JUMP_RATIO   1.00f
/* 单次边沿测速的物理限幅 rad/s（≈7600rpm@4极对），剔除野值 */
#define HALL_OMEGA_E_MAX        2000.0f

/*---- 3. 相位校正（平滑吃误差） ----*/
/* 校正增益：1.0 = 一个扇区内把边沿残余误差完全吃完（死拍）；再大收敛更快但带纹波 */
#define HALL_PHASE_GAIN         1.00f
/* 校正速度相对估算转速的上限：霍尔测速是上一扇区均值，加速时天然滞后，必须允许拉到 2 倍 */
#define HALL_PHASE_CORR_RATIO   1.00f
/* 校正速度下限 rad/s：低速/启动时估算转速很小，保证仍有纠正能力 */
#define HALL_CORR_MIN_OMEGA     2.00f
/* 相位误差硬复位门限：取 30° 是因为启动后第一个边沿之前估算器没有测速信息、转速滞后几十度 */
#define HALL_SNAP_ERR           (_PI / 6.0f)
#define HALL_EDGE_BACK_LIM   (5.0f * _PI / 180.0f)   /* 回退型假边沿阈值：方向未变但边沿目标角回退>5°判假(rad) */
/* 静摩擦启动补偿(仅扭矩模式)：堵转且目标|iq|<FLOOR 时实际电流抬到 FLOOR
 * (静摩擦≈0.35A)，转子启动(>OUT rpm)后回到用户目标维持。瓶颈只在启动阶段。 */
#define CURR_BOOST_FLOOR_A      0.50f    /* 补偿下限电流(A)：需大于齿槽/静摩擦临界值(约0.40A)才能自动启动 */
#define CURR_BOOST_RPM_IN       10.0f    /* 进入补偿转速阈值(rpm)：静止判定 */
#define CURR_BOOST_RPM_OUT      30.0f    /* 退出补偿转速阈值(rpm)：已启动判定 */

/*4丢边沿保护*/
/* 超过"上次霍尔周期"的多少倍没有新边沿就认为转子停住（角度不再外推） */
#define HALL_EDGE_LOST_RATIO    3U
#define HALL_EDGE_LOST_MS_MIN   40U
#define HALL_EDGE_LOST_MS_MAX   400U
/* 判定停转后估算转速的线性衰减斜率(rad/s²)：按此斜率衰减到 0，不要直接清零
 * （清零会让电角度冻死、电流矢量静止在定子上，转子被吸到 Δ=0 而自锁）。 */
#define HALL_LOST_DECAY_RS      60.0f

/*---- 4b 低速角度助推（解决"扭矩模式 iq<0.4A 不转"）----*/
/* 原理：霍尔在零速/极低速下给不出转子在一个 60° 扇区内的细分位置。若没有新边沿时
 * 把电角度外推速度清零并硬夹在扇区内，施加的电流矢量会在定子上静止，转子永磁 d 轴
 * 被吸到这个矢量上（Δ→0），转矩 = Kt·Iq·cos(Δ) → 0；转子不动又永远没有新边沿 → 自锁。
 * 助推：命令了电流且转速很低时，让电角度按一个很小的"蠕动速度"继续旋转（= 开环 I/F 硬拖），
 * 转子跟着转过 15° 机械角就会产生新边沿，随即交回霍尔闭环。置 0 完全退回旧行为。 */
#define ANGLE_ASSIST_ENABLE      1
#define ANGLE_ASSIST_IQ_MIN      0.05f   /* |Iq指令| 超过该值才助推(A)：0 电流时绝不拖 */
#define ANGLE_ASSIST_RPM_ENTER   25.0f   /* 转速低于该值开始助推(rpm) */
#define ANGLE_ASSIST_RPM_EXIT    45.0f   /* 转速高于该值停止助推，交回霍尔(rpm) */
#define ANGLE_ASSIST_W_CREEP     10.0f   /* 助推电角速度 rad/s(电角度) ≈24rpm@4极对，慢到跟得上 */
#define HALL_CREEP_W_POS       3.0f        /* 位置环堵转蠕动速度(电rad/s，≈7rpm机械)：误差引导单向慢拖，防止拖过头 */

/*5零位细分标定*/
/* 拖动电角速度 rad/s（120°/s 电角度）：足够慢，转子跟得上 d 轴不会甩脱 */
#define HALL_CALIB_RATE         (2.0f * _PI / 3.0f)
/* 拖动超过这个角度还没标定成功就放弃（拿两个边沿最多只需 120° 电角度） */
#define HALL_CALIB_MAX_SWEEP    _PI
/* 相邻两边沿之间转子应恰好转过 60° 电角度，偏差超过此容差说明没跟上 */
#define HALL_CALIB_TOL          (25.0f * _PI / 180.0f)

/*力矩方向自检*/
/* 拖动标定只能定出"模 π"的零位，判错会让闭环极性反 → 上电飞车，故必须自检 */
#define HALL_DIRCHK_IQ      0.25f    /* 自检 q 轴电流(A)：小到不冲，大到能推动转子 */
#define HALL_DIRCHK_MS      300U     /* 自检最长时间(ms)：等一个霍尔边沿 */
/*=========================== 上电预定位参数设定 ==============================*/
#define ALIGN_ANGLE         0.0f        /* 第一次预定位电角度(rad)：对齐 A 相轴线 */
#define ALIGN2_ANGLE        _PI         /* 第二次预定位电角度(rad)：180° 方向 */
#define ALIGN_TIME_MS       1000
#define ALIGN_CURRENT_ID    0.5f        /* 预定位 d 轴电流目标(A) */
/*=========================== 锁相环PLL参数设定 ==============================*/
#define kp_theta            300.0f
#define ki_theta            20000.0f
/*=========================== ADC电流采样参数设定 ==============================*/
#define rs                  0.02f
#define gain                4.023f
#define vref                3.3f
#define ADC_FULL_SCALE_12BIT    4095.0f
/* 三相电流一阶低通系数(每 50us 一拍)。filter.c 里写死的 0.2 对应极点≈710Hz，
 * 三级串联在 300Hz 电流环带宽上吃掉约 69° 相位，相位裕度只剩十几度。
 * 0.4 对应极点≈1.6kHz，相位损失减半。1.0 = 完全旁路(不推荐，会放进 ADC 噪声)。 */
#define CURR_FILTER_ALPHA       0.40f
/*=========================== 电流环PI参数设定==============================*/
#define MOTOR_R            0.415f      /* 相电阻 Ω（官方参数） */
#define MOTOR_L            0.00124f    /* 相电感 H（官方 1.24mH） */
#define CURRENT_BW_HZ       300.0f      /* 电流环带宽 Hz，20kHz 采样取 300~500 */
/* 零极点对消法：Kp=L·ωc·2π, Ki=R·ωc·2π */
#define CURRENT_KP          (MOTOR_L * CURRENT_BW_HZ * 2.0f * _PI)
#define CURRENT_KI          (MOTOR_R * CURRENT_BW_HZ * 2.0f * _PI)
#define CURRENT_KD          0.0f         
/* 输出限幅：SVPWM 最大不失真电压 = Udc/√3 */
#define CURRENT_OUT_MAX     (_Udc / SQRT3)
#define CURRENT_OUT_MIN     (-_Udc / SQRT3)
/*PI电流环 Id Iq目标值*/
#define CURRENT_ID_TARGET   0.0f
#define CURRENT_IQ_TARGET   (0.0f)
#define TORQUE_CURRENT_MAX  2.0f    /* VOFA 转矩命令限幅(A)：Id/Iq 均限制在 ±2A。
                                     * 2.0A 幅值 = 1.41A RMS = 额定 5.9A 的 24% */
/*=========================== 速度环参数设定 ==============================*/
#define SPEED_TARGET_RPM    100.0f
#define SPEED_LOOP_DIV      20        /* 速度环分频：20kHz/20 = 1kHz */
#define SPEED_KP            0.002f    /* 比例 A/rpm：过大在低速产生停顿点与振荡 */
#define SPEED_KI            0.010f    /* 积分 A/(rpm·s)：反转问题由速度环制动逻辑解决，
                                     * 增益只需保证稳态积分足够 */
#define SPEED_KD            0.0f
#define SPEED_IQ_MAX        2.00f     /* 速度环输出 Iq 限幅(A)=1.41A RMS(额定 5.9A 的 24%) */
#define SPEED_IQ_POS_LIM_LOW_BASE_A  0.15f     /* 低速正积分下限基准(A) */
#define SPEED_IQ_POS_LIM_LOW_K_A     0.0008f  /* 低速正积分每rpm增量(A/rpm), @200=0.25A */
#define SPEED_IQ_NEG_LIM_A  0.50f     /* 积分负向钳制(A)：比低速稳态抵消前馈所需(-0.41A)
                                     * 多0.09A余量；刹车后积分最深-0.50A，恢复期仍有正向
                                     * 保持力矩、无转矩真空。正向积分不限(±SPEED_IQ_MAX=2.00A)保证
                                     * 升档/启动爬升力矩。 */
#define SPEED_IQ_SLEW_A_S   100.0f    /* Iq指令斜率限制 A/s：50A/s时退出饱和约20ms，低速容易冲过头 */
#define SPEED_FB_LP_HZ      30.0f     /* 速度反馈一阶低通截止(Hz)，降低控制相位滞后 */
#define SPEED_MEAS_WINDOW_DIV 200     /* 低速机械角测速窗口：200*50us=10ms；50ms延迟过大已否定 */
#define SPEED_MEAS_LOW_RPM  200.0f    /* 200rpm以下主要使用机械角净位移测速 */
#define SPEED_MEAS_HIGH_RPM 300.0f    /* 300rpm以上主要使用霍尔边沿周期测速 */
#define SPEED_FB_RPM_MAX    3600.0f   /* 反馈物理限幅(rpm)：先剔粗大尖峰再低通 */
#define SPEED_DEADBAND_RPM  0.0f      /* 干净基线关闭死区，避免小误差被忽略 */
/* 低速摩擦补偿：REST阶段从动摩擦电流平滑爬升到静摩擦电流；脱开后先保留
 * 释放电流并平滑降至动摩擦电流，避免转矩突降导致再次停转。 */
#define SPEED_FRICTION_ENABLE        1
#define SPEED_FRICTION_STATIC_A     0.75f   /* 静摩擦补偿上限(A)：与速度环比例项叠加后覆盖静摩擦脱开所需电流 */
#define SPEED_FRICTION_KINETIC_A    0.15f   /* 转动后的维持补偿(A)：接近维持转动所需电流 */
#define SPEED_FRICTION_RELEASE_A    0.35f   /* 刚脱开后的维持电流(A)，防止从静摩擦电流直接跌至动摩擦电流 */
#define SPEED_FRICTION_START_A     0.50f   /* 低速段目标前馈电流(A)：>静摩擦0.4A，目标非零立即给，电机马上启动 */
#define SPEED_FRICTION_MIN_CMD_RPM  5.0f    /* 速度指令超过该值才补偿 */
#define SPEED_FRICTION_REST_RPM     10.0f   /* 实际速度低于该值进入REST */
#define SPEED_FRICTION_MOVE_RPM     20.0f   /* 实际速度高于该值进入MOVING */
#define SPEED_FRICTION_OVERSHOOT_RPM 30.0f /* 实际速度超过目标该值进入BRAKE并关闭补偿 */
#define SPEED_FRICTION_RAMP_MS      150U    /* REST阶段从动摩擦补偿爬升到静摩擦补偿的时间 */
#define SPEED_FRICTION_RELEASE_MS    80U    /* 脱开后从释放电流平滑降至动摩擦电流的时间 */
#define SPEED_FRICTION_STATIC_MS    500U    /* 静态补偿最长持续时间，防止长时间堵转 */
#define SPEED_NO_BRAKE_RPM          200.0f  /* 该速度以下同向运行不输出反向制动Iq，交给自然摩擦减速 */
#define SPEED_OVERSPEED_BRAKE_A     0.40f   /* 超速最大反向制动Iq(A)：>动摩擦0.15A保留有效制动，
                                     * 但不会把低速段刹穿零速造成真反转 */
#define SPEED_BRAKE_STOP_RPM_MARGIN 15.0f   /* PLL判停保护：原始PLL(边沿真相)回落到
                                     * 目标+该值以下即撤超速制动（刹进停转点会触发齿槽回弹） */
#define SPEED_BRAKE_STOP_HOLD_A     0.20f   /* 撤制动后的最小正向保持电流(A)：
                                     * >动摩擦0.15A保持转动，不产生新过冲驱动 */
/* 摩擦前馈仅用于"目标速度较高、霍尔净位移测速不再离散"的区段：
 * 低速下 HALL_GetMechSpeedRpm() 在 0~250rpm 之间离散乱跳，摩擦前馈状态机在
 * REST/MOVING 间反复切换 → Iq 前馈 0.75↔0.15 阶跃 → 电机抖振自激。低速段的
 * "突破静摩擦"已由 ANGLE_ASSIST(I/F 蠕动拖过静摩擦)承担，PI 积分(可到
 * SPEED_IQ_MAX)负责稳态力矩，无需前馈。 */
#define SPEED_FRICTION_CMD_RPM_MAX  200.0f  /* 速度目标(rpm)绝对值低于该值时禁用摩擦前馈 */
/* PLL 电角速度 ωe(rad/s) 换算机械转速(rpm)：rpm = ωe·60/(2π·P) */
#define RPM_FROM_WE(we)     ((we) * 60.0f / (2.0f * _PI * HALL_POLE_PAIR))
/* PLL 机械角速度 ωm(rad/s) 换算机械转速(rpm)：rpm = ωm·60/(2π) */
#define RPM_FROM_WM(wm)     ((wm) * 60.0f / (2.0f * _PI))

/*---- P0-D：PLL 转速观测器（解决"霍尔测速 ±90rpm 阶梯 + 40ms 延迟"）----*/
/* 现成的 PLL.c（二阶锁相环）跟踪连续机械角，输出连续平滑的机械角速度，无采样保持
 * 阶梯、延迟远小于"10ms 窗净位移 + 30Hz 低通"。启用后 Run_UpdateObserver 每拍调用
 * PLL_Loop()，速度环 PI 反馈(f_speed_fb)直接用 PLL 输出，不再走 30Hz 低通。
 * 摩擦前馈状态机判定仍用 HALL_GetMechSpeedRpm()（净位移，注释见 PID.c）。
 * 置 0 退回原"净位移+T法融合+低通"路径。 */
#define SPEED_OBS_PLL_ENABLE    1
/* PLL 转速观测器输出低通(Hz)：霍尔 60° 换相 + 六边沿平均测速滞后 → 外推角度在扇区
 * 终点被钳位等新边沿 → 机械角每扇区"停走"台阶(~40Hz@100rpm) → PLL 输出周期性转速
 * 尖峰 → 速度环过激(Iq 猛冲至饱和) → 力矩冲击加剧脉动，正反馈自激。
 * 此低通衰减换相台阶尖峰，速度环只响应中低频，电流平稳后电机凭惯性自然平滑。
 * 15Hz: 40Hz 衰减~0.3倍; 太低压起动/响应变软, 太高滤不掉尖峰。位置环自调时需重新整定。 */
#define SPEED_PLL_LP_HZ         15.0f

/*---- 死区电压补偿（低速小电流转矩指令被 0.171V 死区误差淹没）----*/
/* 逆变器死区+管压降造成每相平均电压损失 ≈ (Tdt/Ts)·Udc = (357ns/50us)·24V ≈ 0.171V，
 * 而静止维持 0.3A 只需 I·R=0.125V —— 不补偿，低 Iq 指令本身就是"糊"的。
 * 补偿量按相电流符号投影到 dq 叠加到 Vd/Vq；|i|<DEAD_IZERO_A 时线性过渡避免零电流
 * 抖动。符号/幅值需上机标定：若小电流电流环鸣叫、或 Iq 实际与指令偏差方向异常，
 * 把 DEADTIME_COMP_SIGN 改为 -1 再试；仍异常则置 0 关闭。 */
#define DEADTIME_COMP_ENABLE    1
#define DEAD_TIME_V             0.171f  /* 死区平均电压损失(V)：按 (Tdt/Ts)·Udc 估算，可作为标定初值 */
#define DEAD_IZERO_A            0.05f   /* 死区带：低于该电流视为 0，避免正反馈抖动(A) */
#define DEADTIME_COMP_SIGN      1.0f    /* 补偿符号：+1/-1，按电流方向确定 */
/*=========================== 位置环参数设定 ==============================*/
/* 位置环串在速度环外面：输入/反馈是机械角(rad)，输出直接就是速度环目标(rpm)，
 * 所以 Kp 单位是 rpm/rad；环周期 = POS_LOOP_DIV·ts = 5ms（与 PID_Speed_Init 同一套路）
 *
 * !! 整定要点 !!
 * 位置环输出的 rpm 要经过速度环才变成电流：Iq ≈ SPEED_KP·rpm_target。
 * 所以整个串级的"位置刚度" = POS_KP × SPEED_KP × Kt：POS_KP 太小就只有很小的
 * 保持力矩，扛不住 0.01 N·m 量级的齿槽/摩擦，表现为松开手回不了位。
 *
 * 另外 PID_Speed_Loop() 里有一条"目标≈0 且反馈在死区内就清积分、输出 0"的静止零转矩
 * 逻辑，位置模式下位置误差小于 POS_DEADBAND_RAD 附近时位置环输出本来就只有零点几 rpm，
 * 会被那条逻辑整条清掉。现在已在位置模式下禁用（见 PID.c）。
 *
 * 整定顺序：先只加 POS_KP 到"轴推不动、松手能回位"，再慢慢加 POS_KI 消除静态误差。
 * 轴发麻/嗡嗡叫或来回摆 = POS_KP 或 POS_KI 太大；软绵绵 = 太小。
 * 注意位置环输出上限只有 POS_RPM_MAX=300rpm，Kp 再大也不会更快。 */
#define POS_LOOP_DIV       100      /* 位置环分频：20kHz/100 = 200Hz，比速度环(1kHz)慢一档 */
#define POS_KP             200.0f    /* 位置环比例 (rpm/rad) ≈3.5rpm/度；先保证松手有力回位 */
#define POS_KI             10.0f     /* 位置环积分 (rpm/(rad·s))：消除小角度静态误差 */
#define POS_KD             0.0f
#define POS_RPM_MAX        40.0f    /* 位置环输出限幅(rpm)：必须低到速度环不饱和(Iq<1.25A)，
                                     * 否则速度环饱和→超速→霍尔高速判向错→位置反馈跳变失控 */
#define POS_DEADBAND_RAD   0.0f     /* 死区(rad)：0 = 关闭，避免目标附近强制切断保持力矩 */
/* 位置模式破静摩擦启动补偿（仿转矩模式 CURR_BOOST）：
 * 位置环指令较大(≥POS_BOOST_CMD_RPM，位置误差≥~2.9°) 且转子静止
 * (低通反馈≤POS_BOOST_RPM_IN) 时，Iq 抬底到 POS_BOOST_A 破障；
 * 转子启动后撤除交回速度环。静止判定必须用低通反馈而非净位移测速
 * （齿槽峰临界微动时净位移会报假速度、使补偿反复撤除导致力矩不足）。
 * 仅位置模式生效，不影响速度/转矩模式。 */
#define POS_BOOST_A            0.65f    /* 破障电流(A)：>静摩擦0.4~0.5A留余量，<0.68A EMI阈值 */
#define POS_BOOST_CMD_RPM      10.0f    /* 位置环指令(rpm)≥该值才补偿：小误差不硬推(≈2.9°) */
#define POS_BOOST_RPM_IN       10.0f    /* 低通反馈转速≤该值判定静止，进入补偿 */
#define POS_BOOST_RPM_OUT      30.0f    /* 低通反馈转速>该值判定已启动，撤补偿 */
#define POS_TARGET_RAD     0.0f     /* 默认目标位置(rad，相对上电零点) */
#endif /* __MOTOR_CONFIG_H__ */
