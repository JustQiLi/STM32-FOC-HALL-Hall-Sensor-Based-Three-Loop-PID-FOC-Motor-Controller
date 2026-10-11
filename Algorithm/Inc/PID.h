#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#include "Motor_Config.h"
#include "QiLiFOC.h"        /* FOCParm_TypeDef：电流环的输入/输出都挂在这里 */

/*=========================== PID控制结构体 ==============================*/
typedef struct
{
    /* 参数 */
    float T;             /* 采样周期（秒） */
    float Kp;            /* 比例系数 */
    float Ki;            /* 积分系数 */
    float Kd;            /* 微分系数 */
    float OutMax;        /* 输出上限 */
    float OutMin;        /* 输出下限 */

    /* 输入与误差 */
    float Target;        /* 目标值 */
    float Feedback;      /* 反馈值 */
    float Error;         /* 当前误差 */
    float ErrorLast;     /* 上次误差 */

    /* 状态 */
    float Integral;      /* 积分累加值 */

    /* 输出 */
    float OutP;          /* 比例项输出 */
    float OutI;          /* 积分项输出 */
    float OutD;          /* 微分项输出 */
    float Out;           /* 总输出 */
} PIDParm_TypeDef;

/* 单个PID增益选择，用于上位机在线修改Kp/Ki/Kd */
#define PID_GAIN_KP   1U
#define PID_GAIN_KI   2U
#define PID_GAIN_KD   3U

/*=========================== 三个环各自的运行状态 ==============================*/
/* 每个环的状态只由本文件（PID.c）读写，所以就地声明在这里。
 * 状态机（StateMachine.h）只把它们当作成员组合进 MotorState_TypeDef，
 * 不再逐个散落十几个裸字段 —— 那样既看不出归属，也看不出谁在改它们。 */

/** @brief 低速摩擦前馈状态机（属于速度环） */
typedef enum
{
    FRICTION_OFF = 0,        /* 目标转速太小，不补偿 */
    FRICTION_REST,           /* 卡在静摩擦/齿槽峰上，前馈朝 STATIC_A 爬升 */
    FRICTION_MOVING,         /* 已脱开，前馈按 RELEASE_MS 平滑降回 KINETIC_A */
    FRICTION_BRAKE,          /* 反向或明显过冲，前馈撤到 0，制动权交回速度环 */
} FrictionStateEnum;

/** @brief 电流环：d/q 两个 PI，20kHz 每拍执行 */
typedef struct
{
    PIDParm_TypeDef d;
    PIDParm_TypeDef q;
} CurrentCtrl_TypeDef;

/** @brief 速度环：PI + 本环独占的辅助状态（分频、Iq 斜率限制、摩擦前馈） */
typedef struct
{
    PIDParm_TypeDef   pid;          /* 反馈单位 rpm，输出单位 A */
    float             f_speed_fb;   /* 低通后的实际转速(rpm)，本环唯一反馈源 */
    float             f_speed_raw;  /* PLL观测器原始输出(rpm, 未低通)=边沿真相，
                                     * 供超速制动判停保护（刹进停转点会触发回弹） */
    float             f_speed_raw_prev; /* 上一拍钳位后的PLL转速，供每帧速率钳位 */
    uint32_t          div_cnt;      /* 分频计数：每 SPEED_LOOP_DIV 拍跑一次 PI */
    float             last_cmd_iq;  /* Iq 指令斜率限制：上一拍实际输出(A) */
    FrictionStateEnum friction_state;
    uint16_t          friction_ms;  /* REST/BRAKE 状态已持续的时间(ms) */
} SpeedCtrl_TypeDef;

/** @brief 位置环：PI + 本环独占的辅助状态（原始命令值、分频） */
typedef struct
{
    PIDParm_TypeDef pid;            /* 目标/反馈单位 rad，输出单位 rpm */
    float           pos_target_cmd; /* 上位机下发的目标机械角(rad)，绝对式单圈 [0,2π)（0~360°）。
                                     * pid.Target 是"化成最短路径"的工作值（与 Feedback 同一圈），
                                     * 保证 PID_Loop 内部误差 = 最短路径误差 ∈ [-π,π]。 */
    uint32_t        div_cnt;        /* 分频计数：每 POS_LOOP_DIV 拍跑一次 PI */
} PositionCtrl_TypeDef;

/*=========================== 基础 PID ==============================*/
void PID_Init(PIDParm_TypeDef *pid);
void PID_SetParm(PIDParm_TypeDef *pid, float t, float kp, float ki, float kd);
void PID_SetGain(PIDParm_TypeDef *pid, uint8_t which, float value);
void PID_SetLimit(PIDParm_TypeDef *pid, float out_max, float out_min);
void PID_Loop(PIDParm_TypeDef *pid, float target, float feedback);

/*=========================== 电流环 ==============================*/
void PI_Current_Init(CurrentCtrl_TypeDef *cur);
void PI_Current_Loop(CurrentCtrl_TypeDef *cur, FOCParm_TypeDef *foc);

/*=========================== 速度环 ==============================*/
void PID_Speed_Init(SpeedCtrl_TypeDef *sc);
void PID_SpeedState_Reset(SpeedCtrl_TypeDef *sc);   /* 复位积分/分频/Iq斜率缓存/摩擦状态 */

/** @brief 速度环：分频到 1kHz 跑 PI，含低速摩擦前馈与 Iq 斜率限制
  * @param[in,out] sc          速度环状态
  * @param[in]     rpm_target  目标转速(rpm)
  * @param[in]     mech_rpm    Hall 净位移机械转速(rpm)，只给摩擦前馈判"卡住没有"用；
  *                           必须是调用者从 Hall 模块取的最新值，本模块不认识 Hall
  * @param[in]     from_position 1 = 本拍目标是位置环给的。位置模式下不做"静止切断输出"，
  *                           否则位置环维持位置所需的保持力矩会被自己清掉
  * @return 本拍 Iq 指令(A)，已过斜率限制；调用者自行写入电流环 q 轴目标 */
float PID_Speed_Loop(SpeedCtrl_TypeDef *sc, float rpm_target, float mech_rpm, uint8_t from_position);

/*=========================== 位置环 ==============================*/
void PID_Position_Init(PositionCtrl_TypeDef *pc);
void PID_PositionState_Reset(PositionCtrl_TypeDef *pc);  /* 复位积分/历史误差/输出，不动分频 */

/** @brief 位置环：分频到 200Hz 跑 PI，输出速度指令存入 pc->pid.Out
  * @param[in,out] pc   位置环状态
  * @param[in]     mech 当前机械角反馈(rad)，由调用者从 Hall 模块读取 */
void PID_Position_Loop(PositionCtrl_TypeDef *pc, float mech);

#endif /* __PID_H__ */
