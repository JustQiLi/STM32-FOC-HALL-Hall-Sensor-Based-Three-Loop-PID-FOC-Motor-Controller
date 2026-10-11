#ifndef __STATE_MACHINE_H_
#define __STATE_MACHINE_H_

#include "QiLiFOC.h"
#include "bsp_tim.h"
#include "bsp_encoder.h"
#include "Motor_Config.h"
#include "VF.h"
#include "filter.h"
#include "PLL.h"
#include "bsp_adc.h"
#include "PID.h"
/*0°校准，采用两次对齐校准*/
typedef enum
{
    MOTOR_STATE_ALIGN = 0,   /* 第一次对齐：0度 */
    MOTOR_STATE_ALIGN2,      /* 第二次对齐：180度 */
    MOTOR_STATE_CALIB,       /* 零位细分标定：缓慢拖动转子跨过霍尔边沿，精确求零位与方向 */
    MOTOR_STATE_DIRCHK,      /* 力矩方向自检：给小的 +Iq 探测极性，判掉 π 二义性，防飞车 */
    MOTOR_STATE_RUN,
} MotorStateEnum;

/* RUN 状态内部的控制模式（可在线切换） */
typedef enum
{
    MOTOR_RUN_TORQUE = 0,    /* 转矩模式：直接给定目标Iq */
    MOTOR_RUN_SPEED,         /* 速度模式：速度环PI输出目标Iq */
    MOTOR_RUN_POSITION,      /* 位置模式：位置环PI输出速度指令(rpm)，交给速度环 */
} MotorRunModeEnum;
/* 电机总状态：只放"状态机自己"的东西 + 三个环各自的状态块。
 * 三个环的内部细节（分频计数、Iq斜率缓存、摩擦前馈状态、目标命令值……）
 * 全部封装在 PID.h 的 CurrentCtrl_TypeDef / SpeedCtrl_TypeDef / PositionCtrl_TypeDef 里，
 * 由 Algorithm/Src/PID.c 独占读写。本文件不再逐个展开这些字段，
 * 也不再把 FrictionStateEnum 拉进来 —— 状态机不需要知道摩擦前馈有几个状态。 */
typedef struct
{
    /* 硬件与算法参数（由各自的模块初始化并维护） */
    FOCParm_TypeDef       FOCParm;
    VFControl_TypeDef     VFParm;
    HALLParm_TypeDef      HALLParm;
    FilterMethod_TypeDef  FilterMethod;
    PLLParm_TypeDef       PLLParm;
    ADCSamp_TypeDef       ADCSamp;

    /* 三个环的运行状态，定义在 PID.h */
    CurrentCtrl_TypeDef   current;        /* 电流环：d/q 两个PI，20kHz */
    SpeedCtrl_TypeDef     speed;          /* 速度环：PI + 分频 + Iq斜率 + 摩擦前馈 */
    PositionCtrl_TypeDef  position;       /* 位置环：PI + 分频 + 目标命令 */

    /* 状态机自己的状态 */
    MotorStateEnum        state;          /*电机运行状态下不同模式选择*/
    MotorRunModeEnum      run_mode;       /* 电机当前状态 */
    uint32_t              align_cnt;      /*用于0°校准时长*/
} MotorState_TypeDef;

void Motor_StateMachine_Init(MotorState_TypeDef * state);
void Motor_StateMachine_Run(MotorState_TypeDef * state);
void Motor_RunMode_Set(MotorState_TypeDef * state, MotorRunModeEnum mode); /* 运行模式选择 */
float Motor_GetSpeedFB(void);   /* 读取速度环实际使用的滤波后转速(rpm) */
void Motor_PosTarget_Set(float deg); /* 设置位置模式目标机械角，单位：度(°)；内部换算成 rad */
float Motor_GetPosFB(void);     /* 读取位置模式实际使用的机械角反馈(rad)：绝对式单圈 [0,2π) */
float Motor_GetPosTarget(void); /* 读取位置模式目标机械角，单位：度(°) */
MotorRunModeEnum Motor_GetRunMode(void); /* 读取当前运行模式(0=转矩 1=速度 2=位置) */
void Motor_TorqueIdTarget_Set(float id_a); /* 设置转矩模式Id目标(A)，不切换模式 */
void Motor_TorqueIqTarget_Set(float iq_a); /* 设置转矩模式Iq目标(A)，不切换模式 */
void Motor_AlignState_Init(MotorState_TypeDef * state); /* 上电预定位：置固定电角度、给Id电流、进入ALIGN状态 */

extern MotorState_TypeDef MotorBLDC;

/* 运行时目标转速(rpm)，可由VOFA命令实时修改，正=正转 */
extern float speed_target;
#endif
