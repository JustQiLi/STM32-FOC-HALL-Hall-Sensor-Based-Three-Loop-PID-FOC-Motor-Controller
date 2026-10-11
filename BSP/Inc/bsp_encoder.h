/**
  * @file    bsp_encoder.h
  * @brief   霍尔编码器接口：电角度估算、零位标定、力矩方向自检、机械角/测速
  * @note    霍尔接口 TIM5 (PH10/PH11/PH12)，200kHz 时基；估算器在 20kHz FOC 中断中调用。
  *          用法：
  *          1. HALL_Speed_Init()            —— 上电初始化（含 TIM 配置）
  *          2. HALL_CalibAlign1/Align2      —— 两次电角度对齐（记录兜底零位）
  *          3. HALL_CalibRampBegin/Run/Finish —— 零位细分标定（Id 拖动）
  *          4. HALL_DirCheckBegin/Run/Finish —— 力矩方向自检（判模 π 二义性）
  *          5. 运行期 FOC 中断里每拍调 HALL_GetElecAngle() 取电角度；
  *             上层状态机用 HALL_GetSpeedRpm()/HALL_GetMechAngle()/HALL_GetMechSpeedRpm()。
  *          6. 位置模式切入时 HALL_SetDebounceTicks() 加大去抖窗，退出恢复默认；
  *             HALL_ResetSpeedObserver() 清掉残留假速度。
  * @note    模块参数（HALL_TICK_HZ / 去抖窗 / 测速融合阈值等）在 Motor_Config.h 统一配置。
  */

#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#include <stdint.h>
#include "tim.h"
#include "Motor_Config.h"

#ifndef HALL_CALIB_ENABLE
#define HALL_CALIB_ENABLE   1
#endif

#ifndef HALL_BREAKAWAY_MS
#define HALL_BREAKAWAY_MS   60U
#endif

/*=========================== 霍尔相位定义 ==============================*/
#define HALL_CH1_PORT       GPIOH
#define HALL_CH2_PORT       GPIOH
#define HALL_CH3_PORT       GPIOH
#define HALL_CH1_PIN        GPIO_PIN_10   /* A(U) */
#define HALL_CH2_PIN        GPIO_PIN_11   /* B(V) */
#define HALL_CH3_PIN        GPIO_PIN_12   /* C(W) */

/*=========================== 霍尔模块状态 ==============================*/
typedef struct
{
    uint8_t  last_phase;      /* 上次霍尔相位（边沿方向查表用） */
    uint8_t  first;           /* 1 = 首个霍尔捕获尚未处理 */
    uint32_t last_period;     /* 上次霍尔周期 CCR1（tick），用于角度插值 */
} HALLParm_TypeDef;

/*=========================== 霍尔功能接口 ==============================*/

/* 初始化 / 标定 / 方向自检 */
void    HALL_Speed_Init(HALLParm_TypeDef *state_hall);
void    HALL_CalibAlign1(void);
void    HALL_CalibAlign2(void);
void    HALL_CalibRampBegin(float theta_start);
uint8_t HALL_CalibRampRun(float *theta_cmd);
void    HALL_CalibFinish(void);
uint8_t HALL_IsCalibrated(void);
void    HALL_DirCheckBegin(void);
uint8_t HALL_DirCheckRun(float *theta_cmd);
void    HALL_DirCheckFinish(void);

/* 运行期（FOC 中断每拍） */
float   HALL_GetElecAngle(HALLParm_TypeDef *state);   /* 电角度 [0,2π)，主入口 */
float   HALL_GetElecOmega(void);                       /* 最新单边沿电角速度 rad/s */
float   HALL_GetSpeedRpm(void);                        /* 融合后的机械转速 rpm */
float   HALL_GetMechSpeedRpm(void);                    /* 固定窗口机械角净位移测速 rpm */
float   HALL_GetMechAngle(void);                       /* 机械角 rad（连续多圈，标定结束为 0） */
void    HALL_SetMechAngleZero(void);                   /* 把当前机械角置 0 */
uint8_t HALL_TakeBreakawayFlag(void);                  /* 取走"刚脱开卡滞"标志 */
void    HALL_ResetSpeedObserver(void);                 /* 清速度观测器残留假速度 */
void    HALL_SetDebounceTicks(uint32_t ticks);         /* 动态调整去抖窗 */

/* 低速角度助推（I/F 硬拖，位置环用） */
void    HALL_SetCreepOmega(float w);
void    HALL_SetForceCreep(uint8_t on);
void    HALL_ResetEstimateForStart(void);

/* 其他 */
int32_t HALL_GetPhase(void);                           /* 当前霍尔电平编码 0~7 */

#endif /* __BSP_ENCODER_H__ */
