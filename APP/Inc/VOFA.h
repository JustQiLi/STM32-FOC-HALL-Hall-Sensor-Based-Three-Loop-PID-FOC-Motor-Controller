#ifndef __VOFA_H__
#define __VOFA_H__

#include "stdint.h"
#include "string.h"
#include "usart.h"
#include "StateMachine.h"
#include "tim.h"

/*===================================  JustFloat协议  ==============================*/
/* 各模式发送通道数：发送函数只根据当前run_mode分支，不在VOFA里切换控制模式 */
#define VOFA_CH_TORQUE    6    /* 转矩模式：Id/Iq目标与实际 + 速度反馈 + 电角度 */
#define VOFA_CH_SPEED     18   /* 速度模式：状态 + Id/Iq + 电流环参数/输出 + Vd/Vq + 电角度 + 三相CCR */
#define VOFA_CH_POSITION  8    /* 位置模式：位置目标/实际/误差 + 速度目标/实际 + Iq目标/实际 + 电角度 */
#define VOFA_CH_MAX       VOFA_CH_SPEED
#define VOFA_FRAME_LEN(ch)       ((ch) * 4 + 4)
#define VOFA_FRAME_LEN_MAX       VOFA_FRAME_LEN(VOFA_CH_MAX)

/*=========================== 结构体 ==============================*/
typedef struct
{
    float    Send_Data_Array[VOFA_CH_MAX];
    uint8_t  Byte_Data_Array[VOFA_FRAME_LEN_MAX];
    uint8_t  channels;             /* 当前帧通道数 */
    uint8_t  tx_busy;              /* DMA发送忙标志：1=正在发送，0=空闲 */
} VofaJustWater_TypeDef;

/*=========================== 函数声明 ==============================*/
void VofaJustWater_Send(void);
void Vofa_Data_Process(void);
void VOFA_TxCpltCallback(void);

#endif /* __VOFA_H__ */
