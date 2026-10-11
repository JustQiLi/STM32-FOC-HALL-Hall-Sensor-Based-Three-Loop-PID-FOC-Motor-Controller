/*
 * 文件名 
 * 作者 
 * 版本 V1.0
 * 创建日期 
 * 功能 本文件功能说明
 * 修改记录 日期 | 作者 | 版本 | 修改描述
 */
#ifndef __VF_H__
#define __VF_H__
/*=========================== 头文件 ==============================*/
#include <math.h>
#include "Motor_Config.h"
/*=========================== 宏定义 ==============================*/
/*=========================== 结构体 ==============================*/
/**
 * @brief VF开环控制参数结构体定义
 **/
typedef struct 
{
    /*输入参数*/
    float ACC;                   /*加速度*/
    float Target_Speed;          /*目标速度*/ 
    float Dead_Zone;             /*死区阈值*/
    float Ts;                    /* 步长 */
    
    /*中间参数*/
    float Speed_Error;           /*速度误差*/
    float Speed_Error_Dead;      /*带死区的速度误差*/
    float Acc_Dir;               /*加速度方向*/
    float Speed_Integrator;      /*速度积分器*/
    float Theta_Integrator;      /*角度积分器*/
    
    /*输出参数*/
    float Speed;                 /*实际速度*/
    float Theta;                 /*实际角度*/
}VFControl_TypeDef;
/*=========================== 函数声明 ==============================*/
void VFControl_Init(VFControl_TypeDef * state ,float parm_Acc ,float parm_Target_Speed,float parm_Dead_Zone , float parm_Ts );
void VFControl_Run(VFControl_TypeDef * state,float * Targer_Speed_FOC);
#endif/* __VF_H__ */