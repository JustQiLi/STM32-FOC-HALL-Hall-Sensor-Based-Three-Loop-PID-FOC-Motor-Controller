/*
 * 文件名 
 * 作者 
 * 版本 V1.0
 * 创建日期 
 * 功能 本文件功能说明
 * 修改记录 日期 | 作者 | 版本 | 修改描述
 */
#include "VF.h"
/**
 * @brief VF强拖初始化参数函数
 * @param[in] VF参数结构体，加速度，目标速度，死区阈值，步长
 * @param[out] 更新后VF参数结构体
 * @param[inout] 输入输出参数
 * @return 无
 * @note 备注说明
 * */
void VFControl_Init(VFControl_TypeDef * state ,float parm_Acc ,float parm_Target_Speed,float parm_Dead_Zone , float parm_Ts )
{
    state->ACC = parm_Acc;
    state->Target_Speed = parm_Target_Speed;
    state->Dead_Zone = parm_Dead_Zone;
    state->Ts = parm_Ts;

}

/**
 * @brief VF强拖执行函数
 * @param[in] VF参数结构体，float指针
 * @param[out] 更新后VF参数结构体，更新的float指针
 * @param[inout] 输入输出参数
 * @return 无
 * @note VF开环不读取实际电机转速和位置，直接软件设定幅值V和频率f，积分计算模拟旋转后的角度变换，通过SVPWM算法得到相电压
 * 将求解的实际速度给到传入的float指针
 * 
 */
void VFControl_Run(VFControl_TypeDef * state, float * Targer_Speed_FOC)
{
    /*计算速度误差*/
    state->Speed_Error = state->Target_Speed - state->Speed_Integrator;
    /*对速度误差进行死区判断*/
    if(fabsf(state->Speed_Error) <= state->Dead_Zone)
     {
        state->Speed_Error_Dead = 0.0;
     }
    else 
     {
        state->Speed_Error_Dead = state->Speed_Error;
     }

    /*加速度方向判断*/
    if(state->Speed_Error_Dead > 0.0f)
     {
        state->Acc_Dir = 1;
     }
    else if(state->Speed_Error_Dead < 0.0f)
     {
        state->Acc_Dir = -1;
     }
    else
     {
        state->Acc_Dir = 0.0f;
     }
     /*计算速度*/
     state->Speed_Integrator += state->ACC * state->Acc_Dir * state->Ts;
     /*计算取模前角度*/
     state->Theta_Integrator +=  state->Speed_Integrator * state->Ts;
     /*计算取模的角度*/
     state->Theta = fmod(state->Theta_Integrator,_2PI);
  
     if(state->Theta < 0)
     {
        state->Theta += _2PI;
     }
     state->Speed = state->Speed_Integrator;
     /*提供对外接口，将VF强拖求解的速度，用于FOC计算*/
     * Targer_Speed_FOC = state->Theta;

     
    
}
