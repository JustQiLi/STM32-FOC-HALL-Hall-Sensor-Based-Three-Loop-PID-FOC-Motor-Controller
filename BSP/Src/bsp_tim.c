/*
 * 文件名 
 * 作者 
 * 版本 V1.0
 * 创建日期 
 * 功能 本文件功能说明
 * 修改记录 日期 | 作者 | 版本 | 修改描述
 */
#include "bsp_tim.h"


/**
 * @brief 电机驱动板定时器初始化
 * @param[in] 无
 * @param[out] 无
 * @param[inout] 输入输出参数
 * @return 无
 * @note 备注说明
 */
void Motor_BspInit(void)
{
    /*使能三路PWMN*/
    Motor_EnablePWM();
    /*初始化定时器中断*/
    HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_4);
    /*初始化确保占空比设置为0*/
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);

}
/**
 * @brief 控制电机逆变器三路互补PWM输出使能
 * @param[in] 无
 * @param[out] 无
 * @param[inout] 输入输出参数
 * @return 无
 * @note 备注说明
 */
void Motor_EnablePWM(void)
{

  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_3);

}
/**
 * @brief 控制电机逆变器三路互补PWM输出失能
 * @param[in] 无
 * @param[out] 无
 * @param[inout] 输入输出参数
 * @return 无
 * @note 备注说明
 */
void Motor_DisablePWM(void)
{
  HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_3);
}

/**
 * @brief 设置电机逆变器三路PWM占空比
 * @param[in] Compare1:通道1占空比 Compare2:通道2占空比 Compare3:通道3占空比
 * @param[out] 无
 * @param[inout] 输入输出参数
 * @return 无
 * @note 设置CCR比较寄存器的值来控制PWM占空比
 */
void Motor_SetPWMCompare(uint16_t Compare1,uint16_t Compare2,uint16_t Compare3)
{
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,Compare1);
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,Compare2);
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,Compare3);
}   