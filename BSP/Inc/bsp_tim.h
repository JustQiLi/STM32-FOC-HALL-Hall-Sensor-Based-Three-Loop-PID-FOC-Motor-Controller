#ifndef __BSP_TIM_H__
#define __BSP_TIM_H__
#include "stdint.h"
#include "tim.h"

/*=========================== º¯ÊýÉùÃ÷ ==============================*/
void Motor_BspInit(void);
void Motor_EnablePWM(void);
void Motor_DisablePWM(void);
void Motor_SetPWMCompare(uint16_t Compare1,uint16_t Compare2,uint16_t Compare3);
#endif /* __BSP_TIM_H__ */