#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#include <stdint.h>
#include "Motor_Config.h"
#include "adc.h"


/*=========================== 宏定义 ==============================*/
typedef struct
{
    /*硬件电路参数*/
    float       Rs;             //采样电阻阻值（单位Ω）
    float       Gain;           //差分运放增益大小,根据实际电路计算
    float       Vref;           //ADC参考电压，一般为3.3V
    
    /*ADC零点校准值*/
    uint16_t    zero_iu;        //U相电流零点ADC值
    uint16_t    zero_iv;        //V相电流零点ADC值
    uint16_t    zero_iw;        //W相电流零点ADC值

    /*实时采样值*/
    uint16_t    raw_iu;         // U相原始ADC值
    uint16_t    raw_iv;         // V相原始ADC值
    uint16_t    raw_iw;         // W相原始ADC值

    /*经过零点校准后的实际电流*/
    float       iu;             // 换算后实际电流，单位A
    float       iv;
    float       iw;
    int     *Sector_ptr;        //保存FOC计算中扇区参数地址
    /*ADC采集状态*/
    uint8_t Zero_Calibrated;    //零点校准完成表示

} ADCSamp_TypeDef;


/*=========================== 函数声明 ==============================*/
void ADC_SampInit(ADCSamp_TypeDef * adc ,int *sector_ptr ,float rs_t, float gain_t, float vref_t );
void ADC_ZeroCalibrate_Run(ADCSamp_TypeDef * adc);
void ADC_RealData_Transf(ADCSamp_TypeDef * adc);


#endif /* __BSP_ADC_H__ */
