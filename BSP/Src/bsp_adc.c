#include "bsp_adc.h"


/**
 * @brief ADC参数初始化
 * @param[in] ADC参数结构体、FOC扇区参数地址，采样电阻阻值、运放增益、MCU参考电压
 * @param[out] 更新后的ADC参数结构体
 * @param[inout] 输入输出参数
 * @return 返回值
 * @note 对ADC的参数初始化，其中rs 和gain 需要根据实际硬件电路参数计算
 */
void ADC_SampInit(ADCSamp_TypeDef * adc ,int  *sector_ptr ,float rs_t, float gain_t, float vref_t )
{

    
    adc->Sector_ptr = sector_ptr;
    adc->Rs = rs_t;
    adc->Gain = gain_t;
    adc->Vref = vref_t;
    /*校准标志位置零*/
    adc->Zero_Calibrated = 0;
    /*默认中点，校准后会覆盖*/
    adc->zero_iu = (ADC_FULL_SCALE_12BIT+1.0f) * 0.5f; 
    adc->zero_iv = (ADC_FULL_SCALE_12BIT+1.0f) * 0.5f;
    adc->zero_iw = (ADC_FULL_SCALE_12BIT+1.0f) * 0.5f;
    /*开启ADC中断*/
    __HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_JEOC);
    HAL_ADCEx_InjectedStart(&hadc1);
}
/**
 * @brief  均值法零点校准
 * @param[in] ADC参数结构体
 * @param[out] 校准数据后的ADC结构体参数
 * @param[inout] 输入输出参数
 * @return 返回值
 * @note 电机静止、不转的时候调用，连续采样1000次取平均作为零点
 */
void ADC_ZeroCalibrate_Run(ADCSamp_TypeDef * adc)
{
    uint32_t sum_iu = 0, sum_iv = 0, sum_iw = 0;
    const uint16_t calib_cnt = 1000;
    uint32_t timeout;

    /* 校准期间关闭 JEOC 中断，防止进入回调函数读取ADC值，干扰校准 */
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);

    /* 丢弃前 10 次转换，等注入通道稳定 */
    for(uint16_t i = 0; i < 10; i++)
    {   
        timeout = 0xFFFFFF;
        while((!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC)) && (--timeout));
        /*手动清楚JEOC标志位（硬件只在读取ADC值时自动清零）*/
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    }

    for(uint16_t i = 0; i < calib_cnt; i++)
    {
        /* 等本次注入转换完成再读，避免读到复位值 0 / 旧值 */
        timeout = 0xFFFFFF;
        while((!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC)) && (--timeout));

        sum_iu += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        sum_iv += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        sum_iw += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);  
    }

    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);

    adc->zero_iu = (uint16_t)(sum_iu / calib_cnt);
    adc->zero_iv = (uint16_t)(sum_iv / calib_cnt);
    adc->zero_iw = (uint16_t)(sum_iw / calib_cnt);
    adc->Zero_Calibrated = 1;
}
/**
 * @brief 真实相电流计算函数
 * @param[in] ADC参数结构体
 * @param[out] ADC转换后的ADC参数结构体
 * @param[inout] 输入输出参数
 * @return 返回值
 * @note 对零点校准后的ADC值进行转换，得到真实电路中的三相电流Iu Iv Iw
 */
void ADC_RealData_Transf(ADCSamp_TypeDef * adc)
{   
    /*读3路ADC注入通道原始值*/ 
    adc->raw_iu = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    adc->raw_iv = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    adc->raw_iw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
    
    /*若没校准则不换算，防止错误值*/
    if(!adc->Zero_Calibrated) return;

    /*换算实际三相电流 Iu Iv Iw*/
    float k = adc->Vref / (ADC_FULL_SCALE_12BIT  * adc->Gain * adc->Rs);
    adc->iu = -(adc->raw_iu - adc->zero_iu) * k;
    adc->iv = -(adc->raw_iv - adc->zero_iv) * k;
    adc->iw = -(adc->raw_iw - adc->zero_iw) * k;

/*1=启用扇区无效相修正（有效矢量采样时开），0=关闭（零矢量采样时关）*/
#define ENABLE_SECTOR_CURR_CORRECT  0
#if ENABLE_SECTOR_CURR_CORRECT
    /*根据扇区丢弃无效相，补算正确电流（PWM模式1逻辑）*/
    switch(*adc->Sector_ptr)
    {
        case 1:
        case 2:
            // 丢弃iw相，保留iu/iv，补算iw
            adc->iw = -(adc->iu + adc->iv);
            break;
        case 3:
        case 4:
            // 丢弃iu相，保留iv/iw，补算iu
            adc->iu = -(adc->iv + adc->iw);
            break;
        case 5:
        case 6:
            // 丢弃iv相，保留iu/iw，补算iv
            adc->iv = -(adc->iu + adc->iw);
            break;
    }
#endif
    
}
