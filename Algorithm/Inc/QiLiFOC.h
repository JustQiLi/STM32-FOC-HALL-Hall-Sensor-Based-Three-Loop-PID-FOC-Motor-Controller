/*
 * 文件名 
 * 作者 
 * 版本 V1.0
 * 创建日期 
 * 功能 本文件功能说明
 * 修改记录 日期 | 作者 | 版本 | 修改描述
 */
#ifndef __QILIFOC_H_
#define __QILIFOC_H_

#include "math.h"
#include "tim.h"

/*=========================== 宏定义 ==============================*/
#define EPS 1e-3f   // 扇区滞环阈值
/*=========================== 结构体 ==============================*/
/**
 * @brief FOC参数结构体定义
 **/
typedef struct
{   
    float Udc;     //实际电路母线电压
    float ARR;     //PWM输出定时器ARR值
    float Ts ;     //FOC周期
    float Vd;      //d轴电压
    float Vq;      //q轴电压

    float Theta;   //电机电角度
    float Valpha;  //alpha电压
    float Vbeta;   //beta电压
    float Ialpha;  //alpha电流
    float Ibeta;   //beta电流

    float Id;      //d轴电流
    float Iq;      //q轴电流
    
    float Vu;      //三相电压u
    float Vv;      //三相电压v
    float Vw;      //三相电压w

    float Iu;      //三相电流u
    float Iv;      //三相电流v
    float Iw;      //三相电流w

    float Vu_mod;  //u相调制电压
    float Vv_mod;  //v相调制电压
    float Vw_mod;  //w相调制电压

    float V_u;     //u端电压
    float V_v;     //v端电压
    float V_w;     //w端电压
    
    int CCR_U;     //u端占空比
    int CCR_V;     //v端占空比
    int CCR_W;     //w端占空比

    /*七段式SVPWM*/
    int lastA,lastB,lastC;    //存储用于扇区判断的上一次编码值
    int   Sector;             //扇区号(1~6)
    float T1;                 //第一个矢量归一化作用时间
    float T2;                 //第二个矢量归一化作用时间
    float X,Y,Z;              //中间计算量（提高可读性）


}FOCParm_TypeDef;
/*=========================== 函数声明 ==============================*/
void Clarke_Transf(FOCParm_TypeDef * state);
void Park_Transf(FOCParm_TypeDef * state);
void Rev_Park_Transf(FOCParm_TypeDef *  state);
void Rev_Clarke_Transf(FOCParm_TypeDef * state);
void Zero_SqVInjection(FOCParm_TypeDef * state);
void Sector_Calculate(FOCParm_TypeDef * state);
void T1T2_Calculate(FOCParm_TypeDef * state);
void T1T2_CCRTransf(FOCParm_TypeDef * state);
void FOC_Algorithm_ZSq(FOCParm_TypeDef * state);
void FOC_Algorithm_SVPWM(FOCParm_TypeDef * state);
void SetParm_Udc_ARR(FOCParm_TypeDef* state, float Udc, float ARR);
void FOC_SinCos(float theta, float *s, float *c);              /* 电角度sin/cos值缓存(等价优化) */
/*======================= 自定义数学封装函数 ==========================*/
static  float  GetMaxFOCParm_Thr(float parm1,float parm2,float parm3);
static float  GetMinFOCParm_Thr(float parm1,float parm2 ,float parm3);
static float _constrain(float value, float min, float max);

/*=========================== 外部声明 ==============================*/



#endif/* __QILIFOC_H__ */