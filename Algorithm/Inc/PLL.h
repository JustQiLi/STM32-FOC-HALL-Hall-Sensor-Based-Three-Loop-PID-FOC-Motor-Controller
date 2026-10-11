/**
  * @file    PLL.h
  * @brief   锁相环（PLL）角度/角速度观测器接口
  * @note    借鉴自开源库 SguanFOC_Library（有感FOC库）的 Sguan_PLL：
  *          仓库地址 https://github.com/Sguan-ZhouQing/SguanFOC_Library
  *          用法：PLL_SetParm() 设定 T/Kp/Ki → PLL_Init() 预计算离散化系数 →
  *          每拍（FOC 中断）PLL_Loop(pll, 霍尔机械角) 取 OutRe/OutWe。
  */
#ifndef __PLL_H__
#define __PLL_H__

#include <stdint.h>
#include "Motor_Config.h"
/*=========================== 锁相环结构体 ==============================*/
typedef struct
{
    /* 输入 */
    float Error;        /* 角度误差（输入） */

    /* 输出 */
    float OutWe;        /* 角速度输出（rad/s） */
    float OutRe;        /* 角度输出（rad） */

    /* 历史状态 */
    float i;            /* 上次误差 */
    float Xo;           /* 上次PI输出 */
    float Yo;           /* 上次积分输出 */

    /* 参数 */
    float T;            /* 运算周期（秒） */
    float Kp;           /* 比例增益 */
    float Ki;           /* 积分增益 */

    /* 中间系数（PLL_Init 时算好） */
    float X_num[2];    /* PI 离散化系数（双线性变换） */
    float Y_num;        /* 积分器离散化系数（双线性变换） */
}PLLParm_TypeDef;

/*=========================== 函数声明 ==============================*/
void PLL_Init(PLLParm_TypeDef *pll);
void PLL_SetParm(PLLParm_TypeDef * pll,float t , float kp ,float ki );
void PLL_Loop(PLLParm_TypeDef *pll, float theta_hall);

#endif /* __PLL_H__ */
