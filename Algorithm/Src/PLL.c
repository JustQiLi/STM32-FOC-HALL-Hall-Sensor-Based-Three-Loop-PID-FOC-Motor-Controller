/**
  * @file    PLL.c
  * @brief   锁相环（PLL）角度/角速度观测器
  * @note    本文件借鉴自开源库 SguanFOC_Library 的"有感FOC库"（作者：星必尘Sguan）：
  *          仓库地址 https://github.com/Sguan-ZhouQing/SguanFOC_Library
  *          在 Sguan_PLL.c 基础上按本工程结构（PLLParm_TypeDef、系数预计算）改写并补充注释。
  *          本工程把它当作"转速观测器"使用：输入带台阶跳变的霍尔机械角（每 60°电
  *          跨一个边沿），输出连续平滑的角度/角速度，不参与 FOC 电角度解算
  *          （见 Users/Src/StateMachine.c 的 Run_UpdateObserver）。
  */
#include "PLL.h"
#include "Motor_Config.h"

/**
  * @brief  锁相环初始化
  * @param[in] pll PLL结构体指针
  * @note   根据 Kp/Ki/T 预计算双线性变换离散化系数，并清零全部状态
  */
void PLL_Init(PLLParm_TypeDef *pll)
{
    /* 双线性变换（Tustin）离散化 PI 控制器：
     * 连续域 C(s) = Kp + Ki/s，代入 s = (2/T)·(z-1)/(z+1)，得
     *   C(z) = (X0 + X1·z^-1) / (1 + z^-1)
     *   X0 = (2Kp + Ki·T)/2，X1 = (-2Kp + Ki·T)/2
     * 差分方程：u[k] = X0·e[k] + X1·e[k-1] + u[k-1]
     * （与 PLL_Loop 第 2 步的递推一一对应，故系数在初始化时算好、运行期零开销） */
    float temp0 = pll->T * pll->Ki;
    pll->X_num[0] = (2.0f * pll->Kp + temp0) / 2.0f;
    pll->X_num[1] = (-2.0f * pll->Kp + temp0) / 2.0f;

    /* 双线性变换离散化 积分器：
     * 连续域 I(s) = 1/s，Tustin 变换后
     *   y[k] = y[k-1] + (T/2)·(u[k] + u[k-1])，系数 Y_num = T/2
     * （与 PLL_Loop 第 3 步的递推对应；输出即锁相环的角度输出） */
    pll->Y_num = pll->T / 2.0f;

    /* 清零所有状态 */
    pll->i      = 0.0f;
    pll->Xo     = 0.0f;
    pll->Yo     = 0.0f;
    pll->Error  = 0.0f;
    pll->OutWe  = 0.0f;
    pll->OutRe  = 0.0f;
}
/**
  * @brief  设定锁相环参数：T Kp Ki
  * @param[in] pll PLL结构体指针
  * @note   参数变化后需重新调用 PLL_Init() 才会按新参数更新离散化系数
  */
void PLL_SetParm(PLLParm_TypeDef * pll,float t , float kp ,float ki )
{
    pll->T  = t;
    pll->Kp = kp;
    pll->Ki = ki;

}
/**
  * @brief  锁相环运算（在 FOC 中断里每拍调用）
  * @param[in] pll PLL结构体指针
  * @param[in] theta_hall 霍尔估算出的机械角（rad，含换相台阶跳变）
  * @note   输出 OutRe（平滑机械角）与 OutWe（平滑机械角速度）；
  *         环路 = 鉴相器(PI) + 积分器，二阶 PLL 结构，带宽由 Kp/Ki 决定。
  */
void PLL_Loop(PLLParm_TypeDef *pll, float theta_hall)
{
    /* 1. 鉴相：算角度误差 */
    pll->Error = theta_hall - pll->OutRe;

    /* 角度误差归一化到 [-π, π]：保证跨越 0/2π 边界时误差走最短路径，
     * 不会因为"角度从 359° 变到 1°"产生 -358° 的假大误差 */
    /* 有界归一化：输入是累计机械角（HALL_GetMechAngle 只增不减），误差绝对值
     * 可达上千上万 rad。原实现只用下面的 while 逐次减 2π，每拍要迭代 θ/(2π)
     * 次（即自标定以来累计圈数），导致中断体耗时随累计圈数线性增长（实测
     * 15.7us 增到 98us），最终超过 50us 采样周期，丢失 ADC 注入触发、中断率掉半，
     * 所有按 ts 折算的量差 T_real/ts 倍。先用一次乘减把整圈部分消掉，
     * 之后 while 最多迭代 1~2 次，耗时与累计圈数无关（数学上等价）。 */
    pll->Error -= _2PI * (float)(int32_t)(pll->Error * (1.0f / _2PI));
    while (pll->Error > _PI)  pll->Error -= _2PI;
    while (pll->Error < -_PI) pll->Error += _2PI;

    /* 2. PI 控制器：输出角速度
     * OutWe = X0·Error + X1·i + Xo，其中 i=上次误差、Xo=上次输出（见 PLL_Init 注释） */
    pll->OutWe = pll->X_num[0] * pll->Error
               + pll->X_num[1] * pll->i
               + pll->Xo;

    /* 3. 积分器：输出角度
     * OutRe = (T/2)·(OutWe + Xo) + Yo，其中 Yo=上次角度输出 */
    pll->OutRe = pll->Y_num * pll->OutWe
               + pll->Y_num * pll->Xo
               + pll->Yo;

    /* 4. 角度归一化到 [0, 2π) */
    while (pll->OutRe >= _2PI) pll->OutRe -= _2PI;
    while (pll->OutRe < 0.0f)  pll->OutRe += _2PI;

    /* 5. 更新历史值，供下一拍递推 */
    pll->i  = pll->Error;
    pll->Xo = pll->OutWe;
    pll->Yo = pll->OutRe;
}
