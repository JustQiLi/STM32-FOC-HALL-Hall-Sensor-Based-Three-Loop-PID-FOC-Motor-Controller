/*
 * 文件名 
 * 作者 
 * 版本 V1.0
 * 创建日期 
 * 功能 本文件功能说明
 * 修改记录 日期 | 作者 | 版本 | 修改描述
 */
#include "QiLiFOC.h"
#include "Motor_Config.h"
#include "StateMachine.h"

/*==================== 电角度三角函数值缓存（纯等价优化） ====================*/
/* 原本每个FOC周期按同一个电角度反复调用cosf/sinf：Park 4次 + Rev_Park 4次 + 死区补偿 6次 = 14次。 */
/* 本函数按"角度值"缓存：角度一变立刻重算，不保留陈旧状态；数值结果与逐次调用cosf/sinf完全一致。 */
static float   s_trig_theta = 0.0f;
static float   s_trig_sin   = 0.0f;
static float   s_trig_cos   = 0.0f;
static uint8_t s_trig_valid = 0U;

void FOC_SinCos(float theta, float *s, float *c)
{
    if ((s_trig_valid == 0U) || (theta != s_trig_theta))
    {
        s_trig_sin   = sinf(theta);
        s_trig_cos   = cosf(theta);
        s_trig_theta = theta;
        s_trig_valid = 1U;
    }
    *s = s_trig_sin;
    *c = s_trig_cos;
}


/**
 * @brief Clarke变换：三相电流 → αβ轴电流
 * @param[in] FOC参数结构体
 * @param[out] 更新ialpha/ibeta后的FOC参数结构体
 * @note 三相三线制，iu+iv+iw=0，只用两相即可
 */
void Clarke_Transf(FOCParm_TypeDef * state)
{
    state->Ialpha = state->Iu;
    state->Ibeta = (state->Iu + 2.0f * state->Iv) / SQRT3;
}

/**
 * @brief Park变换：αβ轴电流 → dq轴电流
 * @param[in] FOC参数结构体（包含电角度Theta）
 * @param[out] 更新id/iq后的FOC参数结构体
 * @note 和现有Rev_Park互为逆变换，角度方向一致
 */
void Park_Transf(FOCParm_TypeDef * state)
{
    float s, c;
    FOC_SinCos(state->Theta, &s, &c);
    state->Id =  state->Ialpha * c + state->Ibeta * s;
    state->Iq = -state->Ialpha * s + state->Ibeta * c;
}

/**
 * @brief 对目标Vd和Vq进行反Park变换得到Valpha和Vbeta
 * @param[in] FOC参数结构体
 * @param[out] 反Park变换后FOC参数结构体
 * @param[inout] 
 * @return 无
 * @note 备注说明
 */
void Rev_Park_Transf(FOCParm_TypeDef *  state)
{    
     float s, c;
     FOC_SinCos(state->Theta, &s, &c);
     state->Valpha = state->Vd * c - state->Vq * s;
     state->Vbeta  = state->Vd * s + state->Vq * c;
     
} 

/**
 * @brief 对Valpha和Vbeta进行反Clarke变换得到Vu，Vv,Vw三相电压
 * @param[in] FOC参数结构体
 * @param[out] 反Clarke变换后FOC参数结构体
 * @param[inout] 输入输出参数
 * @return 返回值
 * @note 备注说明
 */
void Rev_Clarke_Transf(FOCParm_TypeDef * state)
{
    state->Vu = state->Valpha;
    state->Vv = -0.5f * state->Valpha + SQRT3/2.0f * state->Vbeta;
    state->Vw = -0.5f * state->Valpha - SQRT3/2.0f * state->Vbeta;
    
}

/**
 * @brief 基于零序注入法的调制算法
 * @param[in] FOC参数结构体
 * @param[out] 零序注入法后FOC参数结构体
 * @param[inout] 输入输出参数
 * @return 返回值
 * @note 和SVPWM结果等价
 */
void Zero_SqVInjection(FOCParm_TypeDef * state )
{   
    /*计算零序偏移量（共模电压）*/
   float V0 =  -0.5f*(GetMaxFOCParm_Thr(state->Vu,state->Vv,state->Vw)
                   + GetMinFOCParm_Thr(state->Vu,state->Vv,state->Vw));

    /*获得调制后三相电压（马鞍波）*/
   state->Vu_mod = state->Vu + V0;
   state->Vv_mod = state->Vv + V0;
   state->Vw_mod = state->Vw + V0;

   /*三相电压 计算为 三端电压*/
   state->V_u = state->Vu_mod + 0.5f * state->Udc;
   state->V_v = state->Vv_mod + 0.5f * state->Udc;
   state->V_w = state->Vw_mod + 0.5f * state->Udc;

   /*对三端电压进行限幅处理*/
   state->V_u = _constrain(state->V_u,0.0f,state->Udc);
   state->V_v = _constrain(state->V_v,0.0f,state->Udc);
   state->V_w = _constrain(state->V_w,0.0f,state->Udc);

   /*计算实际控制逆变器导通互补PWM信号的三路CCR*/
    state->CCR_U = (int)(state->V_u / state->Udc * state->ARR);
    state->CCR_V = (int)(state->V_v / state->Udc * state->ARR); 
    state->CCR_W = (int)(state->V_w / state->Udc * state->ARR);

}
/**
 * @brief 扇区判断函数(Sector:1~6)
 * @param[in] FOC参数结构体指针
 * @param[out] 更新参数扇区的结构体指针
 * @param[inout] 输入输出参数
 * @return 无
 * @note 使用三位bit移位得到1~6的数字，分别对应六个扇区
 */
void Sector_Calculate(FOCParm_TypeDef * state)
{
    float valA = state->Vbeta;
    float valB = SQRT3 * state->Valpha - state->Vbeta;
    float valC = -SQRT3 * state->Valpha - state->Vbeta;
    /*在[-EPS,EPS]区间：保持上一拍lastA，不做任何操作*/ 
    /*A 双阈值滞环*/ 
    if(valA > EPS)
    {
        state->lastA = 1;
    }
    else if(valA < -EPS)
    {
        state->lastA = 0;
    }
    /*B 双阈值滞环*/ 
    if(valB > EPS)
    {
        state->lastB = 1;
    }
    else if(valB < -EPS)
    {
        state->lastB = 0;
    }

    /*C 双阈值滞环*/ 
    if(valC > EPS)
    {
        state->lastC = 1;
    }
    else if(valC < -EPS)
    {
        state->lastC = 0;
    }

    int A = state->lastA;
    int B = state->lastB;
    int C = state->lastC;
    /*三位bit移位拼成一个数字，范围1~6*/
    int N = 4*C + 2*B + A;
    /*由查表法，根据N的大小判定所在扇区*/
    /*N=3->扇区I
      N=1->扇区II
      N=5->扇区III
      N=4->扇区IV
      N=6->扇区V
      N=2->扇区VI
    */
    switch(N)
    {
        case 1: state->Sector = 2;break;
        case 2: state->Sector = 6;break;
        case 3: state->Sector = 1;break;
        case 4: state->Sector = 4;break;
        case 5: state->Sector = 3;break;
        case 6: state->Sector = 5;break;
        default: state->Sector = 1; break;
    }
}
/**
 * @brief 计算该扇区相邻有效矢量的作用时长T1/T2
 * @param[in] FOC参数结构体指针
 * @param[out] 更新参数后的FOC参数结构体指针
 * @param[inout] 输入输出参数
 * @return 无
 * @note T1/T2投影alpha/beta轴计算，T0=Ts-T1-T2
 */
void T1T2_Calculate(FOCParm_TypeDef * state)
{   
    /*扇区T计算系数K*/
    float K = SQRT3*ts/state->Udc;
    
    /*计算中间变量X,Y,Z提高可读性*/
    state->X = K * state->Vbeta;
    state->Y = K * (SQRT3/2*state->Valpha + 0.5f * state->Vbeta);
    state->Z = K * (-SQRT3/2*state->Valpha+ 0.5f * state->Vbeta);

    /*进行不同扇区分支*/
    switch(state->Sector)
    {
        case 1:state->T1 = -state->Z;  state->T2 =  state->X;break;
        case 2:state->T1 =  state->Z;  state->T2 =  state->Y;break;
        case 3:state->T1 =  state->X;  state->T2 = -state->Y;break;
        case 4:state->T1 = -state->X;  state->T2 =  state->Z;break;
        case 5:state->T1 = -state->Y;  state->T2 = -state->Z;break;
        case 6:state->T1 =  state->Y;  state->T2 = -state->X;break;
    }
    /*T1/T2 负数钳位，防止计算出小于0的情况*/
    state->T1 = state->T1 < 0.0f ? 0.0f : state->T1;
    state->T2 = state->T2 < 0.0f ? 0.0f : state->T2;
    /*过调制处理：T1+T2>1时等比例缩小，此时T1+T2=1*/
    if((state->T1 + state->T2)/ts > 1.0f)
    {
        float ratio = ts/(state->T1 + state->T2);
        /*T1/T2不变，等比例缩小，不改变合成矢量方向*/
        state->T1 *= ratio;
        state->T2 *= ratio;
    }
    
    
}
/**
 * @brief 由T1T2求解出三相CCR
 * @param[in] FOC参数结构体指针
 * @param[out] 更新CCR后的FOC参数结构体指针
 * @param[inout] 输入输出参数
 * @return 无
 * @note 备注说明
 */
void T1T2_CCRTransf(FOCParm_TypeDef * state)
{   /*开通时间最小相时间为0.5 T0*/
    float Tmin = 0.5f * (1.0f-state->T1/ts-state->T2/ts);
    float Tmid = Tmin + state->T2/ts;
    float Tmax = Tmin + state->T1/ts + state->T2/ts;
    /*三相对应的归一化时间(0~1)*/
    float Ta,Tb,Tc;
    /*占一周期时间最小相、中间相、最大相随扇区而变化*/
    switch(state->Sector)
    {   
        case 1: Tc = Tmin; Tb = Tmid; Ta = Tmax; break;
        case 2: Tc = Tmin; Tb = Tmax; Ta = Tmid; break;
        case 3: Tc = Tmid; Tb = Tmax; Ta = Tmin; break;
        case 4: Tc = Tmax; Tb = Tmid; Ta = Tmin; break;
        case 5: Tc = Tmax; Tb = Tmin; Ta = Tmid; break;
        case 6: Tc = Tmid; Tb = Tmin; Ta = Tmax; break;
        default:Ta=Tb=Tc=0.5f; break;
    }
    /*对占空比Tx进行限幅处理*/
    Ta = _constrain(Ta,0.0f,1.0f);
    Tb = _constrain(Tb,0.0f,1.0f);
    Tc = _constrain(Tc,0.0f,1.0f);
    
    /*A,B,C三相分别对应U,V,W三相*/
    /*PWM波形应该从零矢量开始，即CNT<CCR时，PWM输出为0*/
    /*由于选择PWM模式为PWM1，故调整占空比*/
    state->CCR_U = (int)(Ta * state->ARR);
    state->CCR_V = (int)(Tb * state->ARR);
    state->CCR_W = (int)(Tc * state->ARR);

}

/**
 * @brief FOC算法主函数(零序注入法)
 * @param[in] FOC参数结构体
 * @param[out] 无
 * @param[inout] 输入输出参数
 * @return 返回值
 * @note 包括反Park变换，反Clarke变换，零序注入法
 */
void FOC_Algorithm_ZSq(FOCParm_TypeDef * state)
{
   /*反Park变换*/
   Rev_Park_Transf(state);
   /*反Clarke变换*/
   Rev_Clarke_Transf(state);
    /*零序注入法*/
   Zero_SqVInjection(state);

}
/**
 * @brief FOC算法主函数(七段式SVPWM)
 * @param[in] FOC参数结构体
 * @param[out] 无
 * @param[inout] 输入输出参数
 * @return 返回值
 * @note 包括反Park变换，反Clarke变换，七段式SVPWM
 */
void FOC_Algorithm_SVPWM(FOCParm_TypeDef * state)
{
   /*反Park变换*/
   Rev_Park_Transf(state);
   /*反Clarke变换*/
   Rev_Clarke_Transf(state);
    /*七段式SVPWM*/
    /*获取扇区*/
    Sector_Calculate(state);
    /*计算T1/T2*/
    T1T2_Calculate(state);
    /*计算CCR*/
    T1T2_CCRTransf(state);

}
/**
 * @brief 初始化FOC参数结构体中的Udc和ARR值
 * @param[in] FOC参数结构体 ，Udc值，ARR值
 * @param[out] 初始化后的结构体
 * @param[inout] 输入输出参数
 * @return 返回值
 * @note 外部接口函数
 */
void SetParm_Udc_ARR(FOCParm_TypeDef* state, float Udc, float ARR)
{
    state->Udc = Udc;
    state->ARR = ARR;
}



/**
 * @brief 寻找三浮点数中最大值
 * @param[in] 用作比较的三个浮点数
 * @param[out] 三个浮点数中最大的参数
 * @param[inout] 输出输出参数
 * @return 最大值
 * @note 数学计算封装
 */
static float  GetMaxFOCParm_Thr(float parm1,float parm2,float parm3)
{
    return (parm1 > parm2 ? (parm1>parm3?parm1:parm3) : (parm2>parm3?parm2:parm3) );
}

/**
 * @brief 寻找三浮点数中最小值
 * @param[in] 用作比较的三个浮点数
 * @param[out] 三个浮点数中最小的参数
 * @param[inout] 输入输出参数
 * @return 最小值
 * @note 数学计算封装
 */
static float  GetMinFOCParm_Thr(float parm1,float parm2 ,float parm3)
{
    return (parm1 < parm2 ? (parm1<parm3?parm1:parm3) : (parm2<parm3?parm2:parm3) );
}
/**
 * @brief 限幅函数，把输入value限制在 [min, max]之间
 * @param value: 待限制的浮点数值
 * @param min: 下限
 * @param max: 上限
 * @retval 限幅之后的值
 */
static float _constrain(float value, float min, float max)
{
    if(value < min)
        return min;
    else if(value > max)
        return max;
    else
        return value;
}

