/**
  * @file    filter.h
  * @brief   常用软件滤波算法库（头文件）
  * @note    本库汇集嵌入式/单片机中常用的经典软件滤波算法，纯 C 实现，
  *          不依赖任何硬件平台，可直接移植到 51 / STM32 / GD32 等工程。
  *          每个“有状态”滤波器 = 结构体 + 初始化函数 + 处理函数；
  *          对每个通道定义一个结构体，先 init 再循环调用 process。
  */

#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif





/* 滤波数值类型：默认 float，如需更高精度可改为 double，如需定点可改为 int */
typedef float filter_t;

/* ========================================================================== */
/*  1. 限幅滤波法（程序判断滤波）                                             */
/* ========================================================================== */
/* 原理：设定两次采样允许的最大偏差 max_delta。本次值与上次值之差 ≤ max_delta  */
/*       则认为有效；否则认为有干扰，用上次值代替。                           */
/* 特点：能克服偶然脉冲干扰，但无法抑制周期性干扰，平滑度较差。               */
typedef struct {
    filter_t last;      /* 上一次有效输出值 */
    filter_t max_delta; /* 允许的最大偏差   */
    bool     has_last;  /* 是否已有上次值   */
} filter_limit_t;

/**
  * @brief  初始化限幅滤波器
  * @param[in]  f 滤波器句柄
  * @param[in]  max_delta 允许的最大偏差
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  无
  */
void filter_limit_init(filter_limit_t *f, filter_t max_delta);

/**
  * @brief  执行一次限幅滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态（last/has_last）
  * @return  滤波后的有效值
  * @note  偏差超限时沿用上次有效值
  */
filter_t filter_limit(filter_limit_t *f, filter_t v);

/* ========================================================================== */
/*  2. 中位值滤波法                                                           */
/* ========================================================================== */
/* 原理：连续采样 N 次（N 取奇数），排序后取中间值作为有效值。                 */
/* 特点：能克服偶然波动干扰，适合温度、液位等缓慢变化参数；                   */
/*       对流量、速度等快速变化参数不宜。                                     */
#ifndef FILTER_MEDIAN_WINDOW
#define FILTER_MEDIAN_WINDOW 5   /* 窗口大小，必须为奇数 */
#endif

typedef struct {
    filter_t buf[FILTER_MEDIAN_WINDOW];  /* 采样窗口       */
    filter_t tmp[FILTER_MEDIAN_WINDOW];  /* 排序用临时数组 */
    uint8_t  idx;                        /* 当前写入下标   */
    uint8_t  cnt;                        /* 已采样个数     */
} filter_median_t;

/**
  * @brief  初始化中位值滤波器
  * @param[in]  f 滤波器句柄
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  无
  */
void filter_median_init(filter_median_t *f);

/**
  * @brief  执行一次中位值滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部窗口与计数
  * @return  窗口内排序后的中间值
  * @note  窗口未满时对已采样部分取中值
  */
filter_t filter_median(filter_median_t *f, filter_t v);

/* ========================================================================== */
/*  3. 算术平均滤波法                                                         */
/* ========================================================================== */
/* 原理：对一段连续采样值求算术平均，N 越大平滑度越高、灵敏度越低。           */
/* 特点：适用于有随机干扰、信号在某一数值附近上下波动的场合。                 */

/**
  * @brief  对一段采样缓冲求算术平均
  * @param[in]  buf 采样缓冲首地址
  * @param[in]  n 样本个数
  * @param[out] 无
  * @param[inout] 无
  * @return  算术平均值；buf 为空或 n==0 时返回 0
  * @note  无状态，可在任意时刻对任意缓冲调用
  */
filter_t filter_average(const filter_t *buf, uint16_t n);

/* ========================================================================== */
/*  4. 递推平均滤波法（滑动平均）                                             */
/* ========================================================================== */
/* 原理：固定长度队列，新值入队尾、旧值出队首，对队列取平均。                 */
/* 特点：对周期性干扰抑制好、平滑度高，适用于高频振荡系统；                   */
/*       但灵敏度低，对偶然脉冲干扰抑制较差。                                 */
#ifndef FILTER_MOVING_WINDOW
#define FILTER_MOVING_WINDOW 8
#endif

typedef struct {
    filter_t buf[FILTER_MOVING_WINDOW]; /* 滑动窗口     */
    filter_t sum;                       /* 窗口内元素和 */
    uint8_t  idx;                       /* 当前写入下标 */
    uint8_t  cnt;                       /* 已采样个数   */
} filter_moving_t;

/**
  * @brief  初始化滑动平均滤波器
  * @param[in]  f 滤波器句柄
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  无
  */
void filter_moving_init(filter_moving_t *f);

/**
  * @brief  执行一次滑动平均滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部窗口、和与计数
  * @return  当前窗口的滑动平均值
  * @note  窗口未满时按实际已采样个数取平均
  */
filter_t filter_moving(filter_moving_t *f, filter_t v);

/* ========================================================================== */
/*  5. 中位值平均滤波法（防脉冲干扰平均滤波）                                 */
/* ========================================================================== */
/* 原理：去掉一个最大值和一个最小值后，对剩余 N-2 个值取平均。                */
/* 特点：融合中位值与算术平均的优点，可消除脉冲干扰引起的采样偏差。           */

/**
  * @brief  对一段采样缓冲做去极值平均（中位值平均）
  * @param[in]  buf 采样缓冲首地址
  * @param[in]  n 样本个数
  * @param[out] 无
  * @param[inout] 无
  * @return  去掉最大最小值后的平均值；样本不足 3 个时退化为普通平均
  * @note  无状态，可在任意时刻对任意缓冲调用
  */
filter_t filter_median_average(const filter_t *buf, uint16_t n);

/* ========================================================================== */
/*  6. 限幅平均滤波法                                                         */
/* ========================================================================== */
/* 原理：先限幅去除明显异常值，再做滑动平均。                                 */
/* 特点：融合限幅与滑动平均优点，对偶然脉冲与周期性干扰均有较好抑制。         */
typedef struct {
    filter_limit_t  limiter;  /* 限幅部分 */
    filter_moving_t averager; /* 平均部分 */
} filter_limit_avg_t;

/**
  * @brief  初始化限幅平均滤波器
  * @param[in]  f 滤波器句柄
  * @param[in]  max_delta 限幅允许的最大偏差
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  内部依次初始化限幅器与滑动平均器
  */
void filter_limit_avg_init(filter_limit_avg_t *f, filter_t max_delta);

/**
  * @brief  执行一次限幅平均滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态
  * @return  限幅后经滑动平均的输出值
  * @note  先限幅、再滑动平均
  */
filter_t filter_limit_avg(filter_limit_avg_t *f, filter_t v);

/* ========================================================================== */
/*  7. 一阶滞后滤波法（低通滤波）                                             */
/* ========================================================================== */
/* 原理：y = a*x + (1-a)*y_last，a 为滤波系数（0<a<1），a 越小滞后越大。      */
/* 特点：对周期性干扰抑制好，适用于波动频率较高的场合；                       */
/*       但相位滞后、灵敏度低。                                               */
typedef struct {
    filter_t a;        /* 滤波系数 0<a<1 */
    filter_t y;        /* 上一次输出     */
    bool     has_y;    /* 是否已有输出   */
} filter_lag_t;

/**
  * @brief  初始化一阶滞后（低通）滤波器
  * @param[in]  f 滤波器句柄
  * @param[in]  a 滤波系数（0<a≤1）
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  系数越界时自动夹在 (0,1] 区间
  */
void filter_lag_init(filter_lag_t *f, filter_t a);

/**
  * @brief  执行一次一阶滞后（低通）滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态（y/has_y）
  * @return  滤波后的输出值
  * @note  首个样本直接作为输出
  */
filter_t filter_lag(filter_lag_t *f, filter_t v);

/* ========================================================================== */
/*  8. 加权递推平均滤波法                                                     */
/* ========================================================================== */
/* 原理：滑动平均的改进，越新的数据权重越大，以提高对当前采样值的灵敏度。     */
/* 特点：灵敏度高于普通滑动平均，适合需区分新旧数据重要性的场合。             */
#ifndef FILTER_WEIGHTED_WINDOW
#define FILTER_WEIGHTED_WINDOW 8
#endif

typedef struct {
    filter_t buf[FILTER_WEIGHTED_WINDOW];
    uint8_t  idx;
    uint8_t  cnt;
} filter_weighted_t;

/**
  * @brief  初始化加权递推平均滤波器
  * @param[in]  f 滤波器句柄
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  无
  */
void filter_weighted_init(filter_weighted_t *f);

/**
  * @brief  执行一次加权递推平均滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部窗口与计数
  * @return  加权平均后的输出值
  * @note  权重从旧到新依次为 1,2,3,...（越新权重越大）
  */
filter_t filter_weighted(filter_weighted_t *f, filter_t v);

/* ========================================================================== */
/*  9. 消抖滤波法                                                             */
/* ========================================================================== */
/* 原理：设置计数阈值 N。某值连续出现 N 次以上才认为有效并更新输出，否则维持。 */
/* 特点：对脉冲干扰抑制明显，适合按键消抖、开关量、缓变量等；                 */
/*       但可能对快速变化量响应迟钝。                                         */
typedef struct {
    filter_t last;       /* 上一次有效输出 */
    filter_t candidate;  /* 当前候选值     */
    uint8_t  same_cnt;   /* 连续出现次数   */
    uint8_t  threshold;  /* 判定阈值 N     */
    bool     has_last;
} filter_debounce_t;

/**
  * @brief  初始化消抖滤波器
  * @param[in]  f 滤波器句柄
  * @param[in]  threshold 判定阈值（连续出现次数）
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  阈值小于 1 时自动置 1
  */
void filter_debounce_init(filter_debounce_t *f, uint8_t threshold);

/**
  * @brief  执行一次消抖滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态
  * @return  消抖后的有效值（未达阈值时返回上次有效值）
  * @note  值与候选值一致则累计，否则重新计数
  */
filter_t filter_debounce(filter_debounce_t *f, filter_t v);

/* ========================================================================== */
/*  10. 限幅消抖滤波法                                                        */
/* ========================================================================== */
/* 原理：先限幅滤除大幅度异常，再消抖滤除抖动。                               */
/* 特点：继承限幅与消抖优点，抗脉冲、抗抖动能力强。                           */
typedef struct {
    filter_limit_t    limiter;  /* 限幅部分 */
    filter_debounce_t debounce; /* 消抖部分 */
} filter_limit_debounce_t;

/**
  * @brief  初始化限幅消抖滤波器
  * @param[in]  f 滤波器句柄
  * @param[in]  max_delta 限幅允许的最大偏差
  * @param[in]  threshold 消抖判定阈值
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  内部依次初始化限幅器与消抖器
  */
void filter_limit_debounce_init(filter_limit_debounce_t *f,
                                filter_t max_delta,
                                uint8_t  threshold);

/**
  * @brief  执行一次限幅消抖滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态
  * @return  限幅并消抖后的有效值
  * @note  先限幅、再消抖
  */
filter_t filter_limit_debounce(filter_limit_debounce_t *f, filter_t v);

/* ========================================================================== */
/*  11. 一维卡尔曼滤波                                                        */
/* ========================================================================== */
/* 原理：通过“预测-更新”递推得到最小均方误差意义下的最优估计。               */
/*       q 为过程噪声方差，r 为测量噪声方差，q/r 决定信任模型还是测量。       */
/* 特点：能同时平滑噪声并跟踪真实值。                                         */
typedef struct {
    filter_t x;        /* 状态估计值   */
    filter_t p;        /* 估计协方差   */
    filter_t q;        /* 过程噪声方差 */
    filter_t r;        /* 测量噪声方差 */
    bool     has_init; /* 是否已初始化 */
} filter_kalman_t;

/**
  * @brief  初始化一维卡尔曼滤波器
  * @param[in]  f 滤波器句柄
  * @param[in]  q 过程噪声方差
  * @param[in]  r 测量噪声方差
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  q、r 非正时自动置为 0.001
  */
void filter_kalman_init(filter_kalman_t *f, filter_t q, filter_t r);

/**
  * @brief  执行一次一维卡尔曼滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次观测值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态（x/p/has_init）
  * @return  状态最优估计值
  * @note  首个观测值直接作为初始状态
  */
filter_t filter_kalman(filter_kalman_t *f, filter_t v);


/**
 * @brief 各种滤波初始化函数
 * @param[in] 无
 * @param[out] 无
 * @param[inout] 无
 * @return 返回值
 * @note 用于封装调用不同滤波，进行滤波方法初始化的函数
 */
void Filtering_Init(void);


/*对外滤波方法选择结构体*/
typedef struct
{
 
    /*一阶低通滤波实例: 三相采样电流滤波*/
    filter_lag_t        Iu_filter;    
    filter_lag_t        Iv_filter;
    filter_lag_t        Iw_filter;
    /*一阶低通滤波实例：当前机械转速滤波*/
        filter_lag_t        f_rpm_fb_lag;
    /*一阶通滤波实现：PLL转速观测器输出滤波(层换相台阶梯尖峰) */
    filter_lag_t        f_pll_lag;
    
    
}FilterMethod_TypeDef;

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */
