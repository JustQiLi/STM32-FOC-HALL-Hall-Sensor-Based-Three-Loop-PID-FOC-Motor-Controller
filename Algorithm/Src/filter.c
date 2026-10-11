/**
  * @file    filter.c
  * @brief   常用软件滤波算法库（实现文件）
  * @note    与 filter.h 配套使用。
  */

#include "filter.h"
#include <string.h>
#include <math.h>
#include "StateMachine.h"
/* ========================================================================== */
/*  1. 限幅滤波法（程序判断滤波）                                             */
/* ========================================================================== */

/**
  * @brief  初始化限幅滤波器
  * @param[in]  f 滤波器句柄
  * @param[in]  max_delta 允许的最大偏差
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  无
  */
void filter_limit_init(filter_limit_t *f, filter_t max_delta)
{
    if (f == NULL) return;
    f->last      = 0;
    f->max_delta = max_delta;
    f->has_last  = false;
}

/**
  * @brief  执行一次限幅滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态（last/has_last）
  * @return  滤波后的有效值
  * @note  偏差超限时沿用上次有效值
  */
filter_t filter_limit(filter_limit_t *f, filter_t v)
{
    if (f == NULL) return v;

    if (!f->has_last) {
        /* 第一个样本直接作为有效值 */
        f->last     = v;
        f->has_last = true;
        return v;
    }

    if ((v - f->last) > f->max_delta || (f->last - v) > f->max_delta) {
        /* 偏差超限，认为有干扰，沿用上次有效值 */
        return f->last;
    }

    f->last = v;
    return v;
}

/* ========================================================================== */
/*  2. 中位值滤波法                                                           */
/* ========================================================================== */

/**
  * @brief  初始化中位值滤波器
  * @param[in]  f 滤波器句柄
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  无
  */
void filter_median_init(filter_median_t *f)
{
    if (f == NULL) return;
    memset(f, 0, sizeof(*f));
}

/**
  * @brief  执行一次中位值滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部窗口与计数
  * @return  窗口内排序后的中间值
  * @note  窗口未满时对已采样部分取中值
  */
filter_t filter_median(filter_median_t *f, filter_t v)
{
    uint8_t i, j;

    if (f == NULL) return v;

    /* 写入窗口 */
    f->buf[f->idx] = v;
    f->idx = (uint8_t)((f->idx + 1) % FILTER_MEDIAN_WINDOW);
    if (f->cnt < FILTER_MEDIAN_WINDOW) f->cnt++;

    /* 复制到临时数组并做冒泡排序（窗口很小，开销可忽略） */
    memcpy(f->tmp, f->buf, sizeof(f->tmp));
    for (i = 0; i < f->cnt - 1; i++) {
        for (j = 0; j < f->cnt - 1 - i; j++) {
            if (f->tmp[j] > f->tmp[j + 1]) {
                filter_t t  = f->tmp[j];
                f->tmp[j]     = f->tmp[j + 1];
                f->tmp[j + 1] = t;
            }
        }
    }

    /* 取中间值（窗口为奇数，cnt 未满时取已排序部分的中间值） */
    return f->tmp[f->cnt / 2];
}

/* ========================================================================== */
/*  3. 算术平均滤波法                                                         */
/* ========================================================================== */

/**
  * @brief  对一段采样缓冲求算术平均
  * @param[in]  buf 采样缓冲首地址
  * @param[in]  n 样本个数
  * @param[out] 无
  * @param[inout] 无
  * @return  算术平均值；buf 为空或 n==0 时返回 0
  * @note  无状态，可在任意时刻对任意缓冲调用
  */
filter_t filter_average(const filter_t *buf, uint16_t n)
{
    filter_t sum = 0;
    uint16_t i;

    if (buf == NULL || n == 0) return 0;

    for (i = 0; i < n; i++) {
        sum += buf[i];
    }
    return sum / (filter_t)n;
}

/* ========================================================================== */
/*  4. 递推平均滤波法（滑动平均）                                             */
/* ========================================================================== */

/**
  * @brief  初始化滑动平均滤波器
  * @param[in]  f 滤波器句柄
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  无
  */
void filter_moving_init(filter_moving_t *f)
{
    if (f == NULL) return;
    memset(f, 0, sizeof(*f));
}

/**
  * @brief  执行一次滑动平均滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部窗口、和与计数
  * @return  当前窗口的滑动平均值
  * @note  窗口未满时按实际已采样个数取平均
  */
filter_t filter_moving(filter_moving_t *f, filter_t v)
{
    if (f == NULL) return v;

    /* 减去将被覆盖的旧值，加上新值，维护窗口和 */
    f->sum -= f->buf[f->idx];
    f->buf[f->idx] = v;
    f->sum += v;

    f->idx = (uint8_t)((f->idx + 1) % FILTER_MOVING_WINDOW);
    if (f->cnt < FILTER_MOVING_WINDOW) f->cnt++;

    return f->sum / (filter_t)f->cnt;
}

/* ========================================================================== */
/*  5. 中位值平均滤波法（防脉冲干扰平均滤波）                                 */
/* ========================================================================== */

/**
  * @brief  对一段采样缓冲做去极值平均（中位值平均）
  * @param[in]  buf 采样缓冲首地址
  * @param[in]  n 样本个数
  * @param[out] 无
  * @param[inout] 无
  * @return  去掉最大最小值后的平均值；样本不足 3 个时退化为普通平均
  * @note  无状态，可在任意时刻对任意缓冲调用
  */
filter_t filter_median_average(const filter_t *buf, uint16_t n)
{
    filter_t sum = 0;
    filter_t max, min;
    uint16_t i;

    if (buf == NULL || n < 3) {
        /* 样本不足 3 个时无法“去极值”，退化为普通平均 */
        return (buf == NULL || n == 0) ? 0 : filter_average(buf, n);
    }

    max = min = buf[0];
    sum = 0;
    for (i = 0; i < n; i++) {
        sum += buf[i];
        if (buf[i] > max) max = buf[i];
        if (buf[i] < min) min = buf[i];
    }

    /* 去掉一个最大值和一个最小值后取平均 */
    return (sum - max - min) / (filter_t)(n - 2);
}

/* ========================================================================== */
/*  6. 限幅平均滤波法                                                         */
/* ========================================================================== */

/**
  * @brief  初始化限幅平均滤波器
  * @param[in]  f 滤波器句柄
  * @param[in]  max_delta 限幅允许的最大偏差
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  内部依次初始化限幅器与滑动平均器
  */
void filter_limit_avg_init(filter_limit_avg_t *f, filter_t max_delta)
{
    if (f == NULL) return;
    filter_limit_init(&f->limiter, max_delta);
    filter_moving_init(&f->averager);
}

/**
  * @brief  执行一次限幅平均滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态
  * @return  限幅后经滑动平均的输出值
  * @note  先限幅、再滑动平均
  */
filter_t filter_limit_avg(filter_limit_avg_t *f, filter_t v)
{
    filter_t limited;

    if (f == NULL) return v;

    /* 先限幅，再滑动平均 */
    limited = filter_limit(&f->limiter, v);
    return filter_moving(&f->averager, limited);
}

/* ========================================================================== */
/*  7. 一阶滞后滤波法（低通滤波）                                             */
/* ========================================================================== */

/**
  * @brief  初始化一阶滞后（低通）滤波器
  * @param[in]  f 滤波器句柄
  * @param[in]  a 滤波系数（0<a≤1）
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  系数越界时自动夹在 (0,1] 区间
  */
void filter_lag_init(filter_lag_t *f, filter_t a)
{
    if (f == NULL) return;

    /* 系数合法性保护：夹在 (0,1] 之间 */
    if (a <= 0) a = (filter_t)0.001;
    if (a > 1)  a = (filter_t)1.0;

    f->a     = a;
    f->y     = 0;
    f->has_y = false;
}

/**
  * @brief  执行一次一阶滞后（低通）滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态（y/has_y）
  * @return  滤波后的输出值
  * @note  首个样本直接作为输出
  */
filter_t filter_lag(filter_lag_t *f, filter_t v)
{
    if (f == NULL) return v;

    if (!f->has_y) {
        f->y     = v;
        f->has_y = true;
        return v;
    }

    /* y = a * x + (1 - a) * y_last */
    f->y = f->a * v + (1 - f->a) * f->y;
    return f->y;
}

/* ========================================================================== */
/*  8. 加权递推平均滤波法                                                     */
/* ========================================================================== */

/**
  * @brief  初始化加权递推平均滤波器
  * @param[in]  f 滤波器句柄
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  无
  */
void filter_weighted_init(filter_weighted_t *f)
{
    if (f == NULL) return;
    memset(f, 0, sizeof(*f));
}

/**
  * @brief  执行一次加权递推平均滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部窗口与计数
  * @return  加权平均后的输出值
  * @note  权重从旧到新依次为 1,2,3,...（越新权重越大）
  */
filter_t filter_weighted(filter_weighted_t *f, filter_t v)
{
    filter_t sum        = 0;
    filter_t weight_sum = 0;
    uint8_t  i, w;

    if (f == NULL) return v;

    /* 写入窗口 */
    f->buf[f->idx] = v;
    f->idx = (uint8_t)((f->idx + 1) % FILTER_WEIGHTED_WINDOW);
    if (f->cnt < FILTER_WEIGHTED_WINDOW) f->cnt++;

    /* 越新的数据权重越大：按“从旧到新”依次编号，权重 = 序号 + 1 */
    for (i = 0; i < f->cnt; i++) {
        uint8_t pos = (uint8_t)((f->idx - f->cnt + i + FILTER_WEIGHTED_WINDOW)
                                % FILTER_WEIGHTED_WINDOW);
        w = (uint8_t)(i + 1);
        sum        += f->buf[pos] * (filter_t)w;
        weight_sum += (filter_t)w;
    }

    return sum / weight_sum;
}

/* ========================================================================== */
/*  9. 消抖滤波法                                                             */
/* ========================================================================== */

/**
  * @brief  初始化消抖滤波器
  * @param[in]  f 滤波器句柄
  * @param[in]  threshold 判定阈值（连续出现次数）
  * @param[out] 无
  * @param[inout] 无
  * @return  无
  * @note  阈值小于 1 时自动置 1
  */
void filter_debounce_init(filter_debounce_t *f, uint8_t threshold)
{
    if (f == NULL) return;

    /* 阈值至少为 1，否则失去消抖意义 */
    if (threshold < 1) threshold = 1;

    f->last      = 0;
    f->candidate = 0;
    f->same_cnt  = 0;
    f->threshold = threshold;
    f->has_last  = false;
}

/**
  * @brief  执行一次消抖滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态
  * @return  消抖后的有效值（未达阈值时返回上次有效值）
  * @note  值与候选值一致则累计，否则重新计数
  */
filter_t filter_debounce(filter_debounce_t *f, filter_t v)
{
    if (f == NULL) return v;

    if (!f->has_last) {
        /* 首个样本直接作为有效输出 */
        f->last      = v;
        f->candidate = v;
        f->same_cnt  = 1;
        f->has_last  = true;
        return v;
    }

    if (v == f->candidate) {
        /* 与候选值一致，累计连续次数 */
        if (f->same_cnt < 255) f->same_cnt++;
        if (f->same_cnt >= f->threshold) {
            f->last = v;          /* 达到阈值，确认有效 */
        }
    } else {
        /* 与候选值不同，重新开始计数 */
        f->candidate = v;
        f->same_cnt  = 1;
    }

    return f->last;
}

/* ========================================================================== */
/*  10. 限幅消抖滤波法                                                        */
/* ========================================================================== */

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
                                uint8_t  threshold)
{
    if (f == NULL) return;
    filter_limit_init(&f->limiter, max_delta);
    filter_debounce_init(&f->debounce, threshold);
}

/**
  * @brief  执行一次限幅消抖滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次采样值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态
  * @return  限幅并消抖后的有效值
  * @note  先限幅、再消抖
  */
filter_t filter_limit_debounce(filter_limit_debounce_t *f, filter_t v)
{
    filter_t limited;

    if (f == NULL) return v;

    /* 先限幅，再消抖 */
    limited = filter_limit(&f->limiter, v);
    return filter_debounce(&f->debounce, limited);
}

/* ========================================================================== */
/*  11. 一维卡尔曼滤波                                                        */
/* ========================================================================== */

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
void filter_kalman_init(filter_kalman_t *f, filter_t q, filter_t r)
{
    if (f == NULL) return;

    /* 噪声方差必须为正，做合法性保护 */
    if (q <= 0) q = (filter_t)0.001;
    if (r <= 0) r = (filter_t)0.001;

    f->x        = 0;
    f->p        = 1;    /* 初始协方差给一个较大值，加快收敛 */
    f->q        = q;
    f->r        = r;
    f->has_init = false;
}

/**
  * @brief  执行一次一维卡尔曼滤波
  * @param[in]  f 滤波器句柄
  * @param[in]  v 本次观测值
  * @param[out] 无
  * @param[inout] f 滤波器内部状态（x/p/has_init）
  * @return  状态最优估计值
  * @note  首个观测值直接作为初始状态
  */
filter_t filter_kalman(filter_kalman_t *f, filter_t v)
{
    filter_t k;

    if (f == NULL) return v;

    if (!f->has_init) {
        /* 用首个观测值初始化状态 */
        f->x        = v;
        f->has_init = true;
        return v;
    }

    /* 预测：状态不变（一维恒值模型），协方差叠加过程噪声 */
    f->p = f->p + f->q;

    /* 更新：计算卡尔曼增益 */
    k = f->p / (f->p + f->r);

    /* 用观测值修正状态估计，并更新协方差 */
    f->x = f->x + k * (v - f->x);
    f->p = (1 - k) * f->p;

    return f->x;
}

/**
 * @brief 滤波初始化函数
 * @param[in] 无
 * @param[out] 无
 * @param[inout] 无
 * @return 返回值
 * @note 用于封装调用不同滤波，进行滤波方法初始化的函数
 */
void Filtering_Init(void)
{

    /* 速度反馈低通初始化：系数 a≈2π·fc·fs周期（fc<<fs） */
    filter_lag_init(&MotorBLDC.FilterMethod.f_rpm_fb_lag, _2PI * SPEED_FB_LP_HZ * ts);
    /* PLL转速观测器输出低通：滤除换相台阶梯尖峰，速度环不再过激反应 */
    filter_lag_init(&MotorBLDC.FilterMethod.f_pll_lag, _2PI * SPEED_PLL_LP_HZ * ts);
    /* 覆盖三相电流低通系数：filter.c 里写死的 0.2 在 300Hz 电流环带宽上吃掉约 69° 相位，
     * 三级串联后相位裕度只剩十几度，低速小电流时电流环会抖。改成 Motor_Config.h 里的宏。 */
    filter_lag_init(&MotorBLDC.FilterMethod.Iu_filter, CURR_FILTER_ALPHA);
    filter_lag_init(&MotorBLDC.FilterMethod.Iv_filter, CURR_FILTER_ALPHA);
    filter_lag_init(&MotorBLDC.FilterMethod.Iw_filter, CURR_FILTER_ALPHA);
}


