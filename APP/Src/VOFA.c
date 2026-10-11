/*
 * 文件: VOFA.c
 * 说明: VOFA上位机数据发送，使用DMA非阻塞发送
 * 版本: V2.0
 * 用途: 向上位机(VOFA+)发送控制量与反馈量，用于波形观察与参数整定
 */
#include "VOFA.h"
#include "filter.h"
#include <stdlib.h>

VofaJustWater_TypeDef VofaSend = {0};


/*===================================  JustFloat协议  ==============================*/
/**     
 *@brief  VOFA数据填充函数
 *@param  无
 *@return 无
**/
void VofaJustWater_Send(void)
{   
    /*如果DMA正在发送，跳过这一帧（丢帧不排队）*/
    if(VofaSend.tx_busy == 1) return;


    /* 根据当前控制模式填充对应反馈通道；VOFA只读取和发送，不主动切换run_mode */
    switch (MotorBLDC.run_mode)
    {
        case MOTOR_RUN_TORQUE:
            VofaSend.channels = VOFA_CH_TORQUE;
            VofaSend.Send_Data_Array[0] = MotorBLDC.current.d.Target;   /* Id目标(A) */
            VofaSend.Send_Data_Array[1] = MotorBLDC.FOCParm.Id;    /* Id实际(A) */
            VofaSend.Send_Data_Array[2] = MotorBLDC.current.q.Target;   /* Iq目标(A) */
            VofaSend.Send_Data_Array[3] = MotorBLDC.FOCParm.Iq;    /* Iq实际(A) */
            VofaSend.Send_Data_Array[4] = Motor_GetSpeedFB();      /* 环内实际转速(rpm) */
            VofaSend.Send_Data_Array[5] = MotorBLDC.FOCParm.Theta * RAD2DEG(1.0f); /* 电角度(度) */
            break;

        case MOTOR_RUN_SPEED:
            VofaSend.channels = VOFA_CH_SPEED;
            VofaSend.Send_Data_Array[0]  = (float)MotorBLDC.state;              /* 状态机状态0~4 */
            VofaSend.Send_Data_Array[1]  = MotorBLDC.current.d.Target;         /* Id目标(A) */
            VofaSend.Send_Data_Array[2]  = MotorBLDC.FOCParm.Id;               /* Id实际(A) */
            VofaSend.Send_Data_Array[3]  = MotorBLDC.current.q.Target;         /* Iq目标(A) */
            VofaSend.Send_Data_Array[4]  = MotorBLDC.FOCParm.Iq;               /* Iq实际(A) */
            VofaSend.Send_Data_Array[5]  = MotorBLDC.current.d.Error;          /* d轴误差 */
            VofaSend.Send_Data_Array[6]  = MotorBLDC.current.q.Error;          /* q轴误差 */
            VofaSend.Send_Data_Array[7]  = MotorBLDC.current.d.Kp;             /* Kp */
            VofaSend.Send_Data_Array[8]  = MotorBLDC.current.d.Ki;             /* Ki */
            VofaSend.Send_Data_Array[9]  = MotorBLDC.current.d.OutMax;         /* OutMax(V) */
            VofaSend.Send_Data_Array[10] = MotorBLDC.current.d.Out;            /* d轴PI输出 */
            VofaSend.Send_Data_Array[11] = MotorBLDC.current.q.Out;            /* q轴PI输出 */
            VofaSend.Send_Data_Array[12] = MotorBLDC.FOCParm.Vd;               /* Vd(V) */
            VofaSend.Send_Data_Array[13] = MotorBLDC.FOCParm.Vq;               /* Vq(V) */
            VofaSend.Send_Data_Array[14] = MotorBLDC.FOCParm.Theta * RAD2DEG(1.0f); /* 电角度(度) */
            VofaSend.Send_Data_Array[15] = (float)MotorBLDC.FOCParm.CCR_U;     /* U相CCR */
            VofaSend.Send_Data_Array[16] = (float)MotorBLDC.FOCParm.CCR_V;     /* V相CCR */
            VofaSend.Send_Data_Array[17] = (float)MotorBLDC.FOCParm.CCR_W;     /* W相CCR */
            break;

        case MOTOR_RUN_POSITION:
        default:
            VofaSend.channels = VOFA_CH_POSITION;
            VofaSend.Send_Data_Array[0] = Motor_GetPosTarget();      /* 位置目标(度) */
            /* 位置实际显示"单圈角"(0~360°)：累计机械角在 VOFA 里不直观且无意义 */
            {
                float _pf = fmodf(Motor_GetPosFB(), _2PI);
                if (_pf < 0.0f) _pf += _2PI;   /* 负角正数化到 0~360 */
                VofaSend.Send_Data_Array[1] = _pf * RAD2DEG(1.0f); /* 位置实际(度) */
            }
            VofaSend.Send_Data_Array[2] = MotorBLDC.position.pid.Error * RAD2DEG(1.0f); /* 位置误差(度) */
            VofaSend.Send_Data_Array[3] = MotorBLDC.speed.pid.Target; /* 速度目标(rpm) */
            VofaSend.Send_Data_Array[4] = Motor_GetSpeedFB();        /* 速度实际(rpm) */
            VofaSend.Send_Data_Array[5] = MotorBLDC.current.q.Target;     /* Iq目标(A) */
            VofaSend.Send_Data_Array[6] = MotorBLDC.FOCParm.Iq;      /* Iq实际(A) */
            VofaSend.Send_Data_Array[7] = MotorBLDC.FOCParm.Theta;   /* 电角度(rad) */
            break;
    }

    Vofa_Data_Process();
}

/**     
 *@brief  VOFA数据组帧并DMA发送（非阻塞）
 *@param  无
 *@return 无
**/
void Vofa_Data_Process(void)
{
    uint16_t frame_len = VOFA_FRAME_LEN(VofaSend.channels);

    /*先标记忙，防止DMA发送期间缓冲区被改写*/
    VofaSend.tx_busy = 1;

    /*拷贝当前模式对应数量的浮点数据到字节数组*/
    memcpy(VofaSend.Byte_Data_Array,
           VofaSend.Send_Data_Array,
           frame_len - 4U);

    /*JustFloat协议固定帧尾*/
    VofaSend.Byte_Data_Array[frame_len - 4U] = 0x00;
    VofaSend.Byte_Data_Array[frame_len - 3U] = 0x00;
    VofaSend.Byte_Data_Array[frame_len - 2U] = 0x80;
    VofaSend.Byte_Data_Array[frame_len - 1U] = 0x7f;

    /*DMA非阻塞发送*/
    HAL_UART_Transmit_DMA(&huart1, VofaSend.Byte_Data_Array, frame_len);
}

/**     
 *@brief  DMA发送完成回调，在HAL_UART_TxCpltCallback里调用
 *        标记发送完成，允许下一帧数据发送
**/
void VOFA_TxCpltCallback(void)
{
    VofaSend.tx_busy = 0;
}

/*===================================  串口命令接收（DMA+空闲，实时改目标转速）  ==============================*/
#define CMD_DMA_LEN  32
static uint8_t s_rx_dma_buf[CMD_DMA_LEN];   /* DMA一行命令接收缓存 */

/**
  * @brief  命令接收初始化：启动USART1 DMA接收，用空闲中断判定一行结束
  * @note   字节由DMA硬件直接搬入缓存，不依赖CPU逐字节响应，
  *         FOC中断阻塞再久也不会溢出丢字
  */
void VOFA_CommandInit(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx_dma_buf, CMD_DMA_LEN);
    /* 关闭DMA半传输中断，只在"总线空闲/缓存满"时触发，避免半包误解析 */
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/* 命令字符串转小写，方便同时识别大小写 */
static void cmd_to_lower(char *s)
{
    while (*s != '\0')
    {
        if (*s >= 'A' && *s <= 'Z') *s = (char)(*s + 32);
        s++;
    }
}

/* 匹配 key=value 或 keyvalue，例如 iq=0.3、iq0.3 */
static int cmd_get_value(const char *line, const char *key, float *value)
{
    size_t key_len = strlen(key);
    const char *p;

    if (strncmp(line, key, key_len) != 0) return 0;

    p = line + key_len;
    if (*p == '=') p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') return 0;

    *value = (float)atof(p);
    return 1;
}

/* d/q电流环使用同一组增益，所以修改电流环参数时同时更新 current.d 和 current.q */
static void cmd_set_current_gain(uint8_t which, float value)
{
    PID_SetGain(&MotorBLDC.current.d, which, value);
    PID_SetGain(&MotorBLDC.current.q, which, value);
}

static void cmd_set_speed_target(float rpm)
{
    if (rpm > SPEED_FB_RPM_MAX) rpm = SPEED_FB_RPM_MAX;
    if (rpm < -SPEED_FB_RPM_MAX) rpm = -SPEED_FB_RPM_MAX;
    speed_target = rpm;
}

/**
  * @brief  串口空闲(或DMA缓存满)事件回调：一整行命令接收完成
  * @note   VOFA只修改目标和PID参数，不切换run_mode。
  *
  *         目标值命令：
  *           "id=0.1" / "d0.1"     Id目标(A)
  *           "iq=0.3" / "q0.3"     Iq目标(A)
  *           "rpm=100" / "s100"    速度目标(rpm)，纯数字也按速度目标解析
  *           "pos=30" / "p30"      位置目标(度)
  *           "posr=1.57" / "pr1.57" 位置目标(rad)
  *
  *         电流环PID：ckp / cki / ckd，例如 "ckp=2.3"
  *         速度环PID：skp / ski / skd，例如 "ski=0.01"
  *         位置环PID：pkp / pki / pkd，例如 "pkp=200"
  *
  *         模式切换：m0/mt=转矩, m1/ms=速度, m2/mp=位置（Motor_RunMode_Set 内部会
  *         复位速度环积分/分频/前馈状态并清电流目标，无扰切换）
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART1) return;

    if (Size > 0)
    {
        uint16_t n = Size;
        /* 去掉行尾的 \r \n，保证atof遇到字符串结束符 */
        while (n > 0 && (s_rx_dma_buf[n-1] == '\r' || s_rx_dma_buf[n-1] == '\n'))
        {
            n--;
        }
        if (n > 0)
        {
            char line[CMD_DMA_LEN];
            if (n >= CMD_DMA_LEN) n = CMD_DMA_LEN - 1;
            memcpy(line, s_rx_dma_buf, n);
            line[n] = '\0';

            cmd_to_lower(line);

            float value;
            if (cmd_get_value(line, "id", &value))
            {
                Motor_TorqueIdTarget_Set(value);
            }
            else if (cmd_get_value(line, "iq", &value))
            {
                Motor_TorqueIqTarget_Set(value);
            }
            else if (cmd_get_value(line, "rpm", &value) ||
                     cmd_get_value(line, "spd", &value))
            {
                cmd_set_speed_target(value);
            }
            else if (cmd_get_value(line, "posr", &value))
            {
                Motor_PosTarget_Set(RAD2DEG(value));
            }
            else if (cmd_get_value(line, "pos", &value))
            {
                Motor_PosTarget_Set(value);
            }
            else if (cmd_get_value(line, "ckp", &value))
            {
                cmd_set_current_gain(PID_GAIN_KP, value);
            }
            else if (cmd_get_value(line, "cki", &value))
            {
                cmd_set_current_gain(PID_GAIN_KI, value);
            }
            else if (cmd_get_value(line, "ckd", &value))
            {
                cmd_set_current_gain(PID_GAIN_KD, value);
            }
            else if (cmd_get_value(line, "skp", &value))
            {
                PID_SetGain(&MotorBLDC.speed.pid, PID_GAIN_KP, value);
            }
            else if (cmd_get_value(line, "ski", &value))
            {
                PID_SetGain(&MotorBLDC.speed.pid, PID_GAIN_KI, value);
            }
            else if (cmd_get_value(line, "skd", &value))
            {
                PID_SetGain(&MotorBLDC.speed.pid, PID_GAIN_KD, value);
            }
            else if (cmd_get_value(line, "pkp", &value))
            {
                PID_SetGain(&MotorBLDC.position.pid, PID_GAIN_KP, value);
            }
            else if (cmd_get_value(line, "pki", &value))
            {
                PID_SetGain(&MotorBLDC.position.pid, PID_GAIN_KI, value);
            }
            else if (cmd_get_value(line, "pkd", &value))
            {
                PID_SetGain(&MotorBLDC.position.pid, PID_GAIN_KD, value);
            }
            else if (line[0] == 'p')
            {
                if (line[1] == 'r') Motor_PosTarget_Set(RAD2DEG(atof(line + 2)));
                else                Motor_PosTarget_Set(atof(line + 1));
            }
            else if (line[0] == 'q')
            {
                Motor_TorqueIqTarget_Set(atof(line + 1));
            }
            else if (line[0] == 'd')
            {
                Motor_TorqueIdTarget_Set(atof(line + 1));
            }
            else if (line[0] == 's')
            {
                cmd_set_speed_target(atof(line + 1));
            }
            else if (line[0] == '-' || line[0] == '+' ||
                     (line[0] >= '0' && line[0] <= '9'))
            {
                cmd_set_speed_target(atof(line));
            }
            else if (line[0] == 'm')
            {
                /* 在线切换控制模式：m0/mt=扭矩, m1/ms=速度, m2/mp=位置 */
                if      (line[1] == '0' || line[1] == 't') Motor_RunMode_Set(&MotorBLDC, MOTOR_RUN_TORQUE);
                else if (line[1] == '1' || line[1] == 's') Motor_RunMode_Set(&MotorBLDC, MOTOR_RUN_SPEED);
                else if (line[1] == '2' || line[1] == 'p') Motor_RunMode_Set(&MotorBLDC, MOTOR_RUN_POSITION);
            }
        }
    }

    /* NORMAL模式下事件后DMA已停，重新挂接收，从缓存起始处开始下一行 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx_dma_buf, CMD_DMA_LEN);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}
