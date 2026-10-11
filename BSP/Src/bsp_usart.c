
#include "bsp_usart.h"


char RxBuffer[RXBUFFERSIZE]; 
/**
  * 函数功能: 重定向c库函数printf到USART1
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/**
  * 函数功能: 重定向c库函数getchar,scanf到USART1
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
 
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}

  