/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2012-02-08     aozima       update for F4.
 * 2012-07-28     aozima       update for ART board.
 * 2016-05-28     armink       add DMA Rx mode
 */


#include "usart.h"
#include "board.h"

#include <rtdevice.h>

#ifdef RT_USING_UART1
#define USART1_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART1_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

/* Definition for USART1 Pins */
#define USART1_TX_PIN                    GPIO_PIN_9
#define USART1_TX_GPIO_PORT              GPIOA
#define USART1_TX_AF                     GPIO_AF1_USART1
#define USART1_RX_PIN                    GPIO_PIN_10
#define USART1_RX_GPIO_PORT              GPIOA
#define USART1_RX_AF                     GPIO_AF1_USART1
#endif

#ifdef RT_USING_UART2
#define USART2_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()
#define USART2_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()

/* Definition for USART2 Pins */
#define USART2_TX_PIN                    GPIO_PIN_5
#define USART2_TX_GPIO_PORT              GPIOD
#define USART2_TX_AF                     GPIO_AF7_USART2
#define USART2_RX_PIN                    GPIO_PIN_6
#define USART2_RX_GPIO_PORT              GPIOD
#define USART2_RX_AF                     GPIO_AF7_USART2
#endif



/* STM32 uart driver */
struct stm32_uart
{
    UART_HandleTypeDef UartHandle;
    IRQn_Type irq;
};

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
  struct stm32_uart *uart;

  RT_ASSERT(serial != RT_NULL);
  RT_ASSERT(cfg != RT_NULL);

  uart = (struct stm32_uart *)serial->parent.user_data;

  uart->UartHandle.Init.BaudRate   = cfg->baud_rate;
  uart->UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  uart->UartHandle.Init.Mode       = UART_MODE_TX_RX;
  uart->UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  switch (cfg->data_bits)
  {
  case DATA_BITS_8:
      uart->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
      break;
  case DATA_BITS_9:
      uart->UartHandle.Init.WordLength = UART_WORDLENGTH_9B;
      break;
  default:
      uart->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
      break;
  }
  switch (cfg->stop_bits)
  {
  case STOP_BITS_1:
      uart->UartHandle.Init.StopBits   = UART_STOPBITS_1;
      break;
  case STOP_BITS_2:
      uart->UartHandle.Init.StopBits   = UART_STOPBITS_2;
      break;
  default:
      uart->UartHandle.Init.StopBits   = UART_STOPBITS_1;
      break;
  }
  switch (cfg->parity)
  {
  case PARITY_NONE:
      uart->UartHandle.Init.Parity     = UART_PARITY_NONE;
      break;
  case PARITY_ODD:
      uart->UartHandle.Init.Parity     = UART_PARITY_ODD;
      break;
  case PARITY_EVEN:
      uart->UartHandle.Init.Parity     = UART_PARITY_EVEN;
      break;
  default:
      uart->UartHandle.Init.Parity     = UART_PARITY_NONE;
      break;
  }

  if (HAL_UART_Init(&uart->UartHandle) != HAL_OK)
  {
      return RT_ERROR;
  }

  return RT_EOK;
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
  struct stm32_uart *uart;

  RT_ASSERT(serial != RT_NULL);
  uart = (struct stm32_uart *)serial->parent.user_data;

  switch (cmd)
  {
  case RT_DEVICE_CTRL_CLR_INT:
      /* disable rx irq */
      NVIC_DisableIRQ(uart->irq);
      /* disable interrupt */
      __HAL_UART_DISABLE_IT(&uart->UartHandle, UART_IT_RXNE);
      break;
  case RT_DEVICE_CTRL_SET_INT:
      /* enable rx irq */
      NVIC_EnableIRQ(uart->irq);
      /* enable interrupt */
      __HAL_UART_ENABLE_IT(&uart->UartHandle, UART_IT_RXNE);
      break;
  }

  return RT_EOK;
}

static int stm32_putc(struct rt_serial_device *serial, char c)
{
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    while((__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_TXE) == RESET));
    uart->UartHandle.Instance->TDR = c;
    return 1;
}

static int stm32_getc(struct rt_serial_device *serial)
{
    int ch;
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    ch = -1;
    if (__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_RXNE) != RESET)
        ch = uart->UartHandle.Instance->RDR & 0xff;
    return ch;
}

/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void uart_isr(struct rt_serial_device *serial) {
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;

    RT_ASSERT(uart != RT_NULL);

    /* UART in mode Receiver -------------------------------------------------*/
    if ((__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&uart->UartHandle, UART_IT_RXNE) != RESET))
    {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
        /* Clear RXNE interrupt flag */
        __HAL_UART_CLEAR_FLAG(&uart->UartHandle, UART_FLAG_RXNE);
    }
}

/**
* @brief UART MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
*           - NVIC configuration for UART interrupt request enable
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

#if defined(RT_USING_UART1)
    if (huart->Instance == USART1)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO TX/RX clock */
        USART1_TX_GPIO_CLK_ENABLE();
        USART1_RX_GPIO_CLK_ENABLE();

        /* Enable USARTx clock */
        __HAL_RCC_USART1_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ##########################################*/
        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = USART1_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = USART1_TX_AF;

        HAL_GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStruct);

        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = USART1_RX_PIN;
        GPIO_InitStruct.Alternate = USART1_RX_AF;

        HAL_GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
#endif

#if defined(RT_USING_UART2)
    if (huart->Instance == USART2)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO TX/RX clock */
        USART2_TX_GPIO_CLK_ENABLE();
        USART2_RX_GPIO_CLK_ENABLE();

        /* Enable USARTx clock */
        __HAL_RCC_USART2_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ##########################################*/
        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = USART2_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = USART2_TX_AF;

        HAL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStruct);

        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = USART2_RX_PIN;
        GPIO_InitStruct.Alternate = USART2_RX_AF;

        HAL_GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
#endif

}

/**
* @brief UART MSP De-Initialization
*        This function frees the hardware resources used in this example:
*          - Disable the Peripheral's clock
*          - Revert GPIO and NVIC configuration to their default state
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
#if defined(RT_USING_UART1)
    if (huart->Instance == USART1)
    {
        /*##-1- Reset peripherals ##################################################*/
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();

        /*##-2- Disable peripherals and GPIO Clocks #################################*/
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(USART1_TX_GPIO_PORT, USART1_TX_PIN);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(USART1_RX_GPIO_PORT, USART1_RX_PIN);

        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
#endif

#if defined(RT_USING_UART2)
    if (huart->Instance == USART2)
    {
        /*##-1- Reset peripherals ##################################################*/
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();

        /*##-2- Disable peripherals and GPIO Clocks #################################*/
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(USART2_TX_GPIO_PORT, USART2_TX_PIN);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(USART2_RX_GPIO_PORT, USART2_RX_PIN);

        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
#endif

}


static const struct rt_uart_ops stm32_uart_ops =
{
    stm32_configure,
    stm32_control,
    stm32_putc,
    stm32_getc,
};

#if defined(RT_USING_UART1)
/* UART1 device driver structure */
struct stm32_uart uart1 =
{
    {USART1},
    USART1_IRQn,
};
struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
/* UART2 device driver structure */
struct stm32_uart uart2 =
{
    {USART2},
    USART2_IRQn,
};
struct rt_serial_device serial2;

void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* RT_USING_UART2 */



int stm32_hw_usart_init(void)
{
    struct stm32_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef RT_USING_UART1
    uart = &uart1;
    uart->UartHandle.Instance = USART1;

    serial1.ops    = &stm32_uart_ops;
    serial1.config = config;

    /* register UART1 device */
    rt_hw_serial_register(&serial1,
                          "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART2
    uart = &uart2;
    uart->UartHandle.Instance = USART2;

    serial2.ops    = &stm32_uart_ops;
    serial2.config = config;

    /* register UART1 device */
    rt_hw_serial_register(&serial2,
                          "uart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
#endif /* RT_USING_UART2 */
    return 0;
}
INIT_BOARD_EXPORT(stm32_hw_usart_init);

