/*
 * File      : led.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-13-05     bright       the first version
 */

#ifndef __LED_H__
#define __LED_H__

#include <rthw.h>
#include <rtthread.h>
#include "stm32f0xx_hal.h"

#define rt_hw_led_on()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET)
#define rt_hw_led_off()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET)

void rt_hw_led_init(void);

#endif
