/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018.10.30     SummerGift   first version
 * 2019.03.05     whj4674672   add stm32h7
 * 2020-10-14     Dozingfiretruck   Porting for stm32wbxx
 */

#ifndef __DRV_USART_H__
#define __DRV_USART_H__

#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include <drv_common.h>
#include "drv_dma.h"

int rt_hw_usart_init(void);

typedef struct
{
    volatile unsigned int USART_CR1;   /**< USART control register 1,                 offset: 0x00 	 串口控制寄存器1，            偏移地址0x00  */
    volatile unsigned int USART_CR2;   /**< USART control register 2,                 offset: 0x04 	 串口控制寄存器2，            偏移地址0x04  */
    volatile unsigned int USART_CR3;   /**< USART control register 3,                 offset: 0x08 	 串口控制寄存器3，            偏移地址0x08  */
    volatile unsigned int USART_BRR;   /**< USART Baud Rate register,                 offset: 0x0C     串口波特率寄存器             偏移地址0x0C  */
    volatile unsigned int USART_GTPR;  /**< USART guard time and prescaler register,  offset: 0x10     串口保护时间和预分频器寄存器 偏移地址0x10  */
    volatile unsigned int USART_RTOR;  /**< USART receiver timeout register,          offset: 0x14     串口接收超时寄存器           偏移地址0x14  */
    volatile unsigned int USART_RQR;   /**< USART request register,                   offset: 0x18     串口请求寄存器               偏移地址0x18  */
    volatile unsigned int USART_ISR;   /**< USART interrupt and status register,      offset: 0x1C     串口中断与状态寄存器         偏移地址0x1C  */
    volatile unsigned int USART_ICR;   /**< USART interrupt flag clear register ,     offset: 0x20     串口中断状态清除寄存器       偏移地址0x20  */
    volatile unsigned int USART_RDR;   /**< USART receive data register,              offset: 0x24     串口接收数据寄存器           偏移地址0x24  */
    volatile unsigned int USART_TDR;   /**< USART transmit data register,             offset: 0x28     串口发送数据寄存器           偏移地址0x28  */
    volatile unsigned int USART_PRESC; /**< USART prescaler register,                 offset: 0x2C     串口预分频器寄存器           偏移地址0x2C  */
} UART_Type;

/* stm32 config class */
struct stm32_uart_config
{
    const char *name;
    rt_uint32_t Instance;
    rt_uint32_t irq_type;
};

/* stm32 uart dirver class */
struct stm32_uart
{
    UART_HandleTypeDef handle;
    struct stm32_uart_config *config;
    struct rt_serial_device serial;
};

#endif /* __DRV_USART_H__ */
