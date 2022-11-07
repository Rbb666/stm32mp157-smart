/*
 *  serial.c UART driver
 *
 * COPYRIGHT (C) 2013, Shanghai Real-Thread Technology Co., Ltd
 *
 *  This file is part of RT-Thread (http://www.rt-thread.org)
 *  Maintainer: bernard.xiong <bernard.xiong at gmail.com>
 *
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-03-30     Bernard      the first verion
 */

#include <rthw.h>
#include <rtdevice.h>

#include "serial.h"
#include "board.h"
#include "mmu.h"

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

struct hw_uart_device
{
    rt_uint32_t hw_base;
    rt_uint32_t irqno;
};

#define UART_DR(base) __REG32(base + 0x00)
#define UART_FR(base) __REG32(base + 0x18)
#define UART_CR(base) __REG32(base + 0x30)
#define UART_IMSC(base) __REG32(base + 0x38)
#define UART_ICR(base) __REG32(base + 0x44)

#define UARTFR_RXFE 0x10
#define UARTFR_TXFF 0x20
#define UARTIMSC_RXIM 0x10
#define UARTIMSC_TXIM 0x20
#define UARTICR_RXIC 0x10
#define UARTICR_TXIC 0x20

static void rt_hw_uart_isr(int irqno, void *param)
{
    struct rt_serial_device *serial = (struct rt_serial_device *)param;
    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
}

static rt_err_t uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
#if 1
    /* 115200,8n1 */

    UART_Type *uart_reg;
    struct hw_uart_device *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct hw_uart_device *)serial->parent.user_data;

    uart_reg = (UART_Type *)uart->hw_base;

    /* 配置引脚功能
     * PA11, PA12 as alternate function
     */

    /* AF6 : UART4_TX, UART4_RX */

    /* 先关闭串口 */
    uart_reg->USART_CR1 &= ~(1 << 0);

    /* 设置波特率
     * 115200 = 104.5*1000,000/USARTDIV
     * USARTDIV = 104.5*1000,000/115200 = 907
     */
    // uart_reg->USART_BRR = 907;

    /* 设置数据格式
     * 8N1
     */
    uart_reg->USART_CR1 |= (1 << 3) | (1 << 2);
    uart_reg->USART_CR2 &= ~(3 << 12);

    /* 使能UART模块 */
    uart_reg->USART_CR1 |= (1 << 0);
#endif
    return RT_EOK;
}

static rt_err_t uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct hw_uart_device *uart;
    UART_Type *uart_reg;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct hw_uart_device *)serial->parent.user_data;
    uart_reg = (UART_Type *)uart->hw_base;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        uart_reg->USART_CR1 &= ~(1 << 5);
        break;

    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        uart_reg->USART_CR1 |= (1 << 5);
        rt_hw_interrupt_umask(uart->irqno);
        break;
    }

    return RT_EOK;
}

static int uart_putc(struct rt_serial_device *serial, char c)
{
    struct hw_uart_device *uart;
    UART_Type *uart_reg;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct hw_uart_device *)serial->parent.user_data;
    uart_reg = (UART_Type *)uart->hw_base;

    while ((uart_reg->USART_ISR & (1 << 7)) == 0)
        ;
    uart_reg->USART_TDR = c;

    return 1;
}

static int uart_getc(struct rt_serial_device *serial)
{
    int ch;
    struct hw_uart_device *uart;
    UART_Type *uart_reg;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct hw_uart_device *)serial->parent.user_data;
    uart_reg = (UART_Type *)uart->hw_base;

    if ((uart_reg->USART_ISR & (1 << 5)) == 0)
        ch = -1;
    else
        ch = uart_reg->USART_RDR;

    return ch;
}

static const struct rt_uart_ops _uart_ops =
{
        uart_configure,
        uart_control,
        uart_putc,
        uart_getc,
};

#ifdef BSP_USING_UART4
/* UART device driver structure */
static struct hw_uart_device _uart0_device =
{
    0x40010000, /* 100ask_stm32mp157 uart4 phy addr */
    84,         /* rx irq */
};
static struct rt_serial_device _serial0;
#endif

#ifdef RT_USING_UART1
/* UART1 device driver structure */
static struct hw_uart_device _uart1_device =
    {
        REALVIEW_UART1_BASE,
        IRQ_PBA8_UART1,
};
static struct rt_serial_device _serial1;
#endif

int rt_hw_usart_init(void)
{
    struct hw_uart_device *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef BSP_USING_UART4
    _uart0_device.hw_base = (uint32_t)rt_ioremap((void *)_uart0_device.hw_base, 0x1000);
    uart = &_uart0_device;

    _serial0.ops = &_uart_ops;
    _serial0.config = config;

    /* register UART1 device */
    rt_hw_serial_register(&_serial0, "uart4",
                          RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
    rt_hw_interrupt_install(uart->irqno, rt_hw_uart_isr, &_serial0, "uart4");
#endif

#ifdef RT_USING_UART1
    _uart1_device.hw_base = (uint32_t)rt_ioremap((void *)_uart1_device.hw_base, 0x1000);
    uart = &_uart1_device;
    _serial1.ops = &_uart_ops;
    _serial1.config = config;

    /* register UART1 device */
    rt_hw_serial_register(&_serial1, "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX, uart);
    /* enable Rx and Tx of UART */
    UART_CR(uart->hw_base) = (1 << 0) | (1 << 8) | (1 << 9);
    rt_hw_interrupt_install(uart->irqno, rt_hw_uart_isr, &_serial1, "uart1");
#endif

    return 0;
}
INIT_BOARD_EXPORT(rt_hw_usart_init);
