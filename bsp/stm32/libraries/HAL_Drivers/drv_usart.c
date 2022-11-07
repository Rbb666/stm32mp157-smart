/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-30     SummerGift   first version
 * 2020-03-16     SummerGift   add device close feature
 * 2020-03-20     SummerGift   fix bug caused by ORE
 * 2020-05-02     whj4674672   support stm32h7 uart dma
 * 2020-09-09     forest-rain  support stm32wl uart
 * 2020-10-14     Dozingfiretruck   Porting for stm32wbxx
 */

#include "board.h"
#include <rthw.h>
#include "mmu.h"

#include "drv_usart.h"
#include "drv_config.h"

#include "stm32mp1xx_hal.h"

#ifdef RT_USING_SERIAL

//#define DRV_DEBUG
#define LOG_TAG             "drv.usart"
#include <drv_log.h>

#if !defined(BSP_USING_UART1) && !defined(BSP_USING_UART2) && !defined(BSP_USING_UART3) && \
    !defined(BSP_USING_UART4) && !defined(BSP_USING_UART5) && !defined(BSP_USING_UART6) && \
    !defined(BSP_USING_UART7) && !defined(BSP_USING_UART8) && !defined(BSP_USING_LPUART1)
#error "Please define at least one BSP_USING_UARTx"
/* this driver can be disabled at menuconfig -> RT-Thread Components -> Device Drivers */
#endif

enum
{
#ifdef BSP_USING_UART1
    UART1_INDEX,
#endif
#ifdef BSP_USING_UART2
    UART2_INDEX,
#endif
#ifdef BSP_USING_UART3
    UART3_INDEX,
#endif
#ifdef BSP_USING_UART4
    UART4_INDEX,
#endif
#ifdef BSP_USING_UART5
    UART5_INDEX,
#endif
#ifdef BSP_USING_UART6
    UART6_INDEX,
#endif
#ifdef BSP_USING_UART7
    UART7_INDEX,
#endif
#ifdef BSP_USING_UART8
    UART8_INDEX,
#endif
#ifdef BSP_USING_LPUART1
    LPUART1_INDEX,
#endif
};

static struct stm32_uart_config uart_config[] =
{
#ifdef BSP_USING_UART1
    UART1_CONFIG,
#endif
#ifdef BSP_USING_UART2
    UART2_CONFIG,
#endif
#ifdef BSP_USING_UART3
    UART3_CONFIG,
#endif
#ifdef BSP_USING_UART4
    UART4_CONFIG,
#endif
#ifdef BSP_USING_UART5
    UART5_CONFIG,
#endif
#ifdef BSP_USING_UART6
    UART6_CONFIG,
#endif
#ifdef BSP_USING_UART7
    UART7_CONFIG,
#endif
#ifdef BSP_USING_UART8
    UART8_CONFIG,
#endif
#ifdef BSP_USING_LPUART1
    LPUART1_CONFIG,
#endif
};

static struct stm32_uart uart_obj[sizeof(uart_config) / sizeof(uart_config[0])] = {0};

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct stm32_uart *uart;
    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    UART_Type *uart_reg;
    uart = (struct stm32_uart *)serial->parent.user_data;
    uart_reg = (UART_Type *)uart->config->Instance;

    uart_reg->USART_CR1 &= ~(1 << 0);
    
    uart_reg->USART_CR1 |= (1 << 3) | (1 << 2);
    uart_reg->USART_CR2 &= ~(3 << 12);

    uart_reg->USART_CR1 |= (1 << 0);

    return RT_EOK;
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct stm32_uart *uart;
    UART_Type *uart_reg;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;
    uart_reg = (UART_Type *)uart->config->Instance;

    switch (cmd)
    {
    /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        uart_reg->USART_CR1 &= ~(1 << 5);
        break;

    /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        uart_reg->USART_CR1 |= (1 << 5);
        rt_hw_interrupt_umask(uart->config->irq_type);
        break;
    }
    return RT_EOK;
}

static int stm32_putc(struct rt_serial_device *serial, char c)
{
    struct stm32_uart *uart;
    UART_Type *uart_reg;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;
    uart_reg = (UART_Type *)uart->config->Instance;

    uart_reg->USART_TDR = c;
    while ((uart_reg->USART_ISR & (1 << 7)) == RESET);

    return 1;    
}

static int stm32_getc(struct rt_serial_device *serial)
{
    int ch;
    struct stm32_uart *uart;
    UART_Type *uart_reg;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;
    uart_reg = (UART_Type *)uart->config->Instance;

    if ((uart_reg->USART_ISR & (1 << 5)) == RESET)
        ch = -1;
    else
        ch = uart_reg->USART_RDR;

    return ch;
}

/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void uart_isr(int irq_type, void* parameter)
{
    struct rt_serial_device *serial;

    rt_interrupt_enter();

    serial = (struct rt_serial_device *)parameter;
    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);

    rt_interrupt_leave();
}

static const struct rt_uart_ops stm32_uart_ops =
{
    .configure = stm32_configure,
    .control = stm32_control,
    .putc = stm32_putc,
    .getc = stm32_getc,
};

int rt_hw_usart_init(void)
{
    struct stm32_uart *uart[sizeof(uart_config) / sizeof(uart_config[0])] = {0};

    rt_size_t obj_num = sizeof(uart_obj) / sizeof(struct stm32_uart);
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t result = 0;

    for (int i = 0; i < obj_num; i++)
    {
        /* init UART object */
        uart_obj[i].config = &uart_config[i];
        uart_obj[i].config->Instance = (rt_uint32_t)rt_ioremap((void *)uart_obj[i].config->Instance, 0x1000);

        uart[i] = &uart_obj[i];

        uart[i]->serial.ops    = &stm32_uart_ops;
        uart[i]->serial.config = config;

        /* register UART device */
        result = rt_hw_serial_register(&uart[i]->serial, uart[i]->config->name,
                                       RT_DEVICE_FLAG_RDWR
                                       | RT_DEVICE_FLAG_INT_RX
                                       | RT_DEVICE_FLAG_STREAM
                                       , uart[i]);

        rt_hw_interrupt_install(uart[i]->config->irq_type, uart_isr, 
                                    &uart[i]->serial, uart[i]->config->name);

        RT_ASSERT(result == RT_EOK);
    }

    return result;
}

#endif /* RT_USING_SERIAL */