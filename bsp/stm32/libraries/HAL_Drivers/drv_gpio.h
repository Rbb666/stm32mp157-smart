/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2022-11-13     Rbb666            first version
 */

#ifndef __DRV_GPIO_H__
#define __DRV_GPIO_H__

#include <drv_common.h>
#include <board.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define __STM32_PORT(port) GPIO##port##_BASE

#define GET_PIN(PORTx, PIN) (GPIO##PORTx == GPIOZ) ? (176 + PIN) : ((rt_base_t)((16 * (((rt_base_t)__STM32_PORT(PORTx) - (rt_base_t)GPIOA_BASE) / (0x1000UL))) + PIN))

#define GET_GPIO_INDEX(__GPIOx__)   (rt_uint8_t)(((__GPIOx__) == (mask_tab[0].gpio_port))? 0U :\
                                              ((__GPIOx__) == (mask_tab[1].gpio_port))? 1U :\
                                              ((__GPIOx__) == (mask_tab[2].gpio_port))? 2U :\
                                              ((__GPIOx__) == (mask_tab[3].gpio_port))? 3U :\
                                              ((__GPIOx__) == (mask_tab[4].gpio_port))? 4U :\
                                              ((__GPIOx__) == (mask_tab[5].gpio_port))? 5U :\
                                              ((__GPIOx__) == (mask_tab[6].gpio_port))? 6U :\
                                              ((__GPIOx__) == (mask_tab[7].gpio_port))? 7U :\
                                              ((__GPIOx__) == (mask_tab[8].gpio_port))? 8U :\
                                              ((__GPIOx__) == (mask_tab[9].gpio_port))? 9U :\
                                              ((__GPIOx__) == (mask_tab[10].gpio_port))? 10U :\
                                              ((__GPIOx__) == (mask_tab[11].gpio_port))? 11U : 25U)

#define __GPIO_EXTI_GET_RISING_IT(__EXTI_LINE__) (exti_addr->RPR1 & (__EXTI_LINE__))

#define __GPIO_EXTI_CLEAR_RISING_IT(__EXTI_LINE__) \
    do                                             \
    {                                              \
        exti_addr->RPR1 = (__EXTI_LINE__);         \
    } while (0);

#define __GPIO_EXTI_GET_FALLING_IT(__EXTI_LINE__) (exti_addr->FPR1 & (__EXTI_LINE__))

#define __GPIO_EXTI_CLEAR_FALLING_IT(__EXTI_LINE__) \
    do                                              \
    {                                               \
        exti_addr->FPR1 = (__EXTI_LINE__);          \
    } while (0);

#define GPIO_MODE           ((rt_uint32_t)0x00000003)
#define EXTI_MODE           ((rt_uint32_t)0x10000000)
#define GPIO_MODE_IT        ((rt_uint32_t)0x00010000)
#define GPIO_MODE_EVT       ((rt_uint32_t)0x00020000)
#define RISING_EDGE         ((rt_uint32_t)0x00100000)
#define FALLING_EDGE        ((rt_uint32_t)0x00200000)
#define GPIO_OUTPUT_TYPE    ((rt_uint32_t)0x00000010)

#define GPIO_NUMBER ((rt_uint32_t)16)

struct pin_irq_map
{
    rt_uint16_t pinbit;
    IRQn_Type irqno;
};

struct pin_mask
{
    GPIO_TypeDef *gpio_port;
    rt_uint32_t gpio_clock;
};

int rt_hw_pin_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_GPIO_H__ */
