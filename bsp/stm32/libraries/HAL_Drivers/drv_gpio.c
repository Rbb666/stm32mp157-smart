/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2022-11-13     Rbb666            first version
 */

#include <board.h>
#include "drv_gpio.h"
#include "stm32mp1xx.h"

#ifdef RT_USING_PIN

#define PIN_NUM(port, no) (((((port) & 0xFu) << 4) | ((no) & 0xFu)))
#define PIN_PORT(pin) ((rt_uint8_t)(((pin) >> 4) & 0xFu))
#define PIN_NO(pin) ((rt_uint8_t)((pin) & 0xFu))

#define PIN_STPIN(pin) ((rt_uint16_t)(1u << PIN_NO(pin)))

#define __STM32_PORT_MAX 12u

#define PIN_STPORT_MAX __STM32_PORT_MAX

struct pin_mask mask_tab[] =
{
    {GPIOA, RCC_MC_AHB4ENSETR_GPIOAEN},
    {GPIOB, RCC_MC_AHB4ENSETR_GPIOBEN},
    {GPIOC, RCC_MC_AHB4ENSETR_GPIOCEN},
    {GPIOD, RCC_MC_AHB4ENSETR_GPIODEN},
    {GPIOE, RCC_MC_AHB4ENSETR_GPIOEEN},
    {GPIOF, RCC_MC_AHB4ENSETR_GPIOFEN},
    {GPIOG, RCC_MC_AHB4ENSETR_GPIOGEN},
    {GPIOH, RCC_MC_AHB4ENSETR_GPIOHEN},
    {GPIOI, RCC_MC_AHB4ENSETR_GPIOIEN},
    {GPIOJ, RCC_MC_AHB4ENSETR_GPIOJEN},
    {GPIOK, RCC_MC_AHB4ENSETR_GPIOKEN},
};

static const struct pin_irq_map pin_irq_map[] =
{
    {GPIO_PIN_0, EXTI0_IRQn},
    {GPIO_PIN_1, EXTI1_IRQn},
    {GPIO_PIN_2, EXTI2_IRQn},
    {GPIO_PIN_3, EXTI3_IRQn},
    {GPIO_PIN_4, EXTI4_IRQn},
    {GPIO_PIN_5, EXTI5_IRQn},
    {GPIO_PIN_6, EXTI6_IRQn},
    {GPIO_PIN_7, EXTI7_IRQn},
    {GPIO_PIN_8, EXTI8_IRQn},
    {GPIO_PIN_9, EXTI9_IRQn},
    {GPIO_PIN_10, EXTI10_IRQn},
    {GPIO_PIN_11, EXTI11_IRQn},
    {GPIO_PIN_12, EXTI12_IRQn},
    {GPIO_PIN_13, EXTI13_IRQn},
    {GPIO_PIN_14, EXTI14_IRQn},
    {GPIO_PIN_15, EXTI15_IRQn},
};

static struct rt_pin_irq_hdr pin_irq_hdr_tab[] =
{
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
};

static rt_uint32_t pin_irq_enable_mask = 0;

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])

static rt_base_t stm32_pin_get(const char *name)
{
    rt_base_t pin = 0;
    int hw_port_num, hw_pin_num = 0;
    int i, name_len;

    name_len = rt_strlen(name);

    if ((name_len < 4) || (name_len >= 6))
    {
        return -RT_EINVAL;
    }
    if ((name[0] != 'P') || (name[2] != '.'))
    {
        return -RT_EINVAL;
    }

    if ((name[1] >= 'A') && (name[1] <= 'Z'))
    {
        hw_port_num = (int)(name[1] - 'A');
    }
    else
    {
        return -RT_EINVAL;
    }

    for (i = 3; i < name_len; i++)
    {
        hw_pin_num *= 10;
        hw_pin_num += name[i] - '0';
    }

    pin = PIN_NUM(hw_port_num, hw_pin_num);

    return pin;
}

static void stm32_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    GPIO_TypeDef *gpio_port;
    rt_uint16_t gpio_pin;

    if (PIN_PORT(pin) < PIN_STPORT_MAX)
    {
        gpio_port = mask_tab[PIN_PORT(pin)].gpio_port;
        gpio_pin = PIN_STPIN(pin);

        HAL_GPIO_WritePin(gpio_port, gpio_pin, (GPIO_PinState)value);
    }
}

static int stm32_pin_read(rt_device_t dev, rt_base_t pin)
{
    GPIO_TypeDef *gpio_port;
    rt_uint16_t gpio_pin;
    int value = PIN_LOW;

    if (PIN_PORT(pin) < PIN_STPORT_MAX)
    {
        gpio_port = mask_tab[PIN_PORT(pin)].gpio_port;
        gpio_pin = PIN_STPIN(pin);
        value = HAL_GPIO_ReadPin(gpio_port, gpio_pin);
    }

    return value;
}

static void hal_gpio_init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init, EXTI_Core_TypeDef *EXTICx, EXTI_TypeDef *EXTIx)
{
  rt_uint32_t position;
  rt_uint32_t ioposition;
  rt_uint32_t iocurrent;
  rt_uint32_t temp;
  EXTI_Core_TypeDef * EXTI_CurrentCPU;

  EXTI_CurrentCPU = EXTICx; /* EXTI for CA7 CPU */

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Init->Pin));
  assert_param(IS_GPIO_MODE(GPIO_Init->Mode));
  assert_param(IS_GPIO_PULL(GPIO_Init->Pull));

  /* Configure the port pins */
  for(position = 0; position < GPIO_NUMBER; position++)
  {
    /* Get the IO position */
    ioposition = ((rt_uint32_t)0x01) << position;
    /* Get the current IO position */
    iocurrent = (rt_uint32_t)(GPIO_Init->Pin) & ioposition;

    if(iocurrent == ioposition)
    {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Alternate function mode selection */
      if((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
      {
        /* Check the Alternate function parameter */
        assert_param(IS_GPIO_AF(GPIO_Init->Alternate));

        /* Configure Alternate function mapped with the current IO */
        temp = GPIOx->AFR[position >> 3];
        temp &= ~((rt_uint32_t)0xF << ((rt_uint32_t)(position & (rt_uint32_t)0x07) * 4)) ;
        temp |= ((rt_uint32_t)(GPIO_Init->Alternate) << (((rt_uint32_t)position & (rt_uint32_t)0x07) * 4));
        GPIOx->AFR[position >> 3] = temp;
      }

      /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
      temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODER0 << (position * 2));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2));
      GPIOx->MODER = temp;

      /* In case of Output or Alternate function mode selection */
      if((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP) ||
         (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
      {
        /* Check the Speed parameter */
        assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
        /* Configure the IO Speed */
        temp = GPIOx->OSPEEDR;
        temp &= ~(GPIO_OSPEEDR_OSPEEDR0 << (position * 2));
        temp |= (GPIO_Init->Speed << (position * 2));
        GPIOx->OSPEEDR = temp;

        /* Configure the IO Output Type */
        temp = GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT0 << position) ;
        temp |= (((GPIO_Init->Mode & GPIO_OUTPUT_TYPE) >> 4) << position);
        GPIOx->OTYPER = temp;
      }

      /* Activate the Pull-up or Pull down resistor for the current IO */
      temp = GPIOx->PUPDR;
      temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2));
      temp |= ((GPIO_Init->Pull) << (position * 2));
      GPIOx->PUPDR = temp;

      /*--------------------- EXTI Mode Configuration ------------------------*/
      /* Configure the External Interrupt or event for the current IO */
      if((GPIO_Init->Mode & EXTI_MODE) == EXTI_MODE)
      {
        temp = EXTIx->EXTICR[position >> 2U];
        temp &= ~(0xFFU << (8U * (position & 0x03U)));
        temp |= (GET_GPIO_INDEX(GPIOx) << (8U * (position & 0x03U)));
        EXTIx->EXTICR[position >> 2U] = temp;
        /* Clear EXTI line configuration */
        temp = EXTI_CurrentCPU->IMR1;
        temp &= ~((rt_uint32_t)iocurrent);
        if((GPIO_Init->Mode & GPIO_MODE_IT) == GPIO_MODE_IT)
        {
          temp |= iocurrent;
        }
        EXTI_CurrentCPU->IMR1 = temp;

        temp = EXTI_CurrentCPU->EMR1;
        temp &= ~((rt_uint32_t)iocurrent);
        if((GPIO_Init->Mode & GPIO_MODE_EVT) == GPIO_MODE_EVT)
        {
          temp |= iocurrent;
        }
        EXTI_CurrentCPU->EMR1 = temp;

        /* Clear Rising Falling edge configuration */
        temp = EXTIx->RTSR1;
        temp &= ~((rt_uint32_t)iocurrent);
        if((GPIO_Init->Mode & RISING_EDGE) == RISING_EDGE)
        {
          temp |= iocurrent;
        }
        EXTIx->RTSR1 = temp;

        temp = EXTIx->FTSR1;
        temp &= ~((rt_uint32_t)iocurrent);
        if((GPIO_Init->Mode & FALLING_EDGE) == FALLING_EDGE)
        {
          temp |= iocurrent;
        }
        EXTIx->FTSR1 = temp;
      }
    }
  }
}

static void stm32_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if (PIN_PORT(pin) >= PIN_STPORT_MAX)
    {
        return;
    }

    /* Configure GPIO_InitStructure */
    GPIO_InitStruct.Pin = PIN_STPIN(pin);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    if (mode == PIN_MODE_OUTPUT)
    {
        /* output setting */
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    }
    else if (mode == PIN_MODE_INPUT)
    {
        /* input setting: not pull. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    }
    else if (mode == PIN_MODE_INPUT_PULLUP)
    {
        /* input setting: pull up. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
    }
    else if (mode == PIN_MODE_INPUT_PULLDOWN)
    {
        /* input setting: pull down. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    }
    else if (mode == PIN_MODE_OUTPUT_OD)
    {
        /* output setting: od. */
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    }

    hal_gpio_init(mask_tab[PIN_PORT(pin)].gpio_port, &GPIO_InitStruct, RT_NULL, RT_NULL);
}

rt_inline rt_int32_t bit2bitno(rt_uint32_t bit)
{
    rt_uint8_t i;
    for (i = 0; i < 32; i++)
    {
        if ((0x01 << i) == bit)
        {
            return i;
        }
    }
    return -1;
}

rt_inline const struct pin_irq_map *get_pin_irq_map(rt_uint32_t pinbit)
{
    rt_int32_t mapindex = bit2bitno(pinbit);
    if (mapindex < 0 || mapindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_NULL;
    }
    return &pin_irq_map[mapindex];
};

static rt_err_t stm32_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                                     rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    rt_base_t level;
    rt_int32_t irqindex = -1;

    if (PIN_PORT(pin) >= PIN_STPORT_MAX)
    {
        return -RT_ENOSYS;
    }

    irqindex = bit2bitno(PIN_STPIN(pin));
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_ENOSYS;
    }

    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[irqindex].pin == pin &&
        pin_irq_hdr_tab[irqindex].hdr == hdr &&
        pin_irq_hdr_tab[irqindex].mode == mode &&
        pin_irq_hdr_tab[irqindex].args == args)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    if (pin_irq_hdr_tab[irqindex].pin != -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EBUSY;
    }
    pin_irq_hdr_tab[irqindex].pin = pin;
    pin_irq_hdr_tab[irqindex].hdr = hdr;
    pin_irq_hdr_tab[irqindex].mode = mode;
    pin_irq_hdr_tab[irqindex].args = args;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t stm32_pin_dettach_irq(struct rt_device *device, rt_int32_t pin)
{
    rt_base_t level;
    rt_int32_t irqindex = -1;

    if (PIN_PORT(pin) >= PIN_STPORT_MAX)
    {
        return -RT_ENOSYS;
    }

    irqindex = bit2bitno(PIN_STPIN(pin));
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_ENOSYS;
    }

    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[irqindex].pin == -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    pin_irq_hdr_tab[irqindex].pin = -1;
    pin_irq_hdr_tab[irqindex].hdr = RT_NULL;
    pin_irq_hdr_tab[irqindex].mode = 0;
    pin_irq_hdr_tab[irqindex].args = RT_NULL;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t stm32_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                                     rt_uint32_t enabled)
{
    const struct pin_irq_map *irqmap;
    rt_base_t level;
    rt_int32_t irqindex = -1;
    GPIO_InitTypeDef GPIO_InitStruct;

    if (PIN_PORT(pin) >= PIN_STPORT_MAX)
    {
        return -RT_ENOSYS;
    }

    if (enabled == PIN_IRQ_ENABLE)
    {
        irqindex = bit2bitno(PIN_STPIN(pin));
        if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
        {
            return RT_ENOSYS;
        }

        level = rt_hw_interrupt_disable();

        rt_kprintf("pin_irq_hdr_tab_pin:%d\n", pin_irq_hdr_tab[irqindex].pin);

        if (pin_irq_hdr_tab[irqindex].pin == -1)
        {
            rt_hw_interrupt_enable(level);
            return RT_ENOSYS;
        }

        irqmap = &pin_irq_map[irqindex];

        /* Configure GPIO_InitStructure */
        GPIO_InitStruct.Pin = PIN_STPIN(pin);
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        switch (pin_irq_hdr_tab[irqindex].mode)
        {
        case PIN_IRQ_MODE_RISING:
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
            break;
        case PIN_IRQ_MODE_FALLING:
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
            break;
        case PIN_IRQ_MODE_RISING_FALLING:
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
            break;
        }

        hal_gpio_init(mask_tab[PIN_PORT(pin)].gpio_port, &GPIO_InitStruct, exti_c1_addr, exti_addr);

        pin_irq_enable_mask |= irqmap->pinbit;

        rt_hw_interrupt_enable(level);
    }
    else if (enabled == PIN_IRQ_DISABLE)
    {
        irqmap = get_pin_irq_map(PIN_STPIN(pin));
        if (irqmap == RT_NULL)
        {
            return RT_ENOSYS;
        }

        level = rt_hw_interrupt_disable();

        HAL_GPIO_DeInit(mask_tab[PIN_PORT(pin)].gpio_port, PIN_STPIN(pin));

        pin_irq_enable_mask &= ~irqmap->pinbit;

        if ((irqmap->pinbit >= GPIO_PIN_5) && (irqmap->pinbit <= GPIO_PIN_9))
        {
            if (!(pin_irq_enable_mask & (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9)))
            {
                // MP1_GIC_DisableIRQ(irqmap->irqno);
            }
        }
        else if ((irqmap->pinbit >= GPIO_PIN_10) && (irqmap->pinbit <= GPIO_PIN_15))
        {
            if (!(pin_irq_enable_mask & (GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)))
            {
                // MP1_GIC_DisableIRQ(irqmap->irqno);
            }
        }
        else
        {
            // MP1_GIC_DisableIRQ(irqmap->irqno);
        }
        rt_hw_interrupt_enable(level);
    }
    else
    {
        return -RT_ENOSYS;
    }

    return RT_EOK;
}

const static struct rt_pin_ops _stm32_pin_ops =
{
    stm32_pin_mode,
    stm32_pin_write,
    stm32_pin_read,
    stm32_pin_attach_irq,
    stm32_pin_dettach_irq,
    stm32_pin_irq_enable,
    stm32_pin_get,
};

rt_inline void pin_irq_hdr(rt_uint16_t irqno)
{
    if (pin_irq_hdr_tab[irqno].hdr)
    {
        pin_irq_hdr_tab[irqno].hdr(pin_irq_hdr_tab[irqno].args);
    }
}

static void GPIO_EXTI_Rising_Callback(rt_uint16_t GPIO_Pin)
{
    pin_irq_hdr(bit2bitno(GPIO_Pin));
}

static void GPIO_EXTI_Falling_Callback(rt_uint16_t GPIO_Pin)
{
    pin_irq_hdr(bit2bitno(GPIO_Pin));
}

static void GPIO_EXTI_IRQHandler(rt_uint16_t GPIO_Pin)
{
  /* EXTI line interrupt detected */
  if (__GPIO_EXTI_GET_RISING_IT(GPIO_Pin) != RESET)
  {
    __GPIO_EXTI_CLEAR_RISING_IT(GPIO_Pin);
    GPIO_EXTI_Rising_Callback(GPIO_Pin);
  }

  if (__GPIO_EXTI_GET_FALLING_IT(GPIO_Pin) != RESET)
  {
    __GPIO_EXTI_CLEAR_FALLING_IT(GPIO_Pin);
    GPIO_EXTI_Falling_Callback(GPIO_Pin);
  }
}

void EXTI0_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    rt_interrupt_leave();
}

void EXTI1_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    rt_interrupt_leave();
}

void EXTI2_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    rt_interrupt_leave();
}

void EXTI3_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    rt_interrupt_leave();
}

void EXTI4_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_4);
    rt_interrupt_leave();
}

void EXTI5_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    rt_interrupt_leave();
}

void EXTI6_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    rt_interrupt_leave();
}

void EXTI7_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    rt_interrupt_leave();
}

void EXTI8_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    rt_interrupt_leave();
}

void EXTI9_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_9);
    rt_interrupt_leave();
}

void EXTI10_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    rt_interrupt_leave();
}

void EXTI11_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    rt_interrupt_leave();
}

void EXTI12_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    rt_interrupt_leave();
}

void EXTI13_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    rt_interrupt_leave();
}

void EXTI14_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    rt_interrupt_leave();
}

void EXTI15_IRQHandler(int irqno, void *param)
{
    rt_interrupt_enter();
    GPIO_EXTI_IRQHandler(GPIO_PIN_15);
    rt_interrupt_leave();
}

void *stm32_get_periph_vaddr(rt_uint32_t paddr)
{
    return rt_ioremap((void *)paddr, sizeof(sizeof(rt_uint32_t)));
}

static void stm32_pin_interrupt_install(void)
{
    rt_hw_interrupt_install(pin_irq_map[0].irqno, EXTI0_IRQHandler, RT_NULL, "GPIO_0");
    rt_hw_interrupt_install(pin_irq_map[1].irqno, EXTI1_IRQHandler, RT_NULL, "GPIO_1");
    rt_hw_interrupt_install(pin_irq_map[2].irqno, EXTI2_IRQHandler, RT_NULL, "GPIO_2");
    rt_hw_interrupt_install(pin_irq_map[3].irqno, EXTI3_IRQHandler, RT_NULL, "GPIO_3");
    rt_hw_interrupt_install(pin_irq_map[4].irqno, EXTI4_IRQHandler, RT_NULL, "GPIO_4");
    rt_hw_interrupt_install(pin_irq_map[5].irqno, EXTI5_IRQHandler, RT_NULL, "GPIO_5");
    rt_hw_interrupt_install(pin_irq_map[6].irqno, EXTI6_IRQHandler, RT_NULL, "GPIO_6");
    rt_hw_interrupt_install(pin_irq_map[7].irqno, EXTI7_IRQHandler, RT_NULL, "GPIO_7");
    rt_hw_interrupt_install(pin_irq_map[8].irqno, EXTI8_IRQHandler, RT_NULL, "GPIO_8");
    rt_hw_interrupt_install(pin_irq_map[9].irqno, EXTI9_IRQHandler, RT_NULL, "GPIO_9");
    rt_hw_interrupt_install(pin_irq_map[10].irqno, EXTI10_IRQHandler, RT_NULL, "GPIO_10");
    rt_hw_interrupt_install(pin_irq_map[11].irqno, EXTI11_IRQHandler, RT_NULL, "GPIO_11");
    rt_hw_interrupt_install(pin_irq_map[12].irqno, EXTI12_IRQHandler, RT_NULL, "GPIO_12");
    rt_hw_interrupt_install(pin_irq_map[13].irqno, EXTI13_IRQHandler, RT_NULL, "GPIO_13");
    rt_hw_interrupt_install(pin_irq_map[14].irqno, EXTI14_IRQHandler, RT_NULL, "GPIO_14");
    rt_hw_interrupt_install(pin_irq_map[15].irqno, EXTI15_IRQHandler, RT_NULL, "GPIO_15");

    rt_hw_interrupt_umask(pin_irq_map[0].irqno);
    rt_hw_interrupt_umask(pin_irq_map[1].irqno);
    rt_hw_interrupt_umask(pin_irq_map[2].irqno);
    rt_hw_interrupt_umask(pin_irq_map[3].irqno);
    rt_hw_interrupt_umask(pin_irq_map[4].irqno);
    rt_hw_interrupt_umask(pin_irq_map[5].irqno);
    rt_hw_interrupt_umask(pin_irq_map[6].irqno);
    rt_hw_interrupt_umask(pin_irq_map[7].irqno);
    rt_hw_interrupt_umask(pin_irq_map[8].irqno);
    rt_hw_interrupt_umask(pin_irq_map[9].irqno);
    rt_hw_interrupt_umask(pin_irq_map[10].irqno);
    rt_hw_interrupt_umask(pin_irq_map[11].irqno);
    rt_hw_interrupt_umask(pin_irq_map[12].irqno);
    rt_hw_interrupt_umask(pin_irq_map[13].irqno);
    rt_hw_interrupt_umask(pin_irq_map[14].irqno);
    rt_hw_interrupt_umask(pin_irq_map[15].irqno);
}

int rt_hw_pin_init(void)
{
    for(int i = 0; i < sizeof(mask_tab) / sizeof(mask_tab[0]); i++)
    {
        mask_tab[i].gpio_port = (GPIO_TypeDef *)stm32_get_periph_vaddr((rt_uint32_t)mask_tab[i].gpio_port);
        rcc_addr->MP_AHB4ENSETR = mask_tab[i].gpio_clock;
    }

    stm32_pin_interrupt_install();

    return rt_device_pin_register("pin", &_stm32_pin_ops, RT_NULL);
}

#endif /* RT_USING_PIN */
