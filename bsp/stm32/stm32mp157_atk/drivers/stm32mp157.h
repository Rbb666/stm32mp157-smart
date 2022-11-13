#ifndef __STM32MP157_H__
#define __STM32MP157_H__

#include "stm32mp1xx.h"

/* for 'rt_inline' */
#include <rtdef.h>
/* SOC-relative definitions */
#include "realview.h"

/* the maximum entries of the exception table */
#define MAX_HANDLERS NR_IRQS_PBA8

extern RCC_TypeDef *rcc_addr;
extern GICDistributor_Type *gic_addr;
extern EXTI_Core_TypeDef *exti_c1_addr;
extern EXTI_TypeDef *exti_addr;

/* the basic constants and interfaces needed by gic */
rt_inline rt_uint32_t platform_get_gic_dist_base(void)
{
    return REALVIEW_GIC_DIST_BASE;
}

rt_inline rt_uint32_t platform_get_gic_cpu_base(void)
{
    return REALVIEW_GIC_CPU_BASE;
}

rt_inline void MP1_GIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  uint32_t mask = gic_addr->IPRIORITYR[IRQn / 4U] & ~(0xFFUL << ((IRQn % 4U) * 8U));
  gic_addr->IPRIORITYR[IRQn / 4U] = mask | ((priority & 0xFFUL) << ((IRQn % 4U) * 8U));
}

rt_inline void MP1_GIC_EnableIRQ(IRQn_Type IRQn)
{
  gic_addr->ISENABLER[IRQn / 32U] = 1U << (IRQn % 32U);
}

rt_inline void MP1_GIC_DisableIRQ(IRQn_Type IRQn)
{
  gic_addr->ICENABLER[IRQn / 32U] = 1U << (IRQn % 32U);
}

rt_inline uint32_t MP1_GIC_GetPriority(IRQn_Type IRQn)
{
  return (gic_addr->IPRIORITYR[IRQn / 4U] >> ((IRQn % 4U) * 8U)) & 0xFFUL;
}

#define GIC_IRQ_START 0

#endif