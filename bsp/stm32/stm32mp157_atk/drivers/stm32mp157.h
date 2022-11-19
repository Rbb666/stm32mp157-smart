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

#define GIC_IRQ_START 0

#endif