/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-11-20     Bernard    the first version
 * 2018-11-22     Jesven     add rt_hw_spin_lock
 *                           add rt_hw_spin_unlock
 *                           add smp ipi init
 */

#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "drv_timer.h"

#include <mmu.h>
#ifdef RT_USING_USERSPACE
#include <page.h>
#include <lwp_arch.h>
#endif

#ifdef RT_USING_SERIAL
#ifdef RT_USING_SERIAL_V2
#include "drv_usart_v2.h"
#else
#include "drv_usart.h"
#endif /* RT_USING_SERIAL */
#endif /* RT_USING_SERIAL_V2 */

RCC_TypeDef *rcc_addr;
GICDistributor_Type *gic_addr;
EXTI_Core_TypeDef *exti_c1_addr;
EXTI_TypeDef *exti_addr;

#ifdef RT_USING_USERSPACE
struct mem_desc platform_mem_desc[] = {
    {KERNEL_VADDR_START, KERNEL_VADDR_START + 0x07FFFFFF, KERNEL_VADDR_START + PV_OFFSET, NORMAL_MEM}};
#else
struct mem_desc platform_mem_desc[] = {
    {0x10000000, 0x50000000, 0x10000000, DEVICE_MEM},
    {0x60000000, 0x70000000, 0x60000000, NORMAL_MEM}};
#endif

const rt_uint32_t platform_mem_desc_size = sizeof(platform_mem_desc) / sizeof(platform_mem_desc[0]);

#define SYS_CTRL __REG32(REALVIEW_SCTL_BASE)

extern void rt_hw_ipi_handler_install(int ipi_vector, rt_isr_handler_t ipi_isr_handler);

void idle_wfi(void)
{
    asm volatile("wfi");
}

/**
 * This function will initialize board
 */

rt_mmu_info mmu_info;

extern size_t MMUTable[];

#ifdef RT_USING_USERSPACE
rt_region_t init_page_region = {
    (uint32_t)PAGE_START,
    (uint32_t)PAGE_END,
};
#endif

void rt_hw_board_init(void)
{
#ifdef RT_USING_USERSPACE
    rt_hw_mmu_map_init(&mmu_info, (void *)0xf0000000, 0x10000000, MMUTable, PV_OFFSET);

    rt_page_init(init_page_region);
    rt_hw_mmu_ioremap_init(&mmu_info, (void *)0xf0000000, 0x10000000);

    arch_kuser_init(&mmu_info, (void *)0xffff0000);
#else
    rt_hw_mmu_map_init(&mmu_info, (void *)0x80000000, 0x10000000, MMUTable, 0);
    rt_hw_mmu_ioremap_init(&mmu_info, (void *)0x80000000, 0x10000000);
#endif
    /* initialize hardware interrupt */
    rt_hw_interrupt_init();

    /* initialize system heap */
    rt_system_heap_init(HEAP_BEGIN, HEAP_END);

    /* RCC VER BASE ADDR */
    rcc_addr = (RCC_TypeDef *)rt_ioremap((void *)RCC, sizeof(sizeof(rt_uint32_t)));

    // gic_addr = (GICDistributor_Type *)rt_ioremap((void *)GIC_DISTRIBUTOR_BASE, sizeof(sizeof(rt_uint32_t)));

    exti_c1_addr = (EXTI_Core_TypeDef *)rt_ioremap((void *)EXTI_C1, sizeof(sizeof(rt_uint32_t)));

    exti_addr = (EXTI_TypeDef *)rt_ioremap((void *)EXTI, sizeof(sizeof(rt_uint32_t)));

    /* USART driver initialization is open by default */
#ifdef RT_USING_SERIAL
    rt_hw_usart_init();
#endif

#ifdef RT_USING_PIN
    rt_hw_pin_init();
#endif

    rt_components_board_init();

    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);

    rt_thread_idle_sethook(idle_wfi);

    rt_kprintf("heap: 0x%08x - 0x%08x\n", HEAP_BEGIN, HEAP_END);

#ifdef RT_USING_SMP
    /* install IPI handle */
    rt_hw_ipi_handler_install(RT_SCHEDULE_IPI, rt_scheduler_ipi_handler);
#endif
}
