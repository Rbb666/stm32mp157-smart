/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-22     Jesven       first version
 */

#include <rthw.h>
#include <rtthread.h>
#include <stdint.h>

#include "mmu.h"

#define TICK_PERIOD (g_sys_freq / RT_TICK_PER_SECOND)
static int g_sys_freq;

#define IRQ_SECURE_PHY_TIMER 29   /* Secure physical timer event */
#define IRQ_NOSECURE_PHY_TIMER 30 /* No-Secure physical timer event */

#define IRQ_SYS_TICK IRQ_NOSECURE_PHY_TIMER

#define isb() __asm__ __volatile__("" \
                                   :  \
                                   :  \
                                   : "memory")
#define dsb() __asm__ __volatile__("" \
                                   :  \
                                   :  \
                                   : "memory")
#define dmb() __asm__ __volatile__("" \
                                   :  \
                                   :  \
                                   : "memory")

static inline void enable_cntp(void)
{
    rt_uint32_t cntv_ctl;
    cntv_ctl = 1;
    asm volatile("mcr p15, 0, %0, c14, c2, 1" ::"r"(cntv_ctl)); // write CNTP_CTL
    isb();
}

static inline void disable_cntp(void)
{
    rt_uint32_t cntv_ctl;
    cntv_ctl = 0;
    asm volatile("mcr p15, 0, %0, c14, c2, 1" ::"r"(cntv_ctl)); // write CNTP_CTL
    isb();
}

static inline rt_uint32_t read_cntfrq(void)
{
    rt_uint32_t val;
    asm volatile("mrc p15, 0, %0, c14, c0, 0"
                 : "=r"(val));
    return val;
}

static inline void write_cntp_tval(rt_uint32_t val)
{
    asm volatile("mcr p15, 0, %0, c14, c2, 0" ::"r"(val));
    isb();
    return;
}

static inline void write_cntp_cval(rt_uint64_t val)
{
    asm volatile("mcrr p15, 2, %Q0, %R0, c14" ::"r"(val));
    isb();
    return;
}

static inline rt_uint64_t read_cntp_cval(void)
{
    rt_uint64_t val;
    asm volatile("mrrc p15, 2, %Q0, %R0, c14"
                 : "=r"(val));
    return (val);
}

static void system_counter_clk_source_init(void)
{
    /* to do */
}

static void system_counter_init(void)
{
    /* enable system_counter */

    /* stm32mp157, tf-a do that */
}

static void arch_timer_init(void)
{
    g_sys_freq = read_cntfrq();

    /* set timeout val */
    disable_cntp();
    write_cntp_tval(TICK_PERIOD);

    /* start timer */
    enable_cntp();

    /* enable irq */
}

static void rt_hw_timer_isr(int vector, void *param)
{
    rt_tick_increase();

    /* setup for next irq */
    /* clear interrupt */
    disable_cntp();
    write_cntp_cval(read_cntp_cval() + TICK_PERIOD);
    enable_cntp();
}

int rt_hw_timer_init(void)
{
    /* Setup Timer for generating irq */
    /* enable timer */
    system_counter_clk_source_init();
    system_counter_init();
    arch_timer_init();

    /* insall irq, enable irq */
    rt_hw_interrupt_install(IRQ_SYS_TICK, rt_hw_timer_isr, RT_NULL, "tick");
    rt_hw_interrupt_umask(IRQ_SYS_TICK);

    return 0;
}
INIT_BOARD_EXPORT(rt_hw_timer_init);
