/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-10     Jesven       first version
 * 2021-02-03     lizhirui     port to riscv64
 * 2021-02-19     lizhirui     port to new version of rt-smart
 */

#include "rtconfig.h"

#define __ASSEMBLY__
#include "cpuport.h"
#include "encoding.h"
#include "stackframe.h"

.section      .text.lwp

/*
 * void arch_start_umode(args, text, ustack, kstack);
 */
.global arch_start_umode
.type arch_start_umode, % function
arch_start_umode:
    // load kstack for user process
    csrw sscratch, a3
    li t0, SSTATUS_SPP | SSTATUS_SIE    // set as user mode, close interrupt
    csrc sstatus, t0 
    li t0, SSTATUS_SPIE // enable interrupt when return to user mode
    csrs sstatus, t0

    csrw sepc, a1
    mv a3, a2
    sret//enter user mode

/*
 * void arch_crt_start_umode(args, text, ustack, kstack);
 */
.global arch_crt_start_umode
.type arch_crt_start_umode, % function
arch_crt_start_umode:
    li t0, SSTATUS_SPP | SSTATUS_SIE    // set as user mode, close interrupt
    csrc sstatus, t0
    li t0, SSTATUS_SPIE // enable interrupt when return to user mode
    csrs sstatus, t0
    
    csrw sepc, a1
    mv s0, a0
    mv s1, a1
    mv s2, a2
    mv s3, a3
    mv a0, s2
    call lwp_copy_return_code_to_user_stack
    mv a0, s2
    call lwp_fix_sp
    mv sp, a0//user_sp
    mv ra, a0//return address
    mv a0, s0//args

    csrw sscratch, s3
    sret//enter user mode

.global arch_ret_to_user
arch_ret_to_user:
    call lwp_signal_check
    beqz a0, ret_to_user_exit
    // now sp is user sp
    J user_do_signal

ret_to_user_exit:
    RESTORE_ALL
    // `RESTORE_ALL` also reset sp to user sp
    sret

/*#ifdef RT_USING_LWP
.global lwp_check_exit
lwp_check_exit:
    push {r0 - r12, lr}
    bl lwp_check_exit_request
    cmp r0, #0
    beq 1f
    mov r0, #0
    bl sys_exit
1:
    pop {r0 - r12, pc}
#endif*/

/*#ifdef RT_USING_GDBSERVER
.global lwp_check_debug
lwp_check_debug:
    push {r0 - r12, lr}
    bl lwp_check_debug_suspend
    cmp r0, #0
    beq lwp_check_debug_quit

    cps #Mode_SYS
    sub sp, #8
    ldr r0, =lwp_debugreturn
    ldr r1, [r0]
    str r1, [sp]
    ldr r1, [r0, #4]
    str r1, [sp, #4]
    mov r0, #0
    mcr p15, 0, r0, c7, c5, 0   ;//iciallu
    dsb
    isb
    mov r0, sp // lwp_debugreturn
    cps #Mode_SVC

    mrs r1, spsr
    push {r1}
    mov r1, #Mode_USR
    msr spsr_cxsf, r1
    movs pc, r0
ret_from_user:
    cps #Mode_SYS
    add sp, #8
    cps #Mode_SVC*/
    /*
    pop {r0 - r3, r12}
    pop {r4 - r6, lr}
    */
    /*add sp, #(4*9)
    pop {r4}
    msr spsr_cxsf, r4
lwp_check_debug_quit:
    pop {r0 - r12, pc}
//#endif
*/

arch_signal_quit:
    call lwp_signal_restore
    //a0 is user_ctx
    mv sp, a0
    RESTORE_ALL
    csrw sscratch, zero
    sret

user_do_signal:
    csrw sscratch, sp
    RESTORE_ALL
    // now sp is user sp
    // and in interrupt close
    SAVE_ALL

    // save user sp in SAVE_ALL frame
    mv t0, sp
    addi t0, t0, CTX_REG_NR * REGBYTES
    STORE t0, 32 * REGBYTES(sp)

    // save lwp_sigreturn in user memory
    mv s0, sp
    la t0, lwp_sigreturn//t0 = src
    la t1, lwp_sigreturn_end
    sub t1, t1, t0//t1 = size
    sub s0, s0, t1//s0 = dst

lwp_sigreturn_copy_loop:
    addi t2, t1, -1//t2 = memory index
    add t3, t0, t2//t3 = src addr
    add t4, s0, t2//t4 = dst addr
    lb t5, 0(t3)
    sb t5, 0(t4)
    mv t1, t2
    bnez t1, lwp_sigreturn_copy_loop

    // restore kernel stack
    csrrw sp, sscratch, s0

    /**
     * a0: user sp
     * a1: user_pc (not used)
     * a2: user_flag (not used)
     */ 
    csrr a0, sscratch
    mv a1, zero
    mv a2, zero
    call lwp_signal_backup
    // a0 <- signal id

    // restore kernel sp to initial, and load `sp` to user stack

    // s2 <- signal id(a0)
    mv s2, a0
    call lwp_sighandler_get
    // a0 <- signal_handler

    // ra <- lwp_sigreturn
    mv ra, s0

    mv s1, s0
    beqz a0, skip_user_signal_handler
    // a0 <- signal_handler
    mv s1, a0

skip_user_signal_handler:
    // enter user mode and enable interrupt when return to user mode
    li t0, SSTATUS_SPP
    csrc sstatus, t0 
    li t0, SSTATUS_SPIE
    csrs sstatus, t0

    /**
     * sp <- user sp
     * sscratch <- kernel sp
     */
    csrrw sp, sscratch, sp

    // sepc <- signal_handler
    csrw sepc, s1
    // a0 <- signal id
    mv a0, s2
    sret

.align 3
lwp_debugreturn:
    li a7, 0xff
    ecall

.align 3
lwp_sigreturn:
    li a7, 0xfe
    ecall

.align 3
lwp_sigreturn_end:

.align 3
.global lwp_thread_return
lwp_thread_return:
    li a0, 0
    li a7, 1
    ecall

.align 3
.global lwp_thread_return_end
lwp_thread_return_end:

.global check_vfp
check_vfp:
    //don't use fpu temporarily
    li a0, 0
    ret

.global get_vfp
get_vfp:
    //don't use fpu temporarily
    li a0, 0
    ret

.globl arch_get_tidr
arch_get_tidr:
    mv a0, tp 
    ret

.global arch_set_thread_area
arch_set_thread_area:
.globl arch_set_tidr
arch_set_tidr:
    mv tp, a0
    ret

.global arch_clone_exit
.global arch_fork_exit
arch_fork_exit:
arch_clone_exit:
    j arch_syscall_exit

.global syscall_entry
syscall_entry:
#ifndef ARCH_USING_NEW_CTX_SWITCH
    //swap to thread kernel stack
    csrr t0, sstatus
    andi t0, t0, 0x100
    beqz t0, __restore_sp_from_tcb

__restore_sp_from_sscratch:
    csrr t0, sscratch
    j __move_stack_context

__restore_sp_from_tcb:
    la a0, rt_current_thread
    LOAD a0, 0(a0)
    jal get_thread_kernel_stack_top
    mv t0, a0

__move_stack_context:
    mv t1, sp//src
    mv sp, t0//switch stack
    addi sp, sp, -CTX_REG_NR * REGBYTES
    //copy context
    li s0, CTX_REG_NR//cnt
    mv t2, sp//dst

copy_context_loop:
    LOAD t0, 0(t1)
    STORE t0, 0(t2)
    addi s0, s0, -1
    addi t1, t1, 8
    addi t2, t2, 8
    bnez s0, copy_context_loop
#endif /* ARCH_USING_NEW_CTX_SWITCH */
    LOAD s0, 7 * REGBYTES(sp)
    addi s0, s0, -0xfe
    beqz s0, arch_signal_quit

#ifdef RT_USING_USERSPACE
    /* save setting when syscall enter */
    call  rt_thread_self
    call  lwp_user_setting_save
#endif

    mv a0, sp
    OPEN_INTERRUPT
    call syscall_handler
    j arch_syscall_exit
    
.global arch_syscall_exit
arch_syscall_exit:
    CLOSE_INTERRUPT

    #if defined(RT_USING_USERSPACE)
        LOAD s0, 2 * REGBYTES(sp)
        andi s0, s0, 0x100
        bnez s0, dont_ret_to_user
        j arch_ret_to_user
    #endif
dont_ret_to_user:

#ifdef RT_USING_USERSPACE
    /* restore setting when syscall exit */
    call  rt_thread_self
    call  lwp_user_setting_restore

    /* after restore the reg `tp`, need modify context */
    STORE tp, 4 * REGBYTES(sp)
#endif

    //restore context
    RESTORE_ALL
    csrw sscratch, zero
    sret

