/*******************************************************************************
 * Copyright 2019 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * @file mss_plic.c
 * @author Microchip FPGA Embedded Systems Solutions
 * @brief PolarFire SoC MSS PLIC and PRCI access data structures and functions.
 *
 * PLIC related data which cannot be placed in mss_plic.h
 *
 */

#include "mpfs_hal/mss_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

    const unsigned long plic_hart_lookup[5U] = {0U, 1U, 3U, 5U, 7U};

#ifdef __cplusplus
}
#endif

void PLIC_init(void)
{
    PLIC_init_static();
}

void PLIC_EnableIRQ(PLIC_IRQn_Type IRQn)
{
    PLIC_EnableIRQ_static(IRQn);
}

void PLIC_EnableIRQ_for_hart(PLIC_IRQn_Type IRQn, uint64_t hart_id)
{
    PLIC_EnableIRQ_for_hart_static(IRQn, hart_id);
}

void PLIC_DisableIRQ(PLIC_IRQn_Type IRQn)
{
    PLIC_DisableIRQ_static(IRQn);
}

void PLIC_SetPriority(PLIC_IRQn_Type IRQn, uint32_t priority)
{
    PLIC_SetPriority_static(IRQn, priority);
}

uint32_t PLIC_GetPriority(PLIC_IRQn_Type IRQn)
{
    return PLIC_GetPriority_static(IRQn);
}

uint32_t PLIC_pending(PLIC_IRQn_Type IRQn)
{
    return PLIC_pending_static(IRQn);
}

uint32_t PLIC_ClaimIRQ(void)
{
    return PLIC_ClaimIRQ_static();
}

void PLIC_CompleteIRQ(uint32_t source)
{
    PLIC_CompleteIRQ_static(source);
}

void PLIC_SetPriority_Threshold(uint32_t threshold)
{
    PLIC_SetPriority_Threshold_static(threshold);
}

void PLIC_ClearPendingIRQ(void)
{
    PLIC_ClearPendingIRQ_static();
}

void PLIC_init_on_reset(void)
{
    PLIC_init_on_reset_static();
}
