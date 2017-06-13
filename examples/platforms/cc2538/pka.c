/* vim: set tw=78 sw=4 ts=4 et:
 *  Copyright (c) 2017, VRT Systems
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "pka.h"
#include "rom-utility.h"

/* PKA SRAM */
static volatile uint8_t * const pka_ram = (volatile uint8_t*)(0x44006000);
#define PKA_RAM_SZ  (0x0800)

/* PKA Operations */
/**
 * PKA Operation: A × B → C
 * Requires: 0 < A_Len, B_Len ≤ Max_Len,
 *   C does not overlap A or B
 * Result length: A_Len + B_Len + 6 (scratch pad)
 */
#define PKA_OP_MUL                  (1ul << 0)
/**
 * PKA Operation: A + C - B → D
 * Requires: 0 < A_Len ≤ Max_Len, A_Len = B_Len, A + C ≥ B,
 *   D does not overlap A or B; OR D is below start of A and B.
 * Result length: A_Len + 1
 */
#define PKA_OP_ADDSUB               (1ul << 1)
#define PKA_OP_MSONE                (1ul << 3)
/**
 * PKA Operation: A + B → C
 * Requires: 0 < A_Len, B_Len ≤ Max_Len,
 *   C does not overlap A or B; OR C is below start of A and B.
 * Result length: Max(A_Len, B_Len) + 1
 */
#define PKA_OP_ADD                  (1ul << 4)
/**
 * PKA Operation: A - B → C
 * Requires: 0 < A_Len, B_Len ≤ Max_Len, A ≥ B,
 *   C does not overlap A or B; OR C is below start of A and B.
 * Result length: Max(A_Len, B_Len)
 */
#define PKA_OP_SUB                  (1ul << 5)
/**
 * PKA Operation: A » Shift → C
 * Requires: 0 < A_Len ≤ Max_Len,
 *   C does not overlap with A OR C is below start of A.
 * Result length: A_Len
 */
#define PKA_OP_RSHIFT               (1ul << 6)
/**
 * PKA Operation: A « Shift → C
 * Requires: 0 < A_Len ≤ Max_Len,
 *   C does not overlap with A OR C is below start of A.
 * Result length:
 * - if Shift MSBs of A are zero:   A_Len
 * - otherwise:                     A_Len + 1
 */
#define PKA_OP_LSHIFT               (1ul << 7)
/**
 * PKA Operation: A % B → C, A ÷ B → D
 * Requires: 1 < B_Len ≤ A_Len ≤ Max_Len, MSB of B ≠ 0,
 *   No overlap allowed between vectors.
 * Result length:
 * - For C: B_Len + 1 (scratch pad)
 * - For D: A_Len + B_Len + 1
 */
#define PKA_OP_DIV                  (1ul << 8)
/**
 * PKA Operation: A % B → C
 * Requires: 1 < B_Len ≤ A_Len ≤ Max_Len, MSB of B ≠ 0
 *   No overlap allowed between vectors.
 * Result length: B_Len + 1
 */
#define PKA_OP_MOD                  (1ul << 9)
/**
 * PKA Operation: compare A and B, result in compare register
 * Requires: 0 < A_Len ≤ Max_Len, A_Len = B_Len
 */
#define PKA_OP_CMP                  (1ul << 10)
/**
 * PKA Operation: A → C
 * Requires: 0 < A_Len ≤ Max_Len,
 *   C does not overlap with A OR C is below start of A.
 * Result length: A_Len
 */
#define PKA_OP_CP                   (1ul << 11)
/**
 * Bit mask for all PKA sequence operations.
 */
#define PKA_OP_SEQ                  (7ul << 12)
#define PKA_OP_SEQ_NOP              (0ul << 12)
/**
 * PKA Operation: C^A % B → D using Chinese Remainders Theorem
 */
#define PKA_OP_SEQ_EXPMOD_CRT       (1ul << 12)
/**
 * PKA Operation: C^A % B → D
 */
#define PKA_OP_SEQ_EXPMOD_ACT4      (2ul << 12)
#define PKA_OP_SEQ_ECCADD           (3ul << 12)
/**
 * PKA Operation: C^A % B → D
 */
#define PKA_OP_SEQ_EXPMOD_ACT2      (4ul << 12)
#define PKA_OP_SEQ_ECCMUL           (5ul << 12)
#define PKA_OP_SEQ_EXPMOD_VAR       (6ul << 12)
#define PKA_OP_SEQ_MODINV           (7ul << 12)
#define PKA_OP_RUN                  (1ul << 15)
#define PKA_OP_STALL                (1ul << 24)

#define PKA_CMP_A_EQ_B              (1ul << 0)
#define PKA_CMP_A_LT_B              (1ul << 1)
#define PKA_CMP_A_GT_B              (1ul << 2)

#define PKA_MSW_IS_ZERO             (1ul << 15)
#define PKA_MSW_ADDR                (0x000000fful)

/**
 * Return the address of a pointer given in the ARM address space as it would
 * be seen by the PKA.
 *
 * @param[in]   ptr     Address of pointer according to ARM CPU
 * @retval      >=0     Address of pointer according to PKA
 * @retval      -EFAULT Address out of range
 */
static int32_t armToPkaAddress(volatile const void* ptr)
{
    if (ptr < &pka_ram)
        return -EFAULT; /* Before PKA RAM block */

    uint32_t offset = ptr - &pka_ram;
    if (offset >= PKA_RAM_SZ)
        return -EFAULT;

    return offset;
}

/**
 * Return the address of a pointer given in the PKA address space as it would
 * be seen by the ARM CPU.
 *
 * @param[in]   ptr     Address of pointer according to PKA
 * @retval      >=0     Address of pointer according to ARM CPU
 * @retval      NULL    Address out of range
 */
static void *pkaToArmAddress(uint32_t ptr)
{
    if (ptr >= PKA_RAM_SZ)
        return NULL;
    return &pka_ram[ptr];
}

/**
 * Copy from host RAM to PKA RAM
 *
 * @param[in]   dst     Address in PKA RAM to copy data to
 * @param[in]   src     Address in host RAM to start copying from
 * @param[in]   sz      Number of bytes to copy from host RAM
 * @retval      0       Success
 * @retval      -EFAULT Bad address (misaligned or out-of-range)
 */
static int cpToPka(uint32_t dst, const void *src, uint16_t sz)
{
    if (dst >= PKA_RAM_SZ)
        return -EFAULT; /* Beyond end */
    if ((dst + sz) > PKA_RAM_SZ)
        return -EFAULT; /* Spills over the end */
    if (dst & 0x4)
        /*
         * All input and output vectors must start at an even 32-bit word
         * address (in other words, must be aligned to an 8 byte boundary
         * in PKA RAM). -- TI CC2538 user's guide SPRU319C 22.1.3.2.
         */
        return -EFAULT;

    /*
     * For simplicity, we will forbid copying from a not-32-bit aligned
     * source address.
     */
    if (((uintptr_t)src) & 0x3)
        return -EFAULT;

    ROM_Memcpy(pkaToArmAddress(dst), src, sz);
    return 0;
}

/**
 * Copy from PKA RAM to host RAM
 *
 * @param[in]   dst     Address in host RAM to copy data to
 * @param[in]   src     Address in PKA RAM to start copying from
 * @param[in]   sz      Number of bytes to copy from PKA RAM
 * @retval      0       Success
 * @retval      -EFAULT Bad address (misaligned or out-of-range)
 */
static int cpToArm(void *dst, uint32_t src, uint16_t sz)
{
    if (src >= PKA_RAM_SZ)
        return -EFAULT; /* Beyond end */
    if ((src + sz) > PKA_RAM_SZ)
        return -EFAULT; /* Spills over the end */
    if (src & 0x4)
        /*
         * All input and output vectors must start at an even 32-bit word
         * address (in other words, must be aligned to an 8 byte boundary
         * in PKA RAM). -- TI CC2538 user's guide SPRU319C 22.1.3.2.
         */
        return -EFAULT;

    /*
     * For simplicity, we will forbid copying from a not-32-bit aligned
     * destination address.
     */
    if (((uintptr_t)dst) & 0x3)
        return -EFAULT;

    ROM_Memcpy(dst, pkaToArmAddress(src), sz);
    return 0;
}
