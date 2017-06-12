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

#include "aes.h"

#define AES_CTRL_ALG_SEL_TAG                (1 << 31)   /* Tag is included in data */
#define AES_CTRL_ALG_SEL_HASH               (1 << 2)    /* Hash mode */
#define AES_CTRL_ALG_SEL_AES                (1 << 1)    /* AES encrypt/decrypt mode */
#define AES_CTRL_ALG_SEL_KEYSTORE           (1 << 0)    /* Key store mode */
#define AES_HASH_MODE_IN_SHA256             (1 << 3)    /* Hash mode is SHA256 */
#define AES_HASH_MODE_IN_NEW                (1 << 0)    /* Generate new hash */
#define AES_CTRL_INT_RESULT_AV              (1 << 0)    /* Result available */
#define AES_CTRL_INT_DMA_IN_DONE            (1 << 1)    /* DMA Input complete */
#define AES_CTRL_INT_DMA_BUS_ERR            (1 << 31)   /* DMA Bus Error */
#define AES_CTRL_INT_DMA_KEY_ST_RD_ERR      (1 << 30)   /* DMA Key Store Read Error */
#define AES_CTRL_INT_DMA_KEY_ST_WR_ERR      (1 << 29)   /* DMA Key Store Write Error */
#define AES_HASH_IO_BUF_CTRL_PAD_DMA_MSG    (1 << 7)    /* Pad DMAed message data */
#define AES_DMAC_CHx_CTRL_EN                (1 << 0)    /* Enable DMA channel */
#define AES_DMAC_CHx_CTRL_PRIO              (1 << 1)    /* Enable DMA priority */

/**
 * ROM built-in memcpy function.
 */
static void (*rom_memcpy)(volatile void *dest, const volatile void *src, size_t sz)
    = (void (*)(volatile void*, const volatile void*, size_t))
        (0x00000048 + (7*sizeof(void*)));

/**
 * Determine the result of the hash operation.
 *
 * @retval      0               Operation competed successfully
 * @retval      -EINPROGRESS    Device is still busy performing an operation
 * @retval      -EIO            Memory transfer failed
 */
int32_t cc2538AesHashStatus()
{
    uint32_t status = HWREG(AES_CTRL_INT_STAT);
    if (!(status & AES_CTRL_INT_RESULT_AV))
        return -EINPROGRESS;
    if (status & AES_CTRL_INT_DMA_BUS_ERR)
        return -EIO;
    /* All good */
    return 0;
}

/**
 * Clear the state of the hashing registers.  This acknowledges the related
 * interrupt flags and resets the state ready for use.
 */
void cc2538AesHashFinish()
{
    HWREG(AES_CTRL_INT_CLR) = AES_CTRL_INT_RESULT_AV
                            | AES_CTRL_INT_DMA_IN_DONE
                            | AES_CTRL_INT_DMA_BUS_ERR
                            | AES_CTRL_INT_DMA_KEY_ST_RD_ERR
                            | AES_CTRL_INT_DMA_KEY_ST_WR_ERR;
    HWREG(AES_DMAC_CH0_CTRL) = 0;
    HWREG(AES_DMAC_CH1_CTRL) = 0;
}

/**
 * Begin hashing a given block of data, returning the hash value to the given
 * block of memory.  The SHA-256 hash of the data block will be computed in
 * hardware and returned to the given block of memory.
 *
 * The following assumptions are made:
 * - the block will not be re-located or modified during the operation
 * - the block start address is correctly aligned (32-bit word alignment)
 * - Both input and output hash buffers are 32-bit word aligned.
 *
 * @param[in]   dataIn  Data input to be hashed
 * @param[in]   dataLen Length of data to be hashed
 * @param[in]   hashIn  Initial hash value input, NULL if creating a new hash.
 * @param[out]  hashOut Hash value output.
 * @param[in]   hashLen Hash value length.
 * @param[in]   pad     Whether or not to pad the data
 *
 * @retval      0       Operation begun
 * @retval      -EBUSY  Device is busy with another operation
 */
int32_t cc2538AesHashStart(const void *dataIn, uint32_t dataLen,
        void *hashIn, void *hashOut, uint32_t hashLen, _Bool pad)
{
    /* Check we're not busy or have an unacknowledged state */
    if (HWREG(AES_DMAC_CH0_CTRL) || HWREG(AES_DMAC_CH1_CTRL)
            || HWREG(AES_CTRL_INT_STAT))
        return -EBUSY;

    /* Select SHA256 as our algorithm */
    HWREG(AES_CTRL_ALG_SEL) = AES_CTRL_ALG_SEL_HASH;

    if (hashIn)
    {
        /* Existing hash given, copy it in first */
        rom_memcpy(&(HWREG(AES_HASH_DIGEST_A)), hashIn, hashLen);
        /* Select SHA256 mode with existing hash result */
        HWREG(AES_HASH_MODE_IN) = AES_HASH_MODE_IN_SHA256;
    }
    else
    {
        /* No existing hash, so assume new hash */
        HWREG(AES_HASH_MODE_IN) = AES_HASH_MODE_IN_SHA256 | AES_HASH_MODE_IN_NEW;
    }

    /* Copy across size. */
    HWREG(AES_HASH_LENGTH_IN_L) = dataLen;
    HWREG(AES_HASH_LENGTH_IN_H) = 0;

    /* Enable padding of the message if selected */
    if (pad)
        HWREG(AES_HASH_IO_BUF_CTRL) = AES_HASH_IO_BUF_CTRL_PAD_DMA_MSG;
    else
        HWREG(AES_HASH_IO_BUF_CTRL) = 0;

    /* Set up DMA channel 0 to read the incoming data */
    HWREG(AES_DMAC_CH0_CTRL) = AES_DMAC_CHx_CTRL_EN;
    HWREG(AES_DMAC_CH0_EXTADDR) = (uint32_t)dataIn;
    HWREG(AES_DMAC_CH0_DMALENGTH) = dataLen;

    /* Set up DMA channel 1 to write the outgoing hash */
    HWREG(AES_DMAC_CH1_CTRL) = AES_DMAC_CHx_CTRL_EN;
    HWREG(AES_DMAC_CH1_EXTADDR) = (uint32_t)hashOut;
    HWREG(AES_DMAC_CH1_DMALENGTH) = hashLen;

    return 0;
}
