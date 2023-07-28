/** @file  evk_flash.h
 *  @brief This module enables the usage of the onboard MCU's flash memory with Little FS library.
 *
 *  @copyright Copyright (C) 2020-2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef EVK_FLASH_H_
#define EVK_FLASH_H_

/* INCLUDES *******************************************************************/
#include "evk_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
#define FLASH_BLOCK_SIZE  0x800
#define FLASH_BLOCK_COUNT 64

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Read a region in a block.
 *
 *  @param[in] block_size  Size of an erasable block.
 *  @param[in] block       Block index.
 *  @param[in] off         Offset index on the block.
 *  @param[in] buffer      Buffer to read.
 *  @param[in] size        Size of the buffer.
 *
 *  @return Negative error codes are propagated to the user.
 */
int evk_lfs_read(uint32_t block_size, uint32_t block, uint32_t off, void *buffer, uint32_t size);

/** @brief Program a region in a block.
 *
 *  The block must be previously erased.
 *
 *  @param[in] block_size  Size of an erasable block.
 *  @param[in] block       Block index.
 *  @param[in] off         Offset index on the block.
 *  @param[in] buffer      Buffer to write.
 *  @param[in] size        Size of the buffer.
 *
 *  @return Negative error codes are propagated to the user.
 *          May return LFS_ERR_CORRUPT if the block should be considered bad.
 */
int evk_lfs_prog(uint32_t block_size, uint32_t block, uint32_t off, const void *buffer, uint32_t size);

/** @brief Erase a block.
 *
 *  The state of an erased block is undefined.
 *  A block must be erased before being programmed.
 *
 *  @param[in] block_count  Number of erasable blocks on the device.
 *  @param[in] block  Block index.
 *
 *  @return Negative error codes are propagated to the user. May return
 *          LFS_ERR_CORRUPT if the block should be considered bad.
 */
int evk_lfs_erase(uint32_t block_count, uint32_t block);

/** @brief Sync the state of the underlying block device.
 *
 *  Not used with stm32 internal flash.
 *
 *  @return Negative error codes are propagated to the user.
 */
int evk_lfs_sync(void);

#ifdef __cplusplus
}
#endif

#endif /* EVK_FLASH_H_ */

