/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-17     armink       the first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtconfig.h>
#include <board.h>

#define RT_APP_PART_ADDR 0x08020000  //app begin address

#define NOR_FLASH_DEV_NAME             "norflash0"

#define FLASH_SIZE_GRANULARITY_16K   (4 * 16 * 1024)
#define FLASH_SIZE_GRANULARITY_64K   (1 * 64 * 1024)
#define FLASH_SIZE_GRANULARITY_128K  (1 * 128 * 1024)

#define STM32_FLASH_START_ADRESS_16K  STM32_FLASH_START_ADRESS
#define STM32_FLASH_START_ADRESS_64K  (STM32_FLASH_START_ADRESS_16K + FLASH_SIZE_GRANULARITY_16K)
#define STM32_FLASH_START_ADRESS_128K (STM32_FLASH_START_ADRESS_64K + FLASH_SIZE_GRANULARITY_64K)

/* ===================== Flash device Configuration ========================= */
extern const struct fal_flash_dev stm32_onchip_flash;
extern struct fal_flash_dev nor_flash0;

/*extern const struct fal_flash_dev stm32_onchip_flash;
extern const struct fal_flash_dev stm32_onchip_flash_64k;
extern const struct fal_flash_dev stm32_onchip_flash_128k;*/

/* flash device table */
#define FAL_FLASH_DEV_TABLE                                          \
{                                                                    \
    &stm32_onchip_flash,                                             \
    &nor_flash0,                                                     \
}
/*
 *
 *   */
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
/* partition table */
#define FAL_PART_TABLE                                                                             \
{                                                                                                  \
    {FAL_PART_MAGIC_WROD, "bootloader", "onchip_flash",            0,     (128*1024) ,   0}, \
    {FAL_PART_MAGIC_WROD, "app",        "onchip_flash",   (128*1024),     (360*1024) ,   0}, \
    {FAL_PART_MAGIC_WROD, "param",      "onchip_flash",   (488*1024),      (24*1024) ,   0}, \
    {FAL_PART_MAGIC_WROD, "download",  NOR_FLASH_DEV_NAME,        0 ,     (360*1024) ,   0}, \
    {FAL_PART_MAGIC_WROD, "factory",   NOR_FLASH_DEV_NAME,(360*1024),     (360*1024) ,   0}, \
}
/*
 *
 * */
#endif /* FAL_PART_HAS_TABLE_CFG */

#endif /* _FAL_CFG_H_ */
