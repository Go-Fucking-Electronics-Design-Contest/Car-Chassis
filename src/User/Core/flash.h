#ifndef __FLASH_H__
#define __FLASH_H__

#include <stdint.h>

/*
 * Flash 通用功能接口。
 *
 * 这里不放任何 IMU / gyro / 参数业务逻辑，只提供擦除和写 word 的基础能力。
 * 上层模块需要自己定义保存地址、数据结构、magic、version 和 checksum。
 */

/* 擦除 address 所在的 Flash sector。 */
uint8_t flash_erase_sector(uint32_t address);

/* 从 words 指向的 RAM 缓冲区连续写入 word_count 个 32-bit word 到 Flash。 */
uint8_t flash_program_words(uint32_t address,
                            const uint32_t *words,
                            uint32_t word_count);

#endif
