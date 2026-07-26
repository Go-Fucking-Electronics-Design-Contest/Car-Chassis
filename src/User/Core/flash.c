#include "flash.h"
#include "ti_msp_dl_config.h"
#include <ti/driverlib/dl_flashctl.h>

/*
 * 擦除 Flash sector。
 *
 * MSPM0 从 Flash 执行代码时，擦写期间不希望中断进来从 Flash 取指，
 * 所以这里短时间关中断。调用方不要在高频实时路径里频繁擦写 Flash。
 */
uint8_t flash_erase_sector(uint32_t address)
{
    DL_FLASHCTL_COMMAND_STATUS status;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(
        FLASHCTL, address, DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_eraseMemoryFromRAM(
        FLASHCTL, address, DL_FLASHCTL_COMMAND_SIZE_SECTOR);

    if (primask == 0u)
    {
        __enable_irq();
    }

    return (status == DL_FLASHCTL_COMMAND_STATUS_PASSED);
}

uint8_t flash_program_words(uint32_t address,
                            const uint32_t *words,
                            uint32_t word_count)
{
    DL_FLASHCTL_COMMAND_STATUS status;
    uint32_t primask;

    if ((words == 0) || (word_count == 0u))
    {
        return 0;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(
        FLASHCTL, address, DL_FLASHCTL_REGION_SELECT_MAIN);

    /*
     * 带 ECC 的器件使用自动生成 ECC 的写入函数；
     * 不带 ECC 的器件使用普通 word 写入函数。
     */
#ifdef __MSPM0_HAS_ECC__
    status = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
        FLASHCTL,
        address,
        (uint32_t *) words,
        word_count,
        DL_FLASHCTL_REGION_SELECT_MAIN);
#else
    status = DL_FlashCTL_programMemoryFromRAM(
        FLASHCTL,
        address,
        (uint32_t *) words,
        word_count,
        DL_FLASHCTL_REGION_SELECT_MAIN);
#endif

    if (primask == 0u)
    {
        __enable_irq();
    }

    return (status == DL_FLASHCTL_COMMAND_STATUS_PASSED);
}
