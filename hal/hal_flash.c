

/**
 * @file    hal_flash.c
 * @brief   Source file for the STM32F0xx flash peripheral HAL driver
*/


#include "hal_flash.h"


/**
 * Public functions
*/

ErrorStatus hal_flash_setLatency(hal_flash_latency_t latency)
{

    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, latency);
    return SUCCESS;

}

ErrorStatus hal_flash_enablePrefetch(void)
{

    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE_Msk);
    return SUCCESS;

}

