

/**
 * @file    hal_flash.h
 * @brief   Header file for the STM32F0xx flash peripheral HAL driver
*/


#ifndef __HAL_FLASH
#define __HAL_FLASH

/**
 * Includes
*/
#include "stm32f0xx.h"


/**
 * Defines
*/

// Flash latency
#define HAL_FLASH_LATENCY_0     0x0
#define HAL_FLASH_LATENCY_1     0x1


/**
 * Public type definitions
*/

typedef uint32_t hal_flash_latency_t;


/**
 * Public function prototypes
*/

ErrorStatus hal_flash_setLatency(hal_flash_latency_t latency);
ErrorStatus hal_flash_enablePrefetch(void);


#endif /* __HAL_FLASH */