

/**
 * @file    hal_rcc.h
 * @brief   Header file for the STM32F0xx RCC peripheral HAL driver
*/


#ifndef __HAL_RCC
#define __HAL_RCC


/**
 * Includes
*/

#include "stm32f0xx.h"


/**
 * Defines
*/

// System clock source
#define HAL_RCC_SYSTEM_CLOCK_SOURCE_HSI     RCC_CFGR_SW_HSI
#define HAL_RCC_SYSTEM_CLOCK_SOURCE_HSE     RCC_CFGR_SW_HSE
#define HAL_RCC_SYSTEM_CLOCK_SOURCE_PLL     RCC_CFGR_SW_PLL
#define HAL_RCC_SYSTEM_CLOCK_SOURCE_HSI48   RCC_CFGR_SW_HSI48

// AHB prescaler
#define HAL_RCC_AHB_PRESCALER_DIV1      RCC_CFGR_HPRE_DIV1
#define HAL_RCC_AHB_PRESCALER_DIV2      RCC_CFGR_HPRE_DIV2
#define HAL_RCC_AHB_PRESCALER_DIV4      RCC_CFGR_HPRE_DIV4
#define HAL_RCC_AHB_PRESCALER_DIV8      RCC_CFGR_HPRE_DIV8
#define HAL_RCC_AHB_PRESCALER_DIV16     RCC_CFGR_HPRE_DIV16
#define HAL_RCC_AHB_PRESCALER_DIV64     RCC_CFGR_HPRE_DIV64
#define HAL_RCC_AHB_PRESCALER_DIV128    RCC_CFGR_HPRE_DIV128
#define HAL_RCC_AHB_PRESCALER_DIV256    RCC_CFGR_HPRE_DIV256
#define HAL_RCC_AHB_PRESCALER_DIV512    RCC_CFGR_HPRE_DIV512

// APB prescaler
#define HAL_RCC_APB_PRESCALER_DIV1      RCC_CFGR_PPRE_DIV1
#define HAL_RCC_APB_PRESCALER_DIV2      RCC_CFGR_PPRE_DIV2
#define HAL_RCC_APB_PRESCALER_DIV4      RCC_CFGR_PPRE_DIV4
#define HAL_RCC_APB_PRESCALER_DIV8      RCC_CFGR_PPRE_DIV8
#define HAL_RCC_APB_PRESCALER_DIV16     RCC_CFGR_PPRE_DIV16

// PLL source
#define HAL_RCC_PLL_SOURCE_HSI_DIV2     RCC_CFGR_PLLSRC_HSI_DIV2
#define HAL_RCC_PLL_SOURCE_HSI_PREDIV   RCC_CFGR_PLLSRC_HSI_PREDIV
#define HAL_RCC_PLL_SOURCE_HSE_PREDIV   RCC_CFGR_PLLSRC_HSE_PREDIV
#define HAL_RCC_PLL_SOURCE_HSI48_PREDIV RCC_CFGR_PLLSRC_HSI48_PREDIV

// PLL pre-divider
#define HAL_RCC_PLL_PREDIV_DIV1     RCC_CFGR2_PREDIV_DIV1
#define HAL_RCC_PLL_PREDIV_DIV2     RCC_CFGR2_PREDIV_DIV2
#define HAL_RCC_PLL_PREDIV_DIV3     RCC_CFGR2_PREDIV_DIV3
#define HAL_RCC_PLL_PREDIV_DIV4     RCC_CFGR2_PREDIV_DIV4
#define HAL_RCC_PLL_PREDIV_DIV5     RCC_CFGR2_PREDIV_DIV5
#define HAL_RCC_PLL_PREDIV_DIV6     RCC_CFGR2_PREDIV_DIV6
#define HAL_RCC_PLL_PREDIV_DIV7     RCC_CFGR2_PREDIV_DIV7
#define HAL_RCC_PLL_PREDIV_DIV8     RCC_CFGR2_PREDIV_DIV8
#define HAL_RCC_PLL_PREDIV_DIV9     RCC_CFGR2_PREDIV_DIV9
#define HAL_RCC_PLL_PREDIV_DIV10    RCC_CFGR2_PREDIV_DIV10
#define HAL_RCC_PLL_PREDIV_DIV11    RCC_CFGR2_PREDIV_DIV11
#define HAL_RCC_PLL_PREDIV_DIV12    RCC_CFGR2_PREDIV_DIV12
#define HAL_RCC_PLL_PREDIV_DIV13    RCC_CFGR2_PREDIV_DIV13
#define HAL_RCC_PLL_PREDIV_DIV14    RCC_CFGR2_PREDIV_DIV14
#define HAL_RCC_PLL_PREDIV_DIV15    RCC_CFGR2_PREDIV_DIV15
#define HAL_RCC_PLL_PREDIV_DIV16    RCC_CFGR2_PREDIV_DIV16

// PLL multiplier
#define HAL_RCC_PLL_MUL_2   RCC_CFGR_PLLMUL2
#define HAL_RCC_PLL_MUL_3   RCC_CFGR_PLLMUL3
#define HAL_RCC_PLL_MUL_4   RCC_CFGR_PLLMUL4
#define HAL_RCC_PLL_MUL_5   RCC_CFGR_PLLMUL5
#define HAL_RCC_PLL_MUL_6   RCC_CFGR_PLLMUL6
#define HAL_RCC_PLL_MUL_7   RCC_CFGR_PLLMUL7
#define HAL_RCC_PLL_MUL_8   RCC_CFGR_PLLMUL8
#define HAL_RCC_PLL_MUL_9   RCC_CFGR_PLLMUL9
#define HAL_RCC_PLL_MUL_10  RCC_CFGR_PLLMUL10
#define HAL_RCC_PLL_MUL_11  RCC_CFGR_PLLMUL11
#define HAL_RCC_PLL_MUL_12  RCC_CFGR_PLLMUL12
#define HAL_RCC_PLL_MUL_13  RCC_CFGR_PLLMUL13
#define HAL_RCC_PLL_MUL_14  RCC_CFGR_PLLMUL14
#define HAL_RCC_PLL_MUL_15  RCC_CFGR_PLLMUL15
#define HAL_RCC_PLL_MUL_16  RCC_CFGR_PLLMUL16

// Peripherals
#define HAL_RCC_PERIPHERAL_GPIOA        0
#define HAL_RCC_PERIPHERAL_GPIOB        1
#define HAL_RCC_PERIPHERAL_GPIOC        2
#define HAL_RCC_PERIPHERAL_GPIOD        3
#define HAL_RCC_PERIPHERAL_GPIOE        4
#define HAL_RCC_PERIPHERAL_GPIOF        5
#define HAL_RCC_PERIPHERAL_DMA          6
#define HAL_RCC_PERIPHERAL_DMA2         7
#define HAL_RCC_PERIPHERAL_SRAM         8
#define HAL_RCC_PERIPHERAL_FLITF        9
#define HAL_RCC_PERIPHERAL_CRC          10
#define HAL_RCC_PERIPHERAL_TSC          11
#define HAL_RCC_PERIPHERAL_SYSCFGCOMP   12
#define HAL_RCC_PERIPHERAL_USART6       13
#define HAL_RCC_PERIPHERAL_USART7       14
#define HAL_RCC_PERIPHERAL_USART8       15
#define HAL_RCC_PERIPHERAL_ADC          16
#define HAL_RCC_PERIPHERAL_TIM1         17
#define HAL_RCC_PERIPHERAL_SPI1         18
#define HAL_RCC_PERIPHERAL_USART1       19
#define HAL_RCC_PERIPHERAL_TIM15        20
#define HAL_RCC_PERIPHERAL_TIM16        21
#define HAL_RCC_PERIPHERAL_TIM17        22
#define HAL_RCC_PERIPHERAL_DBGMCU       23
#define HAL_RCC_PERIPHERAL_TIM2         24
#define HAL_RCC_PERIPHERAL_TIM3         25
#define HAL_RCC_PERIPHERAL_TIM6         26
#define HAL_RCC_PERIPHERAL_TIM7         27
#define HAL_RCC_PERIPHERAL_TIM14        28
#define HAL_RCC_PERIPHERAL_WWDG         29
#define HAL_RCC_PERIPHERAL_SPI2         30
#define HAL_RCC_PERIPHERAL_USART2       31
#define HAL_RCC_PERIPHERAL_USART3       32
#define HAL_RCC_PERIPHERAL_USART4       33
#define HAL_RCC_PERIPHERAL_USART5       34
#define HAL_RCC_PERIPHERAL_I2C1         35
#define HAL_RCC_PERIPHERAL_I2C2         36
#define HAL_RCC_PERIPHERAL_USB          37
#define HAL_RCC_PERIPHERAL_CAN          38
#define HAL_RCC_PERIPHERAL_CRS          39
#define HAL_RCC_PERIPHERAL_PWR          40
#define HAL_RCC_PERIPHERAL_DAC          41
#define HAL_RCC_PERIPHERAL_CEC          42


/**
 * Public data types
*/

typedef uint8_t hal_rcc_HSEBypass_t;
typedef uint8_t hal_rcc_peripheral_t;

typedef struct{

    uint32_t system_clock_source;
    uint32_t ahb_prescaler;
    uint32_t apb_prescaler;

}hal_rcc_systemClockConfigStruct_t;

typedef struct{

    uint32_t pll_enable;
    uint32_t pll_source;
    uint32_t pll_pre_divide;
    uint32_t pll_multiply;

}hal_rcc_PLLConfigStruct_t;


/**
 * Public function prototypes
*/

ErrorStatus hal_rcc_initializeSystemClock(hal_rcc_systemClockConfigStruct_t *system_clock_config_struct);
ErrorStatus hal_rcc_initializePLL(hal_rcc_PLLConfigStruct_t *pll_config_struct);
ErrorStatus hal_rcc_enableHSE(hal_rcc_HSEBypass_t hse_bypass);
ErrorStatus hal_rcc_enableHSI48(void);
ErrorStatus hal_rcc_enablePeripheralClock(hal_rcc_peripheral_t peripheral);
ErrorStatus hal_rcc_disableHSI(void);


#endif /* __HAL_RCC */
