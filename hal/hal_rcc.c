

/**
 * @file    hal_rcc.c
 * @brief   Source file for the STM32F0xx RCC peripheral HAL driver
*/


#include "hal_rcc.h"


/**
 * Public functions
*/

ErrorStatus hal_rcc_initializeSystemClock(hal_rcc_systemClockConfigStruct_t *system_clock_config_struct)
{

    // Return an error if specified system clock source is ready
    switch (system_clock_config_struct->system_clock_source)
    {

        case HAL_RCC_SYSTEM_CLOCK_SOURCE_HSI:

            if(!(RCC->CR & RCC_CR_HSIRDY_Msk))
            {

                return ERROR;

            }

            break;

        case HAL_RCC_SYSTEM_CLOCK_SOURCE_HSE:

            if(!(RCC->CR & RCC_CR_HSERDY_Msk))
            {

                return ERROR;

            }

            break;

        case HAL_RCC_SYSTEM_CLOCK_SOURCE_PLL:

            if(!(RCC->CR & RCC_CR_PLLRDY_Msk))
            {

                return ERROR;

            }

            break;

        case HAL_RCC_SYSTEM_CLOCK_SOURCE_HSI48:

            if(!(RCC->CR2 & RCC_CR2_HSI48RDY_Msk))
            {

                return ERROR;

            }

            break;

        default:

            return ERROR;

    }

    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, system_clock_config_struct->system_clock_source);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, system_clock_config_struct->ahb_prescaler);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE_Msk, system_clock_config_struct->apb_prescaler);

    return SUCCESS;

}

ErrorStatus hal_rcc_initializePLL(hal_rcc_PLLConfigStruct_t *pll_config_struct)
{

    // Return an error if specified PLL clock source is ready
    switch (pll_config_struct->pll_source)
    {

        case HAL_RCC_PLL_SOURCE_HSI_DIV2:

            if(!(RCC->CR & RCC_CR_HSIRDY_Msk))
            {

                return ERROR;

            }

            break;

        case HAL_RCC_PLL_SOURCE_HSI_PREDIV:

            if(!(RCC->CR & RCC_CR_HSIRDY_Msk))
            {

                return ERROR;

            }

            break;

        case HAL_RCC_PLL_SOURCE_HSE_PREDIV:

            if(!(RCC->CR & RCC_CR_HSERDY_Msk))
            {

                return ERROR;

            }

            break;

        case HAL_RCC_PLL_SOURCE_HSI48_PREDIV:

            if(!(RCC->CR & RCC_CR2_HSI48ON_Msk))
            {

                return ERROR;

            }

            break;

        default:

            return ERROR;

    }

    RCC->CR &= ~RCC_CR_PLLON_Msk;  // Disable the PLL

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC_Msk, (pll_config_struct->pll_source | pll_config_struct->pll_multiply));
    MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV_Msk, pll_config_struct->pll_pre_divide);

    RCC->CR |= RCC_CR_PLLON_Msk;  // Enable the PLL

    while(!(RCC->CR & RCC_CR_PLLRDY_Msk));  // Wait for the PLL to lock

    return SUCCESS;

}


ErrorStatus hal_rcc_enableHSE(hal_rcc_HSEBypass_t hse_bypass)
{

    CLEAR_BIT(RCC->CR, RCC_CR_HSEON_Msk);
    CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP_Msk);

    if(hse_bypass)
    {

        SET_BIT(RCC->CR, RCC_CR_HSEBYP_Msk);

    }

    SET_BIT(RCC->CR, RCC_CR_HSEON_Msk);

    while(!(RCC->CR & RCC_CR_HSIRDY_Msk));

    return SUCCESS;

}

ErrorStatus hal_rcc_enableHSI48(void)
{

    SET_BIT(RCC->CR2, RCC_CR2_HSI48ON_Msk);

    while(!(RCC->CR2 & RCC_CR2_HSI48RDY_Msk));

    return SUCCESS;

}

ErrorStatus hal_rcc_enablePeripheralClock(hal_rcc_peripheral_t peripheral)
{

    switch(peripheral)
    {

        case HAL_RCC_PERIPHERAL_GPIOA:

            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_GPIOB:

            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_GPIOC:

            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN_Msk);
            break;
        
        #if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F051x8) || defined(STM32F058xx) || \
            defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
            defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_GPIOD:

            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIODEN_Msk);
            break;

        #endif

        #if defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xx) || \
            defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_GPIOE:

            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOEEN_Msk);
            break;
    
        #endif

        case HAL_RCC_PERIPHERAL_GPIOF:

            SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOFEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_DMA:

            SET_BIT(RCC->AHBENR, RCC_AHBENR_DMAEN_Msk);
            break;

        #if defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_DMA2:

            SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA2EN_Msk);
            break;

        #endif

        case HAL_RCC_PERIPHERAL_SRAM:

            SET_BIT(RCC->AHBENR, RCC_AHBENR_SRAMEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_FLITF:
            
            SET_BIT(RCC->AHBENR, RCC_AHBENR_FLITFEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_CRC:

            SET_BIT(RCC->AHBENR, RCC_AHBENR_CRCEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_TSC:

            SET_BIT(RCC->AHBENR, RCC_AHBENR_TSCEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_SYSCFGCOMP:

            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGCOMPEN_Msk);
            break;

        #if defined(STM32F030xC) || defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_USART6:

            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN_Msk);
            break;

        #endif

        #if defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_USART7:

            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART7EN_Msk);
            break;

        #endif

        #if defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_USART8:

            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART8EN_Msk);
            break;

        #endif

        case HAL_RCC_PERIPHERAL_ADC:

            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADCEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_TIM1:

            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_SPI1:

            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_USART1:

            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN_Msk);
            break;

        #if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || defined(STM32F058xx) || \
            defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
            defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_TIM15:

            RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
            break;

        #endif

        case HAL_RCC_PERIPHERAL_TIM16:

            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM16EN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_TIM17:

            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM17EN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_DBGMCU:

            SET_BIT(RCC->APB2ENR, RCC_APB2ENR_DBGMCUEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_TIM2:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_TIM3:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN_Msk);
            break;

        #if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || defined(STM32F058xx) || \
            defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
            defined(STM32F091xC) || defined(StM32F098xx)
        
        case HAL_RCC_PERIPHERAL_TIM6:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN_Msk);
            break;

        #endif

        #if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
            defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_TIM7:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN_Msk);
            break;

        #endif

        case HAL_RCC_PERIPHERAL_TIM14:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_WWDG:
    
            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_SPI2:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_USART2:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN_Msk);
            break;

        #if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
            defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_USART3:
            
            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN_Msk);
            break;

        #endif

        #if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
            defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_USART4:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART4EN_Msk);
            break;

        #endif

        #if defined(STM32F030xC) || defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_USART5:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART5EN_Msk);
            break;

        #endif

        case HAL_RCC_PERIPHERAL_I2C1:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN_Msk);
            break;

        #if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F051x8) || defined(STM32F058xx) || \
            defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
            defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_I2C2:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN_Msk);
            break;

        #endif

        case HAL_RCC_PERIPHERAL_USB:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_CAN:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_CANEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_CRS:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_CRSEN_Msk);
            break;

        case HAL_RCC_PERIPHERAL_PWR:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN_Msk);
            break;

        #if defined(STM32F051x8) || defined(STM32F058xx) || defined(STM32F071xB) || defined(STM32F072xB) || \
            defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)

        case HAL_RCC_PERIPHERAL_DAC:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_DACEN_Msk);
            break;

        #endif

        case HAL_RCC_PERIPHERAL_CEC:

            SET_BIT(RCC->APB1ENR, RCC_APB1ENR_CECEN_Msk);
            break;

        default:
            
            return ERROR;

    }

    return SUCCESS;

}

ErrorStatus hal_rcc_disableHSI(void)
{

    if((RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_HSI)
    {

        return ERROR;

    }

    CLEAR_BIT(RCC->CR, RCC_CR_HSION_Msk);

    return SUCCESS;

}

