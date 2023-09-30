

/**
 * @file    hal_gpio.h
 * @brief   Header file for the STM32F0xx GPIO peripheral HAL driver
*/


#ifndef __HAL_GPIO
#define __HAL_GPIO


/**
 * Includes
*/

#include "stm32f0xx.h"


/**
 * Defines
*/

// GPIO peripheral base address references
#define HAL_GPIO_PORT_A ((GPIO_TypeDef *) GPIOA_BASE)
#define HAL_GPIO_PORT_B ((GPIO_TypeDef *) GPIOB_BASE)
#define HAL_GPIO_PORT_C ((GPIO_TypeDef *) GPIOC_BASE)
#define HAL_GPIO_PORT_D ((GPIO_TypeDef *) GPIOD_BASE)
#define HAL_GPIO_PORT_E ((GPIO_TypeDef *) GPIOE_BASE)
#define HAL_GPIO_PORT_F ((GPIO_TypeDef *) GPIOF_BASE)

// GPIO modes
#define HAL_GPIO_MODE_INPUT     0x0
#define HAL_GPIO_MODE_OUTPUT    0x1
#define HAL_GPIO_MODE_ALTERNATE 0x2
#define HAL_GPIO_MODE_ANALOG    0x3

// GPIO output types
#define HAL_GPIO_OUTPUT_TYPE_PUSHPULL  0x0
#define HAL_GPIO_OUTPUT_TYPE_OPENDRAIN 0x1

// GPIO output speeds
#define HAL_GPIO_OUTPUT_SPEED_LOW    0x0
#define HAL_GPIO_OUTPUT_SPEED_MEDIUM 0x1
#define HAL_GPIO_OUTPUT_SPEED_HIGH   0x3

// GPIO pull-up/pull-down
#define HAL_GPIO_PULL_NONE 0x0
#define HAL_GPIO_PULL_UP   0x1
#define HAL_GPIO_PULL_DOWN 0x2

// GPIO alternate functions
#define HAL_GPIO_ALTERNATE_FUNCTION_0  0x0
#define HAL_GPIO_ALTERNATE_FUNCTION_1  0x1
#define HAL_GPIO_ALTERNATE_FUNCTION_2  0x2
#define HAL_GPIO_ALTERNATE_FUNCTION_3  0x3
#define HAL_GPIO_ALTERNATE_FUNCTION_4  0x4
#define HAL_GPIO_ALTERNATE_FUNCTION_5  0x5
#define HAL_GPIO_ALTERNATE_FUNCTION_6  0x6
#define HAL_GPIO_ALTERNATE_FUNCTION_7  0x7

// GPIO pins
#define HAL_GPIO_PIN_0  0x0
#define HAL_GPIO_PIN_1  0x1
#define HAL_GPIO_PIN_2  0x2
#define HAL_GPIO_PIN_3  0x3
#define HAL_GPIO_PIN_4  0x4
#define HAL_GPIO_PIN_5  0x5
#define HAL_GPIO_PIN_6  0x6
#define HAL_GPIO_PIN_7  0x7
#define HAL_GPIO_PIN_8  0x8
#define HAL_GPIO_PIN_9  0x9
#define HAL_GPIO_PIN_10 0xA
#define HAL_GPIO_PIN_11 0xB
#define HAL_GPIO_PIN_12 0xC
#define HAL_GPIO_PIN_13 0xD
#define HAL_GPIO_PIN_14 0xE
#define HAL_GPIO_PIN_15 0xF


/**
 * Public data types
*/

typedef GPIO_TypeDef hal_gpio_port_t;
typedef uint32_t hal_gpio_pin_t;

typedef struct{

    uint32_t mode;
    uint32_t output_type;
    uint32_t output_speed;
    uint32_t pull;
    uint32_t alternate_function;

}hal_gpio_pinConfigStruct_t;


/**
 * Public function prototypes
*/

ErrorStatus hal_gpio_initializePin(hal_gpio_port_t *port, hal_gpio_pin_t pin, hal_gpio_pinConfigStruct_t *pin_config_struct);
ErrorStatus hal_gpio_setPin(hal_gpio_port_t *port, hal_gpio_pin_t pin);
ErrorStatus hal_gpio_resetPin(hal_gpio_port_t *port, hal_gpio_pin_t pin);
ErrorStatus hal_gpio_togglePin(hal_gpio_port_t *port, hal_gpio_pin_t pin);


#endif /* __HAL_GPIO */

