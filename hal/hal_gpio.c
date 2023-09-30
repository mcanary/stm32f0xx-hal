

/**
 * @file    hal_gpio.c
 * @brief   Source file for the STM32F0xx GPIO peripheral HAL driver
*/


#include "hal_gpio.h"


/**
 * Public functions
*/

ErrorStatus hal_gpio_initializePin(hal_gpio_port_t *port, hal_gpio_pin_t pin, hal_gpio_pinConfigStruct_t *pin_config_struct)
{

    MODIFY_REG(port->MODER, (0x3 << (pin * 2)), (pin_config_struct->mode << (pin * 2)));
    MODIFY_REG(port->PUPDR, (0x3 << (pin * 2)), (pin_config_struct->pull << (pin * 2)));
    MODIFY_REG(port->AFR[pin >> 3], (0xF << ((pin % 8) * 4)), (pin_config_struct->alternate_function << ((pin % 8) * 4)));

    if(pin_config_struct->mode == HAL_GPIO_MODE_OUTPUT)
    {

        MODIFY_REG(port->OTYPER, (0x1 << pin), (pin_config_struct->output_type << pin));
        MODIFY_REG(port->OSPEEDR, (0x3 << (pin * 2)), (pin_config_struct->output_speed << (pin * 2)));

    }

    return SUCCESS;

}

ErrorStatus hal_gpio_setPin(hal_gpio_port_t *port, hal_gpio_pin_t pin)
{

    SET_BIT(port->BSRR, (0x1 << pin));
    return SUCCESS;

}

ErrorStatus hal_gpio_resetPin(hal_gpio_port_t *port, hal_gpio_pin_t pin)
{

    SET_BIT(port->BSRR, (0x1 << (pin + 16)));
    return SUCCESS;

}

ErrorStatus hal_gpio_togglePin(hal_gpio_port_t *port, hal_gpio_pin_t pin)
{

    if((port->ODR & (0x1 << pin)) == 0)
    {

        SET_BIT(port->BSRR, (0x1 << pin));

    }
    else
    {

        SET_BIT(port->BSRR, (0x1 << (pin + 16)));

    }

    return SUCCESS;

}

