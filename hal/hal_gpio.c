#include "hal_gpio.h"
#include "system_config.h" 
// #include "stm32f4xx.h" // NOTE: In a real project, this would include the specific MCU header

// Define conceptual memory addresses or identifiers for ports and pins
// This structure abstracts away the need for direct register manipulation (e.g., GPIOD->MODER)

// --- Conceptual GPIO Port Structure ---
typedef struct {
    uint32_t MODER; // Mode Register (Input, Output, Alt. Function, Analog)
    uint32_t OTYPER; // Output Type Register (Push-Pull, Open-Drain)
    uint32_t OSPEEDR; // Output Speed Register (Low, Medium, High, Very High)
    uint32_t PUPDR; // Pull-Up/Pull-Down Register
    uint32_t IDR; // Input Data Register
    uint32_t ODR; // Output Data Register
} GPIO_Port_t;

// Conceptual pointer mapping for the sake of the abstraction
#define GPIOA ((GPIO_Port_t*)0x40020000)
// ... other ports defined similarly ...


/**
 * @brief Initializes a specific GPIO pin with defined mode, output type, and pull configuration.
 * * This function configures the hardware registers. 
 * * @param port The conceptual GPIO port (e.g., GPIOA, GPIOB).
 * @param pin The specific pin number (0-15).
 * @param mode The pin mode (Input, Output, Alternate Function, Analog).
 * @param type The output type (PushPull or OpenDrain).
 * @param pull The pull configuration (NoPull, PullUp, or PullDown).
 */
void HAL_GPIO_Init(GPIO_Port_t* port, uint8_t pin, PinMode_t mode, PinOutputType_t type, PinPull_t pull)
{
    // --- 1. Clock Enable ---
    // (In a real system, the clock to the peripheral bus for 'port' must be enabled first)

    // --- 2. Configure Pin Mode (MODER Register) ---
    // Clear the two bits corresponding to the pin's mode
    port->MODER &= ~(0x3 << (pin * 2));
    // Set the new mode
    port->MODER |= (mode << (pin * 2));

    // --- 3. Configure Output Type (OTYPER Register) ---
    if (type == PIN_OUTPUT_OPENDRAIN) {
        port->OTYPER |= (1 << pin);
    } else { // PIN_OUTPUT_PUSHPULL
        port->OTYPER &= ~(1 << pin);
    }
    
    // --- 4. Configure Pull-Up/Pull-Down (PUPDR Register) ---
    // Clear the two bits corresponding to the pull configuration
    port->PUPDR &= ~(0x3 << (pin * 2));
    // Set the new pull configuration
    port->PUPDR |= (pull << (pin * 2));

    // NOTE: Output speed (OSPEEDR) would typically be set here as well.
}


/**
 * @brief Sets the state of an output pin to high (1) or low (0).
 * * @param port The conceptual GPIO port.
 * @param pin The specific pin number.
 * @param state The desired state (PIN_STATE_HIGH or PIN_STATE_LOW).
 */
void HAL_GPIO_WritePin(GPIO_Port_t* port, uint8_t pin, PinState_t state)
{
    if (state == PIN_STATE_HIGH) {
        // Set the bit in the Output Data Register (ODR)
        port->ODR |= (1 << pin); 
    } else {
        // Clear the bit in the Output Data Register (ODR)
        port->ODR &= ~(1 << pin); 
    }
    // NOTE: Direct bit-set/reset registers (BSRR) are often used for faster, atomic writes.
}


/**
 * @brief Reads the state of an input pin.
 * * @param port The conceptual GPIO port.
 * @param pin The specific pin number.
 * @return PinState_t The current state (PIN_STATE_HIGH or PIN_STATE_LOW).
 */
PinState_t HAL_GPIO_ReadPin(GPIO_Port_t* port, uint8_t pin)
{
    // Read the Input Data Register (IDR)
    if (port->IDR & (1 << pin)) {
        return PIN_STATE_HIGH;
    } else {
        return PIN_STATE_LOW;
    }
}
