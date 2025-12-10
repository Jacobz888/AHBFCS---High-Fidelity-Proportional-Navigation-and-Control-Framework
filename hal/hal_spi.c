#include "hal_spi.h"
#include "system_config.h" 
#include "hal_gpio.h" // SPI pins need GPIO initialization

// Define conceptual memory addresses or identifiers for the SPI peripheral
// This structure abstracts away the need for direct register manipulation (e.g., SPI1->CR1)

// --- Conceptual SPI Peripheral Structure ---
typedef struct {
    uint32_t CR1;   // Control Register 1
    uint32_t CR2;   // Control Register 2
    uint32_t SR;    // Status Register
    uint32_t DR;    // Data Register (Read/Write)
    // ... other registers defined similarly (e.g., CRC, I2SCFGR)
} SPI_Port_t;

// Conceptual pointer mapping for the sake of the abstraction
#define SPI1 ((SPI_Port_t*)0x40013000)
// ... other SPI ports defined similarly ...


/**
 * @brief Initializes a specific SPI peripheral for Master mode communication.
 * * Configures critical parameters like clock polarity/phase (CPOL/CPHA) and baud rate.
 * @param spi The conceptual SPI port (e.g., SPI1).
 * @param baud_prescaler The clock divisor to set the desired communication speed.
 * @param cpol Clock polarity (High or Low).
 * @param cpha Clock phase (1-edge or 2-edge).
 */
void HAL_SPI_Init(SPI_Port_t* spi, uint8_t baud_prescaler, uint8_t cpol, uint8_t cpha)
{
    // --- 1. Clock Enable and GPIO Setup ---
    // (In a real system, the clock for 'spi' must be enabled, and GPIO pins
    // for SCK, MISO, and MOSI must be configured for Alternate Function mode.)
    // HAL_GPIO_Init(GPIOA, PIN5, PIN_MODE_ALT_FUNC, ...); // Example for SCK

    // --- 2. Configure Control Register 1 (CR1) ---

    // a. Set Master Mode (MSB bit)
    spi->CR1 |= (1 << 2);

    // b. Set Baud Rate Prescaler (BR bits)
    // Clear and then set the prescaler value based on the baud_prescaler parameter
    spi->CR1 &= ~(0b111 << 3);
    spi->CR1 |= (baud_prescaler << 3); // 0b000=div2, 0b111=div256

    // c. Set CPOL and CPHA (Clock Polarity and Phase)
    if (cpol) spi->CR1 |= (1 << 1); else spi->CR1 &= ~(1 << 1);
    if (cpha) spi->CR1 |= (1 << 0); else spi->CR1 &= ~(1 << 0);

    // d. Enable the SPI peripheral (SPE bit)
    spi->CR1 |= (1 << 6);
}

/**
 * @brief Transmits a single byte and simultaneously receives a single byte (full-duplex).
 * * This function is non-blocking and assumes the Chip Select (CS) pin is already handled by GPIO.
 * @param spi The conceptual SPI port.
 * @param tx_data The byte to transmit.
 * @return uint8_t The received byte of data.
 */
uint8_t HAL_SPI_TransmitReceive(SPI_Port_t* spi, uint8_t tx_data)
{
    // --- 1. Wait for TX Buffer Empty ---
    // Check the Transmit Buffer Empty flag (TXE) in the Status Register (SR)
    while (!(spi->SR & (1 << 1))) { 
        // Spin lock until ready to transmit
    } 

    // --- 2. Write Data to TX Buffer ---
    spi->DR = tx_data;

    // --- 3. Wait for RX Buffer Not Empty ---
    // Check the Receive Buffer Not Empty flag (RXNE) in the Status Register (SR)
    while (!(spi->SR & (1 << 0))) {
        // Spin lock until data is received
    }

    // --- 4. Read Data from RX Buffer ---
    return (uint8_t)spi->DR;
}


/**
 * @brief Performs a sequence of transmit/receive operations (e.g., reading a sensor register).
 * * The Chip Select (CS) pin must be handled externally (via HAL_GPIO). 
 * @param spi The conceptual SPI port.
 * @param tx_buffer Pointer to the array of bytes to send.
 * @param rx_buffer Pointer to the array where received bytes will be stored.
 * @param len The number of bytes to transfer.
 */
void HAL_SPI_Transfer(SPI_Port_t* spi, const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        // Use the single byte transmit/receive function for simplicity
        // Note: tx_buffer might contain dummy data (e.g., 0xFF) if only reading is desired.
        rx_buffer[i] = HAL_SPI_TransmitReceive(spi, tx_buffer[i]);
    }
}
