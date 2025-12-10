#include "imu_driver.h"
#include "hal_spi.h"
#include "hal_gpio.h" // For Chip Select (CS) pin control
#include "system_config.h" 
#include <cmath> // For mathematical operations

// --- IMU Register Addresses (Conceptual for MPU-9250 or similar) ---
#define IMU_ADDR_WHO_AM_I       0x75
#define IMU_ADDR_PWR_MGMT_1     0x6B
#define IMU_ADDR_GYRO_CONFIG    0x1B
#define IMU_ADDR_ACCEL_CONFIG   0x1C
#define IMU_ADDR_ACCEL_XOUT_H   0x3B // Start of data registers

// --- Sensitivity Scales (Based on configuration) ---
// Example: If GYRO_FS_SEL=1 (500 deg/s), Sensitivity = 65.5 LSB/deg/s
#define GYRO_SCALE_FACTOR       (500.0f / 32768.0f) // rad/s per LSB (Converted from deg/s)
#define ACCEL_SCALE_FACTOR      (4.0f / 32768.0f)   // g per LSB (Assuming +/-4g range)


/**
 * @brief Sends a single byte command to a specific IMU register.
 * @param reg_addr Register address to write to.
 * @param data The byte data to write.
 */
void IMUDriver::write_register(uint8_t reg_addr, uint8_t data)
{
    // The CS pin must be driven LOW for the duration of the transfer.
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, PIN_STATE_LOW); 
    
    // Transfer 1: Register Address (MSB indicates Write)
    HAL_SPI_TransmitReceive(IMU_SPI_PORT, reg_addr, nullptr, 1);
    
    // Transfer 2: Data
    HAL_SPI_TransmitReceive(IMU_SPI_PORT, data, nullptr, 1);

    // Drive CS pin HIGH to end the transfer.
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, PIN_STATE_HIGH);
}

/**
 * @brief Reads a single byte from a specific IMU register.
 * @param reg_addr Register address to read from.
 * @return uint8_t The received data byte.
 */
uint8_t IMUDriver::read_register(uint8_t reg_addr)
{
    uint8_t rx_data;
    
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, PIN_STATE_LOW); 
    
    // Transfer 1: Register Address (MSB must indicate READ, e.g., reg_addr | 0x80)
    HAL_SPI_TransmitReceive(IMU_SPI_PORT, reg_addr | 0x80, nullptr, 1);
    
    // Transfer 2: Dummy data sent to clock out the read data
    HAL_SPI_TransmitReceive(IMU_SPI_PORT, 0x00, &rx_data, 1); 

    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, PIN_STATE_HIGH);
    
    return rx_data;
}


/**
 * @brief Initializes the IMU: checks connectivity, resets, and configures ranges.
 * @return bool True if initialization successful, false otherwise.
 */
bool IMUDriver::initialize()
{
    // 1. Verify Connectivity (WHO_AM_I Register Check)
    if (read_register(IMU_ADDR_WHO_AM_I) != 0x71) { // 0x71 is a common WHO_AM_I value
        return false; // Critical failure: IMU not responding or wrong part
    }
    
    // 2. Power Management Reset
    write_register(IMU_ADDR_PWR_MGMT_1, 0x80); // Reset command
    // Wait for reset to complete...

    // 3. Configure Power Management: Set clock source (e.g., PLL with Z-Gyro)
    write_register(IMU_ADDR_PWR_MGMT_1, 0x01); 

    // 4. Configure Gyroscope: Set full-scale range (e.g., +/- 500 deg/s)
    write_register(IMU_ADDR_GYRO_CONFIG, 0x08); // Value corresponds to range setting

    // 5. Configure Accelerometer: Set full-scale range (e.g., +/- 4g)
    write_register(IMU_ADDR_ACCEL_CONFIG, 0x08);

    // 6. Configure Sample Rate, Digital Low-Pass Filters (DLPF) 
    // ... (omitted for brevity)

    return true;
}

/**
 * @brief Reads all raw 16-bit data from the IMU (Accel X,Y,Z, Gyro X,Y,Z, etc.)
 * * This is a burst read starting from the first data register (ACCEL_XOUT_H).
 * @return IMUData_t The structured, converted data in engineering units.
 */
IMUData_t IMUDriver::read_data()
{
    IMUData_t data;
    uint8_t raw_buffer[14]; // Buffer for 7 axis of 16-bit data (14 bytes)

    // --- 1. Burst Read ---
    // Initiate multi-byte read transfer
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, PIN_STATE_LOW); 

    // Send the first register address with the read bit set
    HAL_SPI_TransmitReceive(IMU_SPI_PORT, IMU_ADDR_ACCEL_XOUT_H | 0x80, nullptr, 1);
    
    // Read the 14 data bytes (sending 14 dummy bytes to clock them in)
    for (int i = 0; i < 14; ++i) {
        HAL_SPI_TransmitReceive(IMU_SPI_PORT, 0x00, &raw_buffer[i], 1);
    }

    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, PIN_STATE_HIGH);

    // --- 2. Data Conversion (Raw LSB to Engineering Units) ---
    
    // Combine two 8-bit registers (High byte and Low byte) into a 16-bit signed integer
    int16_t raw_accel_x = (raw_buffer[0] << 8) | raw_buffer[1];
    int16_t raw_accel_y = (raw_buffer[2] << 8) | raw_buffer[3];
    int16_t raw_accel_z = (raw_buffer[4] << 8) | raw_buffer[5];
    
    // Gyro data is typically further down the buffer
    int16_t raw_gyro_x = (raw_buffer[8] << 8) | raw_buffer[9];
    // ... and so on for Gyro Y and Z ...

    // --- 3. Apply Scale Factors and Calibration ---
    
    // Accelerometer: Convert LSB to m/s^2
    data.accel_x_mps2 = raw_accel_x * ACCEL_SCALE_FACTOR * GRAVITY_M_PER_S2; 
    data.accel_y_mps2 = raw_accel_y * ACCEL_SCALE_FACTOR * GRAVITY_M_PER_S2;
    data.accel_z_mps2 = raw_accel_z * ACCEL_SCALE_FACTOR * GRAVITY_M_PER_S2;
    
    // Gyroscope: Convert LSB to rad/s (Required for Kalman Filter quaternion math)
    data.gyro_x_rad_s = raw_gyro_x * GYRO_SCALE_FACTOR * (M_PI / 180.0f); 
    // ... and so on for Gyro Y and Z ...
    
    // NOTE: Real-world drivers apply bias/offset removal here (Gyro Calibration).

    return data;
}
