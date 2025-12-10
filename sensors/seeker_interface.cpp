#include "seeker_interface.h"
#include "system_config.h" 
#include "hal_adc.h" // Assumes a HAL for Analog-to-Digital Conversion
#include "utilities/math_ops.h"

// Define the conceptual ADC conversion parameters
#define ADC_MAX_VALUE 4095.0f // Assuming a 12-bit ADC (2^12 - 1)
#define ADC_VOLTAGE_REF 3.3f // Reference voltage of the ADC

// Define the angular conversion factor (highly complex in a real system)
// This is a conceptual scaling factor from intensity error to an angle (radians).
// The actual value depends on the optics focal length and detector geometry.
#define INTENSITY_TO_ANGLE_SCALE 0.0005f // radians per unit of differential intensity

/**
 * @brief Reads the raw IR intensity data from the four quadrant sensors.
 * * This assumes a simultaneous reading for all four channels to minimize time lag.
 * @return SeekerRawData_t A structure holding the raw ADC counts for Q1-Q4.
 */
SeekerRawData_t SeekerInterface::read_raw_data()
{
    SeekerRawData_t raw_data;
    
    // Assumes a HAL function for reading specific ADC channels
    raw_data.Q1_raw = HAL_ADC_ReadChannel(SEEKER_Q1_ADC_CHANNEL);
    raw_data.Q2_raw = HAL_ADC_ReadChannel(SEEKER_Q2_ADC_CHANNEL);
    raw_data.Q3_raw = HAL_ADC_ReadChannel(SEEKER_Q3_ADC_CHANNEL);
    raw_data.Q4_raw = HAL_ADC_ReadChannel(SEEKER_Q4_ADC_CHANNEL);
    
    return raw_data;
}


/**
 * @brief Converts raw ADC intensity readings into the Line-of-Sight (LOS) angular error.
 * * This uses the core Quadrant Seeker Differential Calculation.
 * @param raw_data The raw ADC counts from the four quadrants.
 * @return SeekerData_t The calculated LOS pitch and yaw angles in radians.
 */
SeekerData_t SeekerInterface::process_data(const SeekerRawData_t& raw_data)
{
    SeekerData_t processed_data;

    // --- 1. Normalize and Sum Intensity ---
    // Convert raw ADC counts to a normalized intensity (0.0 to 1.0)
    float Q1 = raw_data.Q1_raw / ADC_MAX_VALUE;
    float Q2 = raw_data.Q2_raw / ADC_MAX_VALUE;
    float Q3 = raw_data.Q3_raw / ADC_MAX_VALUE;
    float Q4 = raw_data.Q4_raw / ADC_MAX_VALUE;

    // Total intensity is used for normalization and signal presence check
    float Intensity_Total = Q1 + Q2 + Q3 + Q4;
    
    // Safety check: If total intensity is too low, the seeker has lost lock.
    if (Intensity_Total < 0.1f) {
        // Set a flag or return a known 'lock lost' value
        return {0.0f, 0.0f, false}; 
    }

    // --- 2. Calculate Differential Tracking (The core logic) ---
    // The differential calculation yields the normalized angular error.
    
    // Pitch Error: (Top Halves) - (Bottom Halves) / Total Intensity
    float Pitch_Error_Ratio = ( (Q1 + Q2) - (Q3 + Q4) ) / Intensity_Total;

    // Yaw Error: (Right Halves) - (Left Halves) / Total Intensity
    float Yaw_Error_Ratio   = ( (Q2 + Q3) - (Q1 + Q4) ) / Intensity_Total;

    // --- 3. Convert Error Ratio to Angle (LOS Angle) ---
    // The ratio is linearly scaled into an angular error (radians).
    processed_data.los_pitch_angle_rad = Pitch_Error_Ratio * INTENSITY_TO_ANGLE_SCALE;
    processed_data.los_yaw_angle_rad   = Yaw_Error_Ratio * INTENSITY_TO_ANGLE_SCALE;
    processed_data.is_locked = true;

    // NOTE: A real system often requires a specialized filter (e.g., a Band-Pass 
    // filter and demodulator) if the seeker uses a mechanically spinning reticle.

    return processed_data;
}
