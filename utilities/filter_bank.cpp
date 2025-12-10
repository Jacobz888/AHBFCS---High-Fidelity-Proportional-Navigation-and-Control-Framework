#include "filter_bank.h"
#include "system_config.h" // For GNC_DELTA_T
#include "utilities/math_ops.h" 

// --- Digital Filter Structure ---
// Encapsulates the state required for a single-input, single-output filter.
struct DigitalFilterState {
    float previous_output;
    float cutoff_frequency_hz;
    float time_constant_s;
};

/**
 * @brief Initializes a new digital filter with a specified cutoff frequency.
 * * @param filter_state Pointer to the filter structure.
 * @param cutoff_freq_hz The frequency at which signals should begin to be attenuated.
 */
void FilterBank::initialize_filter(DigitalFilterState* filter_state, float cutoff_freq_hz)
{
    filter_state->previous_output = 0.0f;
    filter_state->cutoff_frequency_hz = cutoff_freq_hz;
    
    // Calculate the time constant tau (T) from the cutoff frequency (f_c): T = 1 / (2 * pi * f_c)
    filter_state->time_constant_s = 1.0f / (2.0f * M_PI * cutoff_freq_hz);
}


/**
 * @brief Applies an Exponential Moving Average (EMA) / Digital Low-Pass Filter (LPF) to a signal.
 * * This is a simple, computationally efficient first-order filter.
 * * @param filter_state Pointer to the initialized filter structure.
 * @param input_sample The new raw measurement value.
 * @return float The filtered (smoothed) output value.
 */
float FilterBank::apply_lpf(DigitalFilterState* filter_state, float input_sample)
{
    const float dt = GNC_DELTA_T;
    
    // --- 1. Calculate the Filter Coefficient (Alpha) ---
    // Alpha determines the smoothness vs. lag trade-off.
    // Alpha is calculated from the time constant (T) and the sample time (dt).
    // The formula results in a filter that effectively matches the specified cutoff frequency.
    float alpha = filter_state->time_constant_s / (filter_state->time_constant_s + dt);
    
    // --- 2. Apply the Digital Filter Equation (Exponential Moving Average) ---
    // Output_k = Alpha * Output_{k-1} + (1 - Alpha) * Input_k
    
    float current_output = 
        alpha * filter_state->previous_output + 
        (1.0f - alpha) * input_sample;
        
    // --- 3. Update Filter State ---
    filter_state->previous_output = current_output;
    
    return current_output;
}


// --- Example Initialization and Usage in the main loop ---

// Declare filter states globally or within a class instance
DigitalFilterState gyro_x_filter;
DigitalFilterState accel_z_filter;
DigitalFilterState los_rate_filter;

/**
 * @brief Example function showing how filters are initialized and used.
 */
void FilterBank::example_usage()
{
    // Initialize filters at system startup:
    // Gyroscope data is often noisy but requires a high cutoff (e.g., 50Hz)
    initialize_filter(&gyro_x_filter, 50.0f);
    
    // Accelerometer data for the control loop might use a lower cutoff (e.g., 20Hz)
    initialize_filter(&accel_z_filter, 20.0f); 

    // LOS rate differentiation is extremely noisy and requires aggressive filtering (e.g., 5Hz)
    initialize_filter(&los_rate_filter, 5.0f); 
    
    // --- In the main GNC loop ---
    
    // float raw_gyro_x = IMUDriver::get_raw_gyro_x();
    // float filtered_gyro_x = apply_lpf(&gyro_x_filter, raw_gyro_x);
    
    // // This filtered value is then passed to the EKF prediction step
    // EKF::predict(filtered_gyro_x, ...);
    
    // // This filter smooths the numerically differentiated LOS rate before PN:
    // // float filtered_lambda_dot = apply_lpf(&los_rate_filter, raw_lambda_dot);
    // // ProNav::calculate(filtered_lambda_dot, ...);
}
