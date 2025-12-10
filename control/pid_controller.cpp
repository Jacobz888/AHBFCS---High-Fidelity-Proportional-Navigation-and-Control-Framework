#include "pid_controller.h"
#include "system_config.h" // For GNC_DELTA_T
#include "utilities/math_ops.h" // Assumes utility functions for saturation

/**
 * @brief Construct a new PID Controller object.
 * * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param out_limit Maximum output saturation limit (e.g., max fin deflection).
 * @param integral_limit Maximum limit for the integral term (anti-windup).
 */
PIDController::PIDController(float Kp, float Ki, float Kd, 
                             float out_limit, float integral_limit)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), output_limit_(out_limit), integral_limit_(integral_limit)
{
    // Initialize all states to zero upon creation
    integral_sum_ = 0.0f;
    previous_error_ = 0.0f;
    previous_measurement_ = 0.0f;
}

/**
 * @brief Calculates the control output based on the error.
 * * This implements the discrete-time PID algorithm.
 * * @param setpoint The desired value (e.g., A_c command, 0.0 roll rate).
 * @param measured_value The actual sensor reading (e.g., measured accel, gyro rate).
 * @return float The control output (e.g., desired fin deflection angle in radians).
 */
float PIDController::calculate(float setpoint, float measured_value)
{
    // Ensure the time step is correctly defined (pulled from system_config.h)
    const float dt = GNC_DELTA_T;
    
    // 1. Calculate the Error Term
    float error = setpoint - measured_value;
    
    // --- P Term: Proportional Term (Immediate Response) ---
    float p_out = Kp_ * error;

    // --- I Term: Integral Term (Steady-State Elimination) ---
    // Accumulate the error over time
    integral_sum_ += error * dt;
    
    // Apply Anti-Windup: Limit the accumulated integral sum to prevent overshoot 
    // after large, sustained errors.
    integral_sum_ = saturate(integral_sum_, -integral_limit_, integral_limit_);
    
    float i_out = Ki_ * integral_sum_;
    
    // --- D Term: Derivative Term (Damping and Prediction) ---
    // The derivative can be calculated from either the error or the measurement.
    // Calculating from the measurement (D-on-Measurement) is often preferred 
    // in GNC as it avoids large spikes when the setpoint changes instantly.
    
    // d_error/dt ≈ (current_measurement - previous_measurement) / dt
    float derivative_of_measurement = (measured_value - previous_measurement_) / dt;
    
    // Apply a negative sign because the derivative term damps the system.
    float d_out = -Kd_ * derivative_of_measurement; 
    
    // --- 4. Sum the Components ---
    float control_output = p_out + i_out + d_out;
    
    // --- 5. Apply Saturation ---
    // The output must be limited to the physical constraints of the actuator 
    // (e.g., MAX_FIN_DEFLECTION_RAD).
    control_output = saturate(control_output, -output_limit_, output_limit_);

    // --- 6. Update History ---
    // previous_error_ = error; // (Only needed if D-on-Error is used)
    previous_measurement_ = measured_value;

    return control_output;
}
