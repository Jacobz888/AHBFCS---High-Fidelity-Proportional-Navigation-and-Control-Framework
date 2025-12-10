#include "pro_nav.h"
#include "system_config.h"      // For NAV_CONSTANT_N, MAX_COMMANDED_ACCEL_M_PER_S2
#include "utilities/math_ops.h" // For vector math, saturation, etc.

/**
 * @brief Construct a new ProNavGuidance object.
 */
ProNavGuidance::ProNavGuidance() 
{
    // Initialize previous LOS angles for numerical differentiation
    prev_los_pitch_rad_ = 0.0f;
    prev_los_yaw_rad_ = 0.0f;
}

/**
 * @brief Calculates the commanded lateral acceleration vector using the PN Law.
 * * @param state_estimate The current optimal state vector (from EKF).
 * @param seeker_data Current Line-of-Sight (LOS) angle error from the seeker.
 * @return CommandAccel_t The required lateral acceleration in m/s^2.
 */
CommandAccel_t ProNavGuidance::calculate_command(
    const StateVector_t& state_estimate, 
    const SeekerData_t& seeker_data)
{
    const float dt = GNC_DELTA_T;
    
    // --- 1. Calculate the Line-of-Sight (LOS) Rate (lambda_dot) ---
    // The LOS rate is the rate at which the target's bearing changes relative to the vehicle.
    // This is achieved by numerically differentiating the seeker's measured LOS angle error.

    float current_los_pitch = seeker_data.los_pitch_angle_rad;
    float current_los_yaw = seeker_data.los_yaw_angle_rad;

    // Numerical Differentiation: (Current Angle - Previous Angle) / Delta-T
    float lambda_dot_pitch = (current_los_pitch - prev_los_pitch_rad_) / dt;
    float lambda_dot_yaw = (current_los_yaw - prev_los_yaw_rad_) / dt;

    // NOTE: In a robust system, lambda_dot would be passed through a Low-Pass Filter (LPF)
    // to reduce differentiation noise before being used in the PN law.
    // lambda_dot_pitch = LPF::filter(lambda_dot_pitch, LOS_RATE_LPF_ALPHA);
    
    // --- 2. Calculate Vehicle Velocity (V) ---
    // Use the velocity components from the EKF state estimate.
    float V_total = magnitude(state_estimate.Vx, state_estimate.Vy, state_estimate.Vz);
    
    // Safety check: Avoid division by zero or errors at very low speed (pre-launch)
    if (V_total < 50.0f) {
        return {0.0f, 0.0f}; // Return zero command if speed is insufficient
    }

    // --- 3. Apply the Proportional Navigation Law ---
    // A_c = N * V * lambda_dot

    const float N = NAV_CONSTANT_N; // Navigation Constant from system_config.h

    CommandAccel_t command;
    
    // The command is perpendicular to the LOS rate
    command.A_pitch_cmd_M_S2 = N * V_total * lambda_dot_pitch;
    command.A_yaw_cmd_M_S2   = N * V_total * lambda_dot_yaw;

    // --- 4. Apply Saturation ---
    // Limit the commanded acceleration to the maximum physically achievable 
    // aerodynamic maneuverability of the vehicle.
    
    command.A_pitch_cmd_M_S2 = saturate(
        command.A_pitch_cmd_M_S2, 
        -MAX_COMMANDED_ACCEL_M_PER_S2, 
        +MAX_COMMANDED_ACCEL_M_PER_S2
    );

    command.A_yaw_cmd_M_S2 = saturate(
        command.A_yaw_cmd_M_S2, 
        -MAX_COMMANDED_ACCEL_M_PER_S2, 
        +MAX_COMMANDED_ACCEL_M_PER_S2
    );

    // --- 5. Update History ---
    prev_los_pitch_rad_ = current_los_pitch;
    prev_los_yaw_rad_ = current_los_yaw;

    return command;
}
