#include "control_mixer.h"
#include "system_config.h" // For MAX_FIN_DEFLECTION_RAD, SERVO_NEUTRAL_PWM_US
#include "utilities/math_ops.h" // Assumes utility functions for saturation, etc.

// Define the Control Mixing Matrix (Conceptual)
// This matrix depends on the geometric arrangement and orientation of the four control fins.
// Assuming a typical 'X' configuration (fins at 45, 135, 225, 315 degrees):
//
//               [ Fin1 ]   [ +1   +1   -1 ]   [ Pitch Command ]
// Fin Deflection [ Fin2 ] = [ -1   +1   +1 ] * [ Yaw Command   ]
//               [ Fin3 ]   [ -1   -1   -1 ]   [ Roll Command  ]
//               [ Fin4 ]   [ +1   -1   +1 ]
//
// The coefficients (e.g., +1, -1) represent the effectiveness and sign of the 
// control input on that specific fin.
//
const float MIXING_MATRIX[4][3] = {
    // Pitch, Yaw, Roll
    {+1.0f, +1.0f, -1.0f}, // Fin 1 (e.g., Top-Right)
    {-1.0f, +1.0f, +1.0f}, // Fin 2 (e.g., Top-Left)
    {-1.0f, -1.0f, -1.0f}, // Fin 3 (e.g., Bottom-Left)
    {+1.0f, -1.0f, +1.0f}  // Fin 4 (e.g., Bottom-Right)
};


/**
 * @brief Translates abstract Pitch/Yaw/Roll commands into physical fin deflections.
 * * @param pitch_cmd_rad Output from Pitch PID (Desired deflection in radians).
 * @param yaw_cmd_rad Output from Yaw PID (Desired deflection in radians).
 * @param roll_cmd_rad Output from Roll PID (Desired deflection in radians).
 * @return FinCommand_t A structure containing the individual PWM commands for the 4 fins.
 */
FinCommand_t ControlMixer::calculate_commands(
    float pitch_cmd_rad, 
    float yaw_cmd_rad, 
    float roll_cmd_rad) 
{
    FinCommand_t fin_commands;
    float fin_deflections_rad[4];
    
    // --- 1. Apply Mixing Matrix (Matrix Multiplication) ---
    // Iterate through the four fins
    for (int i = 0; i < 4; ++i) {
        // Calculate the raw deflection required for Fin i
        float raw_deflection = 
            MIXING_MATRIX[i][0] * pitch_cmd_rad + // Pitch contribution
            MIXING_MATRIX[i][1] * yaw_cmd_rad +   // Yaw contribution
            MIXING_MATRIX[i][2] * roll_cmd_rad;   // Roll contribution
            
        // --- 2. Apply Saturation Limit ---
        // The deflection must not exceed the physical limit of the fin's movement.
        fin_deflections_rad[i] = saturate(
            raw_deflection, 
            -MAX_FIN_DEFLECTION_RAD, 
            +MAX_FIN_DEFLECTION_RAD
        );
    }
    
    // --- 3. Convert Radians to PWM Duty Cycle (Actuation Mapping) ---
    // Convert the angular command (radians) into a pulse-width modulation (PWM) 
    // signal required by the hobby servo motors (e.g., 1000us to 2000us).
    
    for (int i = 0; i < 4; ++i) {
        // Calculate the fraction of maximum deflection (range: -1.0 to +1.0)
        float deflection_ratio = fin_deflections_rad[i] / MAX_FIN_DEFLECTION_RAD;

        // Map the ratio to the PWM signal range:
        // PWM = SERVO_NEUTRAL_PWM_US + (deflection_ratio * SERVO_PWM_RANGE_US)
        
        uint16_t pwm_value = (uint16_t)(
            SERVO_NEUTRAL_PWM_US + (deflection_ratio * SERVO_PWM_RANGE_US)
        );

        // Final saturation to ensure PWM stays within 1000us-2000us bounds
        pwm_value = saturate_int(pwm_value, 
            SERVO_NEUTRAL_PWM_US - SERVO_PWM_RANGE_US, 
            SERVO_NEUTRAL_PWM_US + SERVO_PWM_RANGE_US
        );

        // Assign to the output structure
        switch (i) {
            case 0: fin_commands.pwm_fin1 = pwm_value; break;
            case 1: fin_commands.pwm_fin2 = pwm_value; break;
            case 2: fin_commands.pwm_fin3 = pwm_value; break;
            case 3: fin_commands.pwm_fin4 = pwm_value; break;
        }
    }

    return fin_commands;
}
