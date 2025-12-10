#ifndef CONTROL_GAINS_H
#define CONTROL_GAINS_H

// -----------------------------------------------------------------------------
// 1. ROLL AXIS CONTROL GAINS (Inner Loop - Rate Stabilization)
//
// The Roll axis is critical for stability and seeker performance. 
// This loop controls the Roll Rate (p) to zero (or to dampen roll oscillations).
// Units are typically (Torque Command) / (Error Unit).
// -----------------------------------------------------------------------------

/**
 * @brief Proportional Gain (Kp) for Roll Rate Control.
 * Controls the immediate response to roll rate error (p_error).
 */
#define KP_ROLL_RATE 15.0f

/**
 * @brief Integral Gain (Ki) for Roll Rate Control.
 * Eliminates steady-state roll error. Often set low or zero for rate loops.
 */
#define KI_ROLL_RATE 0.05f

/**
 * @brief Derivative Gain (Kd) for Roll Rate Control.
 * Dampens oscillations and provides lead compensation.
 */
#define KD_ROLL_RATE 0.8f

// -----------------------------------------------------------------------------
// 2. PITCH AXIS CONTROL GAINS (Nested Loops)
//
// Controls both the angular pitch rate (q) and tracks the commanded acceleration (A_c_pitch).
// -----------------------------------------------------------------------------

// --- Inner Loop: Pitch Rate (q) Stabilization ---
/**
 * @brief Proportional Gain (Kp) for Pitch Rate Control.
 */
#define KP_PITCH_RATE 12.5f

/**
 * @brief Derivative Gain (Kd) for Pitch Rate Control.
 */
#define KD_PITCH_RATE 0.6f


// --- Outer Loop: Pitch Acceleration (A_c) Tracking ---
/**
 * @brief Proportional Gain (Kp) for Pitch Acceleration Control.
 * Converts the Acceleration Error (A_c_cmd - A_measured) into a rate command.
 */
#define KP_PITCH_ACCEL 0.05f 

/**
 * @brief Integral Gain (Ki) for Pitch Acceleration Control.
 * Used to eliminate steady-state acceleration bias/error.
 */
#define KI_PITCH_ACCEL 0.005f


// -----------------------------------------------------------------------------
// 3. YAW AXIS CONTROL GAINS (Nested Loops)
//
// Controls both the angular yaw rate (r) and tracks the commanded acceleration (A_c_yaw).
// Typically mirrors the Pitch gains due to symmetric aerodynamics.
// -----------------------------------------------------------------------------

// --- Inner Loop: Yaw Rate (r) Stabilization ---
/**
 * @brief Proportional Gain (Kp) for Yaw Rate Control.
 */
#define KP_YAW_RATE 12.5f

/**
 * @brief Derivative Gain (Kd) for Yaw Rate Control.
 */
#define KD_YAW_RATE 0.6f


// --- Outer Loop: Yaw Acceleration (A_c) Tracking ---
/**
 * @brief Proportional Gain (Kp) for Yaw Acceleration Control.
 */
#define KP_YAW_ACCEL 0.05f

/**
 * @brief Integral Gain (Ki) for Yaw Acceleration Control.
 */
#define KI_YAW_ACCEL 0.005f


// -----------------------------------------------------------------------------
// 4. EKF (KALMAN FILTER) PROCESS AND MEASUREMENT NOISE COVARIANCES
//
// These are not PID gains, but they are critical tuning parameters for the EKF.
// They define the confidence in the system model vs. the sensor measurements.
// (Often defined as diagonal matrix elements)
// -----------------------------------------------------------------------------

/**
 * @brief Process Noise Covariance Q (Gyro/Accel terms). 
 * Defines how much the system model is trusted over time.
 */
#define Q_PROCESS_GYRO_VAR 0.0001f

/**
 * @brief Measurement Noise Covariance R (LOS Seeker term).
 * Defines how much the seeker measurement is trusted.
 */
#define R_MEASUREMENT_LOS_VAR 0.01f

#endif // CONTROL_GAINS_H
