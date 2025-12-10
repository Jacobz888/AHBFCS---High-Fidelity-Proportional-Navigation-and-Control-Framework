#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// -----------------------------------------------------------------------------
// 1. GNC TIMING AND FREQUENCY DEFINITIONS
//
// These parameters define the speed and determinism of the control loop.
// -----------------------------------------------------------------------------

/**
 * @brief GNC Loop frequency in Hertz (Hz).
 * A high frequency is crucial for stability in high-dynamic systems.
 * Typical range: 200.0 Hz to 400.0 Hz.
 */
#define GNC_FREQUENCY_HZ 250.0f 

/**
 * @brief The time step (Delta-T) of the GNC loop in seconds.
 * Calculated as 1.0 / GNC_FREQUENCY_HZ.
 */
#define GNC_DELTA_T (1.0f / GNC_FREQUENCY_HZ)

// -----------------------------------------------------------------------------
// 2. PHYSICAL AND AERODYNAMIC CONSTRAINTS
//
// These constants are required by the EKF and Control Mixer for dynamic modeling.
// -----------------------------------------------------------------------------

/**
 * @brief Total estimated mass of the vehicle in kilograms (kg).
 * Used in dynamic calculations (F=ma).
 */
#define VEHICLE_MASS_KG 5.5f

/**
 * @brief Gravitational acceleration (m/s^2).
 * Used for gravity compensation in the accelerometer readings.
 */
#define GRAVITY_M_PER_S2 9.80665f

/**
 * @brief Maximum aerodynamic acceleration the vehicle can achieve (m/s^2).
 * Used to saturate the Proportional Navigation (PN) acceleration command (A_c).
 * This prevents the control system from demanding unachievable maneuvers.
 */
#define MAX_COMMANDED_ACCEL_M_PER_S2 40.0f 

/**
 * @brief Characteristic length or reference diameter (meters).
 * Used in aerodynamic calculations.
 */
#define REF_DIAMETER_M 0.127f

// -----------------------------------------------------------------------------
// 3. PROPORTIONAL NAVIGATION (PN) PARAMETERS
//
// These are the core parameters for the guidance law (A_c = N * V * lambda_dot).
// -----------------------------------------------------------------------------

/**
 * @brief Effective Navigation Constant (N).
 * Must be tuned for optimal intercept trajectory. N=3 is energy optimal; 
 * N=4 to 5 provides a faster, more aggressive intercept.
 */
#define NAV_CONSTANT_N 4.5f

/**
 * @brief Filter constant (alpha) for numerical differentiation of LOS rate (lambda_dot).
 * A value closer to 1.0 means less filtering (more responsive, more noisy).
 * Range: 0.8f to 0.99f.
 */
#define LOS_RATE_LPF_ALPHA 0.9f 

// -----------------------------------------------------------------------------
// 4. ACTUATOR AND CONTROL LIMITS
//
// Defines the physical limits of the control surfaces.
// -----------------------------------------------------------------------------

/**
 * @brief Maximum physical deflection angle of the control fins (radians).
 * Used to saturate the PID output before mixing.
 */
#define MAX_FIN_DEFLECTION_RAD 0.5236f // Approx 30 degrees

/**
 * @brief Neutral PWM duty cycle (center position) for the servos (microseconds).
 * Standard hobby servos: 1500 us.
 */
#define SERVO_NEUTRAL_PWM_US 1500

/**
 * @brief Range of motion from neutral (microseconds).
 */
#define SERVO_PWM_RANGE_US 500

// -----------------------------------------------------------------------------
// 5. HARDWARE INTERFACE DEFINITIONS
//
// Defines the ports and pins for the embedded system peripherals.
// -----------------------------------------------------------------------------

/**
 * @brief SPI port for the high-speed IMU sensor (e.g., MPU-9250).
 */
#define IMU_SPI_PORT SPI1

/**
 * @brief ADC channel for the Seeker's Quadrant 1 infrared sensor.
 */
#define SEEKER_Q1_ADC_CHANNEL 1

/**
 * @brief PWM Timer and Channel for Servo Fin 1 (Pitch/Yaw Actuator).
 */
#define FIN1_PWM_TIMER TIM4
#define FIN1_PWM_CHANNEL 1

// -----------------------------------------------------------------------------
// 6. EKF AND SENSOR NOISE PARAMETERS
//
// Defines the variance of sensor readings for the Kalman Filter's R-matrix.
// -----------------------------------------------------------------------------

/**
 * @brief Variance (sigma^2) of the accelerometer noise (m/s^2)^2.
 * Must be determined through physical calibration.
 */
#define ACCEL_NOISE_VARIANCE 0.001f

/**
 * @brief Variance of the gyroscope noise (rad/s)^2.
 */
#define GYRO_NOISE_VARIANCE 0.00005f

#endif // SYSTEM_CONFIG_H
