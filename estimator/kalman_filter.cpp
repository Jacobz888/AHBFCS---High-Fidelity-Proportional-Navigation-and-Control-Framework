#include "kalman_filter.h"
#include "system_config.h" 
#include "utilities/matrix_ops.h" // Essential for all matrix math
#include "utilities/math_ops.h"   // For quaternion and rotation math

// Define matrix dimensions (N=State size, M=Measurement size)
#define N_STATE 15  // Example: Position (3), Velocity (3), Attitude (4 Quat), Bias (5)
#define M_MEASUREMENT 10 // Example: Accel (3), Gyro (3), Mag (3), Seeker LOS Rate (1)

/**
 * @brief Construct a new EKF object.
 * Initializes the state vector (X), Process Covariance (P), Process Noise (Q), 
 * and Measurement Noise (R) matrices.
 */
KalmanFilter::KalmanFilter() 
{
    // Initialize the 15x1 State Vector (X_hat)
    MatrixOps::zero(X_hat, N_STATE, 1);
    
    // Initialize the 15x15 Process Covariance Matrix (P)
    // Set diagonal elements high initially to represent high uncertainty.
    MatrixOps::set_identity_diagonal(P_covar, N_STATE, 100.0f);
    
    // Initialize 15x15 Process Noise Matrix (Q)
    // Derived from Q_PROCESS_GYRO_VAR, representing confidence in the system model.
    MatrixOps::set_identity_diagonal(Q_noise, N_STATE, Q_PROCESS_GYRO_VAR); 

    // Initialize MxM Measurement Noise Matrix (R)
    // Derived from R_MEASUREMENT_LOS_VAR, representing confidence in the sensors.
    MatrixOps::set_identity_diagonal(R_noise, M_MEASUREMENT, R_MEASUREMENT_LOS_VAR); 
}

/**
 * @brief Predicts the system state to the next time step (k+1).
 * * Uses the non-linear physics model and high-rate IMU data (gyro/accel).
 * * @param raw_imu_data Current raw IMU readings.
 */
void KalmanFilter::predict_step(const RawIMUData_t& raw_imu_data)
{
    const float dt = GNC_DELTA_T;
    
    // --- 1. State Prediction (Non-linear Model: X_k+1|k = f(X_k|k, u_k)) ---
    // Update the state vector (X_hat) using the kinematic model (thrust, drag, gravity)
    // and the high-frequency gyroscope and accelerometer inputs (u_k).
    
     X_hat_k_plus_1 = integrate_kinematics(X_hat, raw_imu_data, dt);
    
    // Example: Attitude update using quaternions and gyro rates (p, q, r)
     X_hat_quat = QuaternionOps::integrate(X_hat_quat, raw_imu_data.gyro_rates, dt);

    // --- 2. Covariance Prediction (Linearized Model: P_k+1|k = F * P * F^T + Q) ---
    // The Covariance Matrix (P) tracks the uncertainty of the state estimate.
    
    // Calculate the 15x15 Jacobian Matrix (F) - the linearization of f() around X_hat.
     F_matrix = Calculate_Jacobian_F(X_hat, raw_imu_data); 
    
     P_covar = F_matrix * P_covar * F_matrix_T + Q_noise;
}


/**
 * @brief Corrects the predicted state (X_k+1|k) using the noisy measurements.
 * * Uses lower-rate, higher-precision data (e.g., seeker LOS, GPS).
 * * @param raw_seeker_data Current raw seeker readings.
 */
void KalmanFilter::correct_step(const RawSeekerData_t& raw_seeker_data)
{
    // --- 1. Innovation Covariance (S) ---
    // The matrix S represents the uncertainty of the measurement prediction.
     S = H * P * H^T + R
    
    // Calculate the MxN Jacobian Matrix (H) - the linearization of h() around X_hat.
     H_matrix = Calculate_Jacobian_H(X_hat); 

     S_covar = H_matrix * P_covar * H_matrix_T + R_noise;


    // --- 2. Kalman Gain (K) ---
    // The Kalman Gain determines how much the measurement should influence the estimate.
     K = P * H^T * S^-1
     K_gain = P_covar * H_matrix_T * MatrixOps::invert(S_covar);


    // --- 3. Measurement Residual (Y) ---
     Y = Z_k+1 - h(X_k+1|k)
     Residual = (Raw Seeker LOS Rate) - (Predicted Seeker LOS Rate from X_hat)


    // --- 4. State Update (X_k+1|k+1 = X_k+1|k + K * Y) ---
     X_hat_updated = X_hat_predicted + K_gain * Residual_vector;


    // --- 5. Covariance Update (P_k+1|k+1 = (I - K * H) * P_k+1|k) ---
     P_covar = (Identity_matrix - K_gain * H_matrix) * P_covar;

     After correction, the State Vector X_hat holds the optimal estimate.
}

/**
 * @brief Returns the current best estimate of the system state.
 */
StateVector_t KalmanFilter::get_state_estimate() const
{
    // Convert the 15-element X_hat vector into a user-friendly structure
    // (e.g., extracting position, velocity, and Euler angles from the quaternion).
    return convert_vector_to_struct(X_hat);
}
