#include "system_config.h"
#include "control_gains.h"
#include "kalman_filter.h"
#include "pid_controller.h"
#include "pro_nav.h"
#include "control_mixer.h"
#include "sensors/imu_driver.h"
#include "sensors/seeker_interface.h"
#include "utilities/filter_bank.h"
#include "hal/hal_system.h" // Assumed system initialization and clock control

// --- Global System Instances (The GNC components) ---

// 1. Estimator
static KalmanFilter EKF; 

// 2. Guidance
static ProNavGuidance ProNav;

// 3. Control (PID Controllers)
// Inner Loop (Rate Stabilization)
static PIDController RollRatePID(KP_ROLL_RATE, KI_ROLL_RATE, KD_ROLL_RATE, 
                                MAX_FIN_DEFLECTION_RAD, 0.5f); // Roll-rate integral windup limit

// Outer Loop (Acceleration Tracking) - Uses the same PID logic but different gains
static PIDController PitchAccelPID(KP_PITCH_ACCEL, KI_PITCH_ACCEL, 0.0f, 
                                 MAX_FIN_DEFLECTION_RAD, 0.01f); 
static PIDController YawAccelPID(KP_YAW_ACCEL, KI_YAW_ACCEL, 0.0f, 
                               MAX_FIN_DEFLECTION_RAD, 0.01f); 
// Note: Rate control is managed implicitly by the PID outputs and the Control Mixer

// 4. Actuation
static ControlMixer Mixer;

// 5. Drivers/Interfaces
static IMUDriver Imu;
static SeekerInterface Seeker;

// --- GNC Loop Function Prototypes ---
void GNC_Control_Loop();
void System_Initialize();


/**
 * @brief The main entry point of the embedded application.
 * Handles system power-up and sets up the RTOS/Scheduler.
 */
int main(void) 
{
    // 1. Initialize all peripherals and core systems
    System_Initialize();

    // 2. Start the GNC loop using a fixed-rate timer interrupt or RTOS task
    // (This is the critical step for deterministic execution)
    HAL_Start_Fixed_Rate_Timer(GNC_FREQUENCY_HZ, GNC_Control_Loop); 
    
    // 3. Loop forever (or idle if using an RTOS)
    while (1) {
        // Lower-priority tasks (Telemetry, Health Monitoring, etc.) run here
        HAL_Delay_ms(100); 
    }
    return 0;
}


/**
 * @brief Performs all system-level initialization (GPIO, SPI, Clocks, etc.).
 */
void System_Initialize() 
{
    // Initialize MCU clock, power, and watchdog timers
    HAL_System_Init(); 

    // Initialize communication peripherals used by sensors/actuators
    HAL_SPI_Init(IMU_SPI_PORT, ...); 
    HAL_PWM_Init(FIN1_PWM_TIMER, ...);

    // Initialize sensor drivers and perform self-tests
    if (!Imu.initialize()) {
        // Log critical failure and enter safe mode
        while(1) { /* Blink error LED */ }
    }
    
    // Initialize the EKF (Setting initial P and Q matrices)
    EKF.initialize(); 

    // Log initialization complete and wait for arming signal
    // ...
}


/**
 * @brief The core Guidance and Control function, executed deterministically 
 * at GNC_FREQUENCY_HZ (e.g., every 4ms).
 * This function must execute entirely within its time budget!
 */
void GNC_Control_Loop()
{
    // --- 1. Data Acquisition ---
    IMUData_t imu_data = Imu.read_data();
    SeekerData_t seeker_data = Seeker.get_processed_data(); // Assuming seeker processing happens separately or here

    // --- 2. State Estimation (EKF) ---
    // Predict: Use raw high-rate IMU data and kinematics
    EKF.predict_step(imu_data); 
    
    // Correct: Use the seeker LOS angle for the measurement update
    if (seeker_data.is_locked) {
        EKF.correct_step(seeker_data);
    }
    
    StateVector_t state = EKF.get_state_estimate();

    // --- 3. Guidance Law (Pro Nav) ---
    // The PN law needs the LOS angle and the current vehicle velocity from the EKF state.
    CommandAccel_t accel_command = ProNav.calculate_command(state, seeker_data);

    // --- 4. Control Law (PID) ---
    
    // Calculate required fin deflection based on acceleration error.
    // NOTE: This assumes a cascade control where the outer PID (Accel) outputs
    // a deflection, which then directly drives the mixer.
    
    float pitch_deflection_cmd_rad = PitchAccelPID.calculate(
        accel_command.A_pitch_cmd_M_S2, 
        state.accel_pitch_mps2 // Measured acceleration from EKF state
    );
    
    float yaw_deflection_cmd_rad = YawAccelPID.calculate(
        accel_command.A_yaw_cmd_M_S2, 
        state.accel_yaw_mps2 // Measured acceleration from EKF state
    );
    
    // The Roll PID is run separately to maintain roll stability (Roll Rate to zero setpoint)
    float roll_deflection_cmd_rad = RollRatePID.calculate(
        0.0f, // Setpoint of 0.0 rad/s
        state.roll_rate_rad_s // Measured roll rate from EKF state
    );


    // --- 5. Actuation and Mixing ---
    // Convert the Pitch, Yaw, and Roll deflection commands into 4 specific PWM outputs.
    FinCommand_t fin_pwm_commands = Mixer.calculate_commands(
        pitch_deflection_cmd_rad, 
        yaw_deflection_cmd_rad, 
        roll_deflection_cmd_rad
    );

    // Send the final commands to the hardware registers via the HAL
    // The function call must be synchronized/atomic.
    // HAL_PWM_Set_Fins(fin_pwm_commands); 
    
    // --- 6. Telemetry and Logging ---
    // Log the current state and commands to flash/telemetry stream (non-critical)
    // Telemetry_Log_State(state, accel_command);
}
