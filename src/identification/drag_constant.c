#include "FreeRTOS.h"      // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"          // FreeRTOS task functions (e.g., vTaskDelay)
#include "supervisor.h"    // Functions to check flight status (e.g., supervisorIsArmed)
#include "commander.h"     // Access to commanded setpoints (e.g., commanderGetSetpoint)
#include "motors.h"        // Low-level motor control interface (e.g., motorsSetRatio)

// Motor coefficients of the quadratic model: PWM = a_2 * omega^2 + a_1 * omega
const float a_2 = 6.9732e-8f;
const float a_1 = 1.9762e-4f;
//Kl = 2.6e-6f; --- IGNORE ---
//Kd = 1.0712e-10f; --- IGNORE ---

// Global variables to store the desired setpoint, the current state (not used here),
// and the computed PWM values for different motor speeds
setpoint_t setpoint;
state_t state;
float pwm_1, pwm_2;
float omega_1, omega_2; 

// Main application
void appMain(void *param)
{
    // Infinite loop (runs forever)
    while (true)
    {
        // Check if the drone is armed (i.e., ready to fly)
        if (supervisorIsArmed())
        {
            // Fetch the latest setpoint from the commander
            commanderGetSetpoint(&setpoint, &state);

            if ((setpoint.position.z) > 0)
            {
                // Set two different angular velocities for the motors
                // Motors M1 and M3 will spin at 2000 rad/s
                // Motors M2 and M4 will spin at 1000 rad/s
                // This configuration induces pure yaw rotation (spinning in place)
                omega_1 = 2000.0f;
                omega_2 = 1000.0f;

                // Convert angular velocities to PWM using the motor model
                pwm_1 = a_2 * omega_1 * omega_1 + a_1 * omega_1;
                pwm_2 = a_2 * omega_2 * omega_2 + a_1 * omega_2;
            }
            else
            {
                // If Z setpoint is not positive, apply minimal power to all motors (for idle spin)
                pwm_1 = 0.1f;
                pwm_2 = 0.1f;
            }
        }
        else
        {
            // If not armed, stop all motors
            pwm_1 = 0.0f;
            pwm_2 = 0.0f;
        }
        // Apply PWM to motors:
        // M1 and M3 get pwm_1 (corresponding to 2000 rad/s)
        // M2 and M4 get pwm_2 (corresponding to 1000 rad/s)
        // This asymmetric configuration results in yaw motion
        motorsSetRatio(MOTOR_M1, pwm_1 * UINT16_MAX);
        motorsSetRatio(MOTOR_M2, pwm_2 * UINT16_MAX);
        motorsSetRatio(MOTOR_M3, pwm_1 * UINT16_MAX);
        motorsSetRatio(MOTOR_M4, pwm_2 * UINT16_MAX);
        // Wait for 100 milliseconds before the next iteration (10 Hz control loop)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}