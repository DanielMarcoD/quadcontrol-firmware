#include "math.h"       // Math functions (e.g., sqrtf, roundf, powf)
#include "FreeRTOS.h"   // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"       // FreeRTOS task functions (e.g., vTaskDelay)
#include "supervisor.h" // Functions to check flight status (e.g., supervisorIsArmed)
#include "commander.h"  // Access to commanded setpoints (e.g., commanderGetSetpoint)
#include "estimator.h"  // Estimation framework for sensor fusion
#include "motors.h"     // Low-level motor control interface (e.g., motorsSetRatio)
#include "debug.h"      // Debug printing functions (e.g., DEBUG_PRINT)
#include "log.h"        // Logging utilities to send data to the CFClient
#define M_PI 3.14159265358979323846


// Sensors
float ax, ay, az;             // Accelerometer [m/s^2]
float gx, gy, gz;             // Gyroscope [rad/s]
float d;                      // Range [m]
float px, py;                 // Optical flow [m]

// System states
float phi, theta, psi;        // Euler angles [rad]
float wx, wy, wz;             // Angular velocities [rad/s]
float x, y, z;               // Positions [m]
float vx, vy, vz;            // Velocities [m/s]

// System references
float phi_r, theta_r, psi_r; // Euler angles reference [rad]
float z_r;


// Physical constants
static const float pi = 3.1416f; // Mathematical constant
static const float g = 9.81f;    // Gravitational acceleration [m/s^2]
static const float dt = 0.005f;  // Loop time step [s] (5 ms -> 200 Hz)

// Quadcopter parameters
static const float l = 35.0e-3f;   // Distance from motor to quadcopter center of mass [m]
static const float m = 38.6e-3f;   // Mass [kg]
static const float Ixx = 20.0e-6f; // Moment of inertia around x-axis [kg.m^2]
static const float Iyy = 20.0e-6f; // Moment of inertia around y-axis [kg.m^2]
static const float Izz = 40.0e-6f; // Moment of inertia around z-axis [kg.m^2]

// Actuators
float pwm1, pwm2, pwm3, pwm4; // Motors PWM

// System inputs
float ft;                     // Thrust force [N]
float tx, ty, tz;             // Roll, pitch and yaw torques [N.m]

// Auxiliary variables for logging Euler angles (CFClient uses degrees and not radians)
float log_phi, log_theta, log_psi;

// Logging group that stream variables to CFClient.
LOG_GROUP_START(stateEstimate)
LOG_ADD_CORE(LOG_FLOAT, roll, &log_phi)
LOG_ADD_CORE(LOG_FLOAT, pitch, &log_theta)
LOG_ADD_CORE(LOG_FLOAT, yaw, &log_psi)
LOG_ADD_CORE(LOG_FLOAT, x, &x)
LOG_ADD_CORE(LOG_FLOAT, y, &y)
LOG_ADD_CORE(LOG_FLOAT, z, &z)
LOG_ADD_CORE(LOG_FLOAT, vx, &vx)
LOG_ADD_CORE(LOG_FLOAT, vy, &vy)
LOG_ADD_CORE(LOG_FLOAT, vz, &vz)
LOG_GROUP_STOP(stateEstimate)

// Get reference setpoints from commander module
// void reference()
// {
//     // Declare variables that store the most recent setpoint and state from commander
//     static setpoint_t setpoint;
//     static state_t state;

//     // Retrieve the current commanded setpoints and state from commander module
//     commanderGetSetpoint(&setpoint, &state);

//     // Extract position references from the received setpoint
//     ft =  roundf((setpoint.position.z) * 2.0f) / 100.0f;    // Thrust force command [N] (maps 0.5m -> 0.01N)
//     tz =  roundf((setpoint.position.y) * 2.0f) / 5000.0f;   // Roll torque command [N.m] (maps 0.5m -> 0.001N.m)
//     ty =  roundf((setpoint.position.x) * 2.0f) / 1000.0f;   // Pitch torque command [N.m] (maps 0.5m -> 0.001N.m)
//     tx = 0.0f;                                              // Yaw torque command [N.m]

//     // Print debug info for the control efforts
//     //DEBUG_PRINT("Ft (N): %.2f | Tx (N.m): %.3f | Ty (N.m): %.3f  | Tz (N.m): %.3f \n", (double)ft, (double)tx, (double)ty, (double)tz);
// }

// Get reference setpoints from commander module
void reference()
{
    // Declare variables that store the most recent setpoint and state from commander
    static setpoint_t setpoint;
    static state_t state;

    // Retrieve the current commanded setpoints and state from commander module
    commanderGetSetpoint(&setpoint, &state);

    // Extract position references from the received setpoint
    z_r = setpoint.position.z;                        // Z position reference [m]
    phi_r = -(setpoint.position.y * 2.0f) * pi/4.0f/9.0f;   // Roll reference command [rad] (maps 0.5m -> pi/4 rad)
    theta_r = (setpoint.position.x * 2.0f) * pi/4.0f/9.0f; // Pitch reference command [rad] (maps 0.5m -> pi/4 rad)
    psi_r = 0.0f;                                     // Yaw reference command [rad]
}

// Compute motor commands
void mixer()
{
    // Quadcopter parameters
    static const float a2 = 6.9732e-8f; // Quadratic motor model gain [s^2/rad^2]
    static const float a1 = 1.9762e-4f; // Linear motor model gain [s/rad]
    static const float kl = 2.6e-8f; // Lift constant [N.s^2]
    static const float kd = 1.0712e-10f; // Drag constant [N.m.s^2]

    const float var1 = 1.0f / (4.0f * kl);
    const float var2 = 1.0f / (4.0f * kd);
    const float var3 = 1.0f / (4.0f * kl * l);

    // Compute required motor angular velocities squared (omega^2)
    float omega1 = var1 * ft - var3 * (tx + ty) - var2 * tz;
    float omega2 = var1 * ft - var3 * (tx - ty) + var2 * tz;
    float omega3 = var1 * ft + var3 * (tx + ty) - var2 * tz;
    float omega4 = var1 * ft + var3 * (tx - ty) + var2 * tz;

    // Clamp to non-negative and take square root (omega)
    omega1 = (omega1 < 0.0f) ? 0.0f : sqrtf(omega1);
    omega2 = (omega2 < 0.0f) ? 0.0f : sqrtf(omega2);
    omega3 = (omega3 < 0.0f) ? 0.0f : sqrtf(omega3);
    omega4 = (omega4 < 0.0f) ? 0.0f : sqrtf(omega4);

    // Compute motor PWM using motor model
    pwm1 = a2 * omega1 * omega1 + a1 * omega1;
    pwm2 = a2 * omega2 * omega2 + a1 * omega2;
    pwm3 = a2 * omega3 * omega3 + a1 * omega3;
    pwm4 = a2 * omega4 * omega4 + a1 * omega4;
}

// Apply motor commands
void actuators()
{
    // Check is quadcopter is armed or disarmed
    if (supervisorIsArmed())
    {
        if (z_r > 0.0f)
        {
            // Apply calculated PWM values if is commanded to take-off
            motorsSetRatio(MOTOR_M1, pwm1 * UINT16_MAX);
            motorsSetRatio(MOTOR_M2, pwm2 * UINT16_MAX);
            motorsSetRatio(MOTOR_M3, pwm3 * UINT16_MAX);
            motorsSetRatio(MOTOR_M4, pwm4 * UINT16_MAX);
        }
        else
        {
            // Apply calculated PWM values if is commanded to take-off
            motorsSetRatio(MOTOR_M1, 0.1f * UINT16_MAX);
            motorsSetRatio(MOTOR_M2, 0.1f * UINT16_MAX);
            motorsSetRatio(MOTOR_M3, 0.1f * UINT16_MAX);
            motorsSetRatio(MOTOR_M4, 0.1f * UINT16_MAX);
        }
    }
    else
    {
        // Turn-off all motor if disarmed
        motorsStop();
    }
}

// Get sensor readings from estimator module
void sensors()
{
    // Declare variable that store the most recent measurement from estimator
    static measurement_t measurement;

    // Retrieve the current measurement from estimator module
    while (estimatorDequeue(&measurement))
    {
        switch (measurement.type)
        {
        // Get accelerometer sensor readings and convert [G's -> m/s^2]
        case MeasurementTypeAcceleration:
            ax = -measurement.data.acceleration.acc.x * g;
            ay = -measurement.data.acceleration.acc.y * g;
            az = -measurement.data.acceleration.acc.z * g;
            break;
        // Get gyroscope sensor readings and convert [deg/s -> rad/s]
        case MeasurementTypeGyroscope:
            gx = measurement.data.gyroscope.gyro.x * pi / 180.0f;
            gy = measurement.data.gyroscope.gyro.y * pi / 180.0f;
            gz = measurement.data.gyroscope.gyro.z * pi / 180.0f;
            break;
        // Get flow sensor readings [m]
        case MeasurementTypeTOF:
            d = measurement.data.tof.distance;
            break;
        // Get optical flow sensor readings and convert [20px -> px]
        case MeasurementTypeFlow:
            px = measurement.data.flow.dpixelx * 0.05f;
            py = measurement.data.flow.dpixely * 0.05f;
            break;
        default:
            break;
        }
    }
}


// Estimate orientation from IMU sensor
void attitudeEstimator()
{
    // Estimator parameters
    static const float wc = 1.0f; // Cut-off frequency [rad/s]
    static const float alpha = wc * dt / (1.0f + wc * dt); // Complementary filter coefficient

    // Measured angles from accelerometer
    float phi_a = atan2f(-ay, -az);
    float theta_a = atan2f(ax, sqrtf(ay*ay + az*az));

    // Measured angles from gyroscope
    float phi_g = phi + (gx + gy * sinf(phi) * tanf(theta) + gz * cosf(phi) * tanf(theta)) * dt;
    float theta_g = theta + (gy * cosf(phi) - gz * sinf(phi)) * dt;
    float psi_g = psi + (gy * sinf(phi) / cosf(theta) + gz * cosf(phi) / cosf(theta)) * dt;

    // Estimated angles (accelerometer and gyroscope with complementary filter)
    phi = (1.0f - alpha) * phi_g + alpha * phi_a;
    theta = (1.0f - alpha) * theta_g + alpha * theta_a;
    psi = psi_g;

    // Angular velocities estimation (gyroscope)
    wx = gx;
    wy = gy;
    wz = gz;

    // Auxiliary variables for logging Euler angles (CFClient uses degrees and not radians)
    log_phi = phi * 180.0f / pi;
    log_theta = -theta * 180.0f / pi;
    log_psi = psi * 180.0f / pi;
}

// Compute desired torques
void attitudeController()
{
    // Ganhos do controlador (ajuste conforme necessário)
    const float kp = 240.28f;
    const float kd = 26.67f;

    const float kp_psi = kp/4;
    const float kd_psi = kd/2;

    float phi_dot_r = 0.0f; // Velocidade angular de referência
    float phi_dot = wx;                 // Velocidade angular atual
    float phi_ddot_r = kp * (phi_r - phi) + kd * (phi_dot_r - phi_dot); // Aceleração angular de referência

    float theta_dot_r = 0.0f; // Velocidade angular de referência
    float theta_dot = wy;                 // Velocidade angular atual
    float theta_ddot_r = kp * (theta_r - theta) + kd * (theta_dot_r - theta_dot); // Aceleração angular de referência

    float psi_dot_r = 0.0f; // Velocidade angular de referência
    float psi_dot = wz;                 // Velocidade angular atual
    float psi_ddot_r = kp_psi * (psi_r - psi) + kd_psi * (psi_dot_r - psi_dot); // Aceleração angular de referência

    // 4) Torque comandado no eixo y (pitch)
    tx = Ixx * phi_ddot_r;
    ty = Iyy * theta_ddot_r;
    tz = Izz * psi_ddot_r;

}

// Estimate vertical position/velocity from range sensor
void verticalEstimator()
{
    // Estimator parameters
    static const float wc = 10.0f; // Cut-off frequency [rad/s]
    static const float zeta = 0.707f; // Damping ratio
    static const float l1 = 2 * zeta * wc;
    static const float l2 = wc * wc;

    // Measured distance from range sensor
    float z_m = d * cosf(phi) * cosf(theta); // Compensate for quadcopter inclination

    // Prediction step (model)
    z = z + vz * dt;
    vz = vz + (-g + (ft / m)) * dt;

    // Correction step (measurement)
    z = z + l1*dt * (z_m - z);
    vz = vz + l2*dt * (z_m - z);
}

// Compute desired thrust force
void verticalController()
{
    // Controller parameters (settling time of 2.0s and overshoot of 0,05%)
    static const float kp = 7.0f;
    static const float ki = 0.90f;  // comece 0.0; depois suba devagar (ex.: 0.5f)
    static const float kd = 4.40f;

    // Compute vertical position error
    float z_e = z_r - z;

    static float z_e_integral;
    z_e_integral += z_e*dt;

    // Compute desired thrust force
    ft = m * (g + (kp * z_e + ki * z_e_integral - kd * vz));
}


// Estimate horizontal position/velocity from optical flow sensor
void horizontalEstimator()
{
    // Estimator parameters
    const float gamma = 42.0f * 3.14f / 180.0f; // Camera sensor angle resolution [rad]
    const float W = 35.0f; // Camera sensor width [px]
    static const float sigma = (2* tanf(gamma * 0.5f))/(W * dt); // Tangente de gamma/2
    static const float wc = 50.0f; // Cut-off frequency [rad/s]
    static const float l = wc; // Estimator gain

    // Measured velocities from optical flow
    float vx_m = (sigma * px + wy) * z;
    float vy_m = (sigma * py - wx) * z;

    // Prediction step (model)
    x += vx * dt;
    y += vy * dt;
    vx += g * theta * dt;  // Assume constant velocity model
    vy -= g * phi * dt;  // Assume constant velocity model

    // Correction step (measurement)
    vx = vx + l*dt * (vx_m - vx);
    vy = vy + l*dt * (vy_m - vy);
}

// Main application task
void appMain(void *param)
{
    // Infinite loop (runs at 200Hz)
    while (true)
    {
        reference();                  // Read reference setpoints (from Crazyflie Client)
        sensors();                    // Read raw sensor measurements
        attitudeEstimator();          // Estimate orientation (roll/pitch/yaw) from IMU sensor
        horizontalEstimator();        // Estimate horizontal positions/velocities from optical flow sensor
        verticalEstimator();          // Estimate vertical position/velocity from range sensor
        verticalController();         // Compute desired thrust force
        attitudeController();         // Compute desired roll/pitch/yaw torques
        mixer();                      // Convert desired force/torques into motor PWM
        actuators();                  // Send commands to motors
        vTaskDelay(pdMS_TO_TICKS(5)); // Loop delay (5 ms)
    }
}