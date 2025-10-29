#include "ekf.h"
#include <Eigen/Dense>
#include <cmath>



extern const std::map<float, float> O5500X_data;
extern const std::map<float, float> M685W_data;
extern const std::map<std::string, std::map<float, float>> motor_data;

// constants
const float pi = 3.14159268;
const float a = 343.0;                // (m/s) speed of sound
const float rho = 1.225;              // average air density
const float r = 0.0396;              // (m)
const float height_full = 3.0259;      // (m) height of rocket Full Stage
const float height_sustainer = 1.5021; // (m) height of rocket Sustainer
const float mass_full = 10.6;        // (kg) Sustainer + Booster
const float mass_sustainer = 4.68;   // (kg) Sustainer
const float gravity_ms2 = 9.81;           // (m/s^2) accel due to gravity

/** 
 * This file contains all the aerodynamic constants
 * aero_data - the aerodynamic coefficients for drag calculations
 * thurst curves for both motors
 * 
 */
typedef struct
{
    float mach;
    float alpha;
    float CA_power_on;
    float CN;
    float CP;
} aero_coeff_t;

// stores the aerodynamic coefficients for the corresponding Mach number
const aero_coeff_t aero_data[] = {
    {0.04, 0, 1.000001789, 25.80486518, 123.856999},
    {0.08, 0, 0.899149955, 25.80486518, 123.856999},
    {0.12, 0, 0.848262793, 25.80486518, 123.856999},
    {0.16, 0, 0.815490497, 25.80486518, 123.856999},
    {0.2, 0, 0.791977907, 25.80486518, 123.856999},
    {0.24, 0, 0.77407759, 25.80486518, 123.856999},
    {0.28, 0, 0.763046286, 25.80486518, 123.856999},
    {0.32, 0, 0.758319846, 25.80486518, 123.856999},
    {0.36, 0, 0.760511343, 25.80486518, 123.856999},
    {0.4, 0, 0.763737136, 25.80486518, 123.856999},
    {0.44, 0, 0.767325178, 25.80486518, 123.856999},
    {0.48, 0, 0.771334851, 25.80486518, 123.856999},
    {0.52, 0, 0.775843406, 25.80486518, 123.856999},
    {0.56, 0, 0.780953377, 25.80486518, 123.856999},
    {0.6, 0, 0.785771581, 25.80486518, 123.856999},
    {0.64, 0, 0.793730593, 25.80486518, 123.856999},
    {0.68, 0, 0.80285965, 25.80486518, 123.856999},
    {0.72, 0, 0.807910063, 25.80486518, 123.856999},
    {0.76, 0, 0.807403195, 25.80486518, 123.856999},
    {0.8, 0, 0.806889479, 25.80486518, 123.856999},
    {0.84, 0, 0.832707826, 25.80486518, 123.856999},
    {0.88, 0, 0.858519521, 25.80486518, 123.856999},
    {0.92, 0, 0.895125486, 25.492166, 124.3408619},
    {0.96, 0, 0.923744595, 24.86676763, 125.3085876},
    {1, 0, 0.941214699, 24.24136926, 126.2763132},
};

// Moonburner motor thrust curve (Sustainer) // Updated to new motor data
const std::map<float, float> M685W_data = {
    {0.13, 1368.376},
    {0.249, 1361.395},
    {0.308, 1380.012},
    {0.403, 1359.068},
    {0.675, 1184.53},
    {1.018, 1072.826},
    {1.456, 996.029},
    {1.977, 958.794},
    {2.995, 914.578},
    {3.99, 856.399},
    {4.985, 781.929},
    {5.494, 730.732},
    {5.991, 679.534},
    {7.258, 542.231},
    {7.862, 463.107},
    {8.015, 456.125},
    {8.998, 330.458},
    {9.993, 207.118},
    {10.514, 137.303},
    {11.496, 34.908},
    {11.994, 0.0}
};

/**
    {0.13, 1368.376},
    {0.249, 1361.395},
    {0.308, 1380.012},
    {0.403, 1359.068},
    {0.675, 1184.53},
    {1.018, 1072.826},
    {1.456, 996.029},
    {1.977, 958.794},
    {2.995, 914.578},
    {3.99, 856.399},
    {4.985, 781.929},
    {5.494, 730.732},
    {5.991, 679.534},
    {7.258, 542.231},
    {7.862, 463.107},
    {8.015, 456.125},
    {8.998, 330.458},
    {9.993, 207.118},
    {10.514, 137.303},
    {11.496, 34.908},
    {11.994, 0.0}

*/

//

// O5500X motor thrust curve (Booster) // Updated to new motor data
const std::map<float, float> O5500X_data = {
    {0.044, 7112.245},
    {0.063, 6734.694},
    {0.078, 6897.959},
    {0.094, 6612.245},
    {0.109, 6765.306},
    {0.125, 6540.816},
    {0.147, 6581.633},
    {0.194, 6520.408},
    {0.350, 6795.918},
    {0.428, 7091.837},
    {0.563, 7285.714},
    {0.694, 7408.163},
    {0.988, 7581.633},
    {1.266, 7622.449},
    {1.491, 7724.490},
    {1.581, 7653.061},
    {1.641, 7540.816},
    {1.684, 7500.000},
    {1.716, 7336.735},
    {1.784, 7224.490},
    {1.938, 6785.714},
    {2.138, 6326.531},
    {2.491, 5897.959},
    {2.600, 5704.082},
    {2.919, 3540.816},
    {3.022, 3408.163},
    {3.138, 2887.755},
    {3.300, 2234.694},
    {3.388, 1673.469},
    {3.441, 1489.796},
    {3.544, 1418.367},
    {3.609, 1295.918},
    {3.688, 816.327},
    {3.778, 653.061},
    {3.819, 581.633},
    {3.853, 489.796},
    {3.897, 285.714},
    {3.981, 20.408},
    {3.997, 0.000}
};

//Old values: 
// 
//     {0.044, 7112.245},
//     {0.063, 6734.694},
//     {0.078, 6897.959},
//     {0.094, 6612.245},
//     {0.109, 6765.306},
//     {0.125, 6540.816},
//     {0.147, 6581.633},
//     {0.194, 6520.408},
//     {0.350, 6795.918},
//     {0.428, 7091.837},
//     {0.563, 7285.714},
//     {0.694, 7408.163},
//     {0.988, 7581.633},
//     {1.266, 7622.449},
//     {1.491, 7724.490},
//     {1.581, 7653.061},
//     {1.641, 7540.816},
//     {1.684, 7500.000},
//     {1.716, 7336.735},
//     {1.784, 7224.490},
//     {1.938, 6785.714},
//     {2.138, 6326.531},
//     {2.491, 5897.959},
//     {2.600, 5704.082},
//     {2.919, 3540.816},
//     {3.022, 3408.163},
//     {3.138, 2887.755},
//     {3.300, 2234.694},
//     {3.388, 1673.469},
//     {3.441, 1489.796},
//     {3.544, 1418.367},
//     {3.609, 1295.918},
//     {3.688, 816.327},
//     {3.778, 653.061},
//     {3.819, 581.633},
//     {3.853, 489.796},
//     {3.897, 285.714},
//     {3.981, 20.408},
//     {3.997, 0.000}


// constant variable that contains the booster and sustainer motors
const std::map<std::string, std::map<float, float>> motor_data = {
    {"Booster", O5500X_data},
    {"Sustainer", M685W_data}
};

/**
 * @brief linearly interpolates the a value based on the lower and upper bound, similar to lerp_() in PySim
 */
inline float linearInterpolation(float x0, float y0, float x1, float y1, float x)
{
    return y0 + ((x - x0) * (y1 - y0) / (x1 - x0));
}

/**
 * @brief Converts a vector in the body frame to the global frame
 *
 * @param angles Roll, pitch, yaw angles
 * @param body_vec Vector for rotation in the body frame
 * @return Eigen::Matrix<float, 3, 1> Rotated vector in the global frame
 */
template <typename Angles>
void BodyToGlobal(Angles& angles_rad, Eigen::Matrix<float, 3, 1> &body_vec)
{
    Eigen::Matrix3f roll, pitch, yaw;
    roll << cos(angles_rad.roll), -sin(angles_rad.roll), 0.,
        sin(angles_rad.roll),  cos(angles_rad.roll), 0.,
        0.,                    0.,                   1.;

pitch << cos(angles_rad.pitch), 0., sin(angles_rad.pitch),
         0.,                    1., 0.,
        -sin(angles_rad.pitch), 0., cos(angles_rad.pitch);

yaw << 1., 0., 0.,
        0., cos(angles_rad.yaw), -sin(angles_rad.yaw),
        0., sin(angles_rad.yaw),  cos(angles_rad.yaw);

    Eigen::Matrix3f rotation_matrix = yaw * pitch * roll;
    Eigen::Vector3f temp = rotation_matrix * body_vec;

    // Convert from Z-up convention to X-up convention
    // Eigen::Vector3f corrected;
    // corrected(0) = temp(2);  // Z → X
    // corrected(1) = temp(1);  // X → Y 
    // corrected(2) = temp(0);  // Y → Z 

    // body_vec = corrected;
}

/**
 * @brief Converts a vector in the global frame to the body frame
 *
 * @param angles Roll, pitch, yaw angles
 * @param global_vec Vector for rotation in the global frame
 *
 * @return Eigen::Matrix<float, 3, 1> Rotated vector in the body frame
 */
template <typename Angles>
void GlobalToBody(Angles& angles_rad, Eigen::Matrix<float, 3, 1> &global_vec)
{
    Eigen::Matrix3f roll, pitch, yaw;
    
    roll << cos(angles_rad.roll), -sin(angles_rad.roll), 0.,
        sin(angles_rad.roll),  cos(angles_rad.roll), 0.,
        0.,                    0.,                   1.;

// Pitch about Y (tilt forward/back)
pitch << cos(angles_rad.pitch), 0., sin(angles_rad.pitch),
         0.,                    1., 0.,
        -sin(angles_rad.pitch), 0., cos(angles_rad.pitch);

// Yaw about X (turn around up axis)
yaw << 1., 0., 0.,
        0., cos(angles_rad.yaw), -sin(angles_rad.yaw),
        0., sin(angles_rad.yaw),  cos(angles_rad.yaw);

    Eigen::Matrix3f rotation_matrix = yaw * pitch * roll;
    Eigen::Vector3f temp = rotation_matrix.transpose() * global_vec;
    Eigen::Matrix3f R_zup_to_xup;
    R_zup_to_xup << 1, 0,0 ,
                    0, 1, 0,
                    0, 0, 1;

    global_vec = (R_zup_to_xup * rotation_matrix).transpose() * global_vec;

}

EKF::EKF() : KalmanFilter()
{
    state = KalmanData();
}



/**
 * THIS IS A PLACEHOLDER FUNCTION SO WE CAN ABSTRACT FROM `kalman_filter.h`
 */
void EKF::priori() {};

/**
 * @brief Sets altitude by averaging 30 barometer measurements taken 100 ms
 * apart
 *
 * The following for loop takes a series of barometer measurements on start
 * up and takes the average of them in order to initialize the kalman filter
 * to the correct initial barometric altitude. This is done so that the
 * kalman filter takes minimal time to converge to an accurate state
 * estimate. This process is significantly faster than allowing the state as
 * letting the filter to converge to the correct state can take up to 3 min.
 * This specific process was used because the barometric altitude will
 * letting the filter to converge to the correct state can take up to 3 min.
 * This specific process was used because the barometric altitude will
 * change depending on the weather and thus, the initial state estimate
 * cannot be hard coded. A GPS altitude may be used instead but due to GPS
 * losses during high speed/high altitude flight, it is inadvisable with the
 * current hardware to use this as a solution. Reference frames should also
 * be kept consistent (do not mix GPS altitude and barometric).
 *
 */
void EKF::initialize(RocketSystems *args)
{
    Orientation orientation = args->rocket_data.orientation.getRecentUnsync();
    float sum = 0;

    for (int i = 0; i < 30; i++)
    {
        Barometer barometer = args->rocket_data.barometer.getRecent();
        LowGData initial_accelerometer = args->rocket_data.low_g.getRecent();
        Acceleration accelerations = {
            .ax = initial_accelerometer.ax,
            .ay = initial_accelerometer.ay,
            .az = initial_accelerometer.az};
        sum += barometer.altitude;

        // init_accel(0, 0) += -accelerations.ax;
        // init_accel(1, 0) += accelerations.ay;
        // init_accel(2, 0) += accelerations.az;
        //THREAD_SLEEP(100);
    }

    // init_accel(0, 0) /= 30;
    // init_accel(1, 0) /= 30;
    // init_accel(2, 0) /= 30;

    euler_t euler = orientation.getEuler();
    // euler.yaw = -euler.yaw;

    // set x_k
    x_k.setZero();
    x_k(0, 0) = sum / 30;
    x_k(3, 0) = 0;
    x_k(6, 0) = 0;

    F_mat.setZero(); // Initialize with zeros

    // Initialize Q from filterpy
    Q(0, 0) = pow(s_dt, 5) / 20;
    Q(0, 1) = pow(s_dt, 4) / 8;
    Q(0, 2) = pow(s_dt, 3) / 6;
    Q(1, 1) = pow(s_dt, 3) / 3; // fxed
    Q(1, 2) = pow(s_dt, 2) / 2;
    Q(2, 2) = s_dt;
    Q(1, 0) = Q(0, 1);
    Q(2, 0) = Q(0, 2);
    Q(2, 1) = Q(1, 2);

    Q(3, 3) = pow(s_dt, 5) / 20;
    Q(3, 4) = pow(s_dt, 4) / 8;
    Q(3, 5) = pow(s_dt, 3) / 6;
    Q(4, 4) = pow(s_dt, 3) / 3; // fixed
    Q(4, 5) = pow(s_dt, 2) / 2;
    Q(5, 5) = s_dt;
    Q(4, 3) = Q(3, 4);
    Q(5, 3) = Q(3, 5);
    Q(5, 4) = Q(4, 5);

    Q(6, 6) = pow(s_dt, 5) / 20;
    Q(6, 7) = pow(s_dt, 4) / 8;
    Q(6, 8) = pow(s_dt, 3) / 6;
    Q(7, 7) = pow(s_dt, 3) / 3; // fixed
    Q(7, 8) = pow(s_dt, 2) / 2;
    Q(8, 8) = s_dt;
    Q(7, 6) = Q(6, 7);
    Q(8, 6) = Q(6, 8);
    Q(8, 7) = Q(7, 8);

    // set H
    H.setZero();
    H(0, 0) = 1;
    H(1, 2) = 1;
    H(2, 5) = 1;
    H(3, 8) = 1;

    Q = Q * spectral_density_;

    // Wind vector
    // Wind(0, 0) = 0.0; // wind in x direction
    // Wind(1, 0) = 0.0; // wind in y direction
    // Wind(2, 0) = 0.0; // wind in z direction

    P_k.setZero();
    P_k.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 1e-2f; // x block (pos,vel,acc)
    P_k.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 1e-2f; // y block
    P_k.block<3, 3>(6, 6) = Eigen::Matrix3f::Identity() * 1e-2f; // z block

    // set
    R(0, 0) = 10.0;
    R(1, 1) = 1.9;
    R(2, 2) = 1.9;
    R(3, 3) = 1.9;

    // set B (don't care about what's in B since we have no control input)
    B(2, 0) = -1;
}

/**
 * @brief Estimates current state of the rocket without current sensor data
 *
 * The priori step of the Kalman filter is used to estimate the current state
 * of the rocket without knowledge of the current sensor data. In other words,
 * it extrapolates the state at time n+1 based on the state at time n.
 */

void EKF::priori(float dt, Orientation &orientation, FSMState fsm)
{
    Eigen::Matrix<float, 9, 1> xdot = Eigen::Matrix<float, 9, 1>::Zero();

    // angular states from sensors
    Velocity omega_rps = orientation.getAngularVelocity(); // rads per sec

    euler_t angles_rad = orientation.getEuler();

    // ignore effects of gravity when on pad
    Eigen::Matrix<float, 3, 1> g_global = Eigen::Matrix<float, 3, 1>::Zero();
    if ((fsm > FSMState::STATE_IDLE))
    {
        g_global(0, 0) = -gravity_ms2;
    }
    else
    {
        g_global(0, 0) = 0;
    }

    // mass and height init
    float curr_mass_kg = mass_sustainer;
    float curr_height_m = height_sustainer;

    if (fsm < FSMState::STATE_BURNOUT)
    {
        curr_mass_kg = mass_full;
        curr_height_m = height_full;
    }

    // Mach number // Subtracting wind from velocity
    // float vel_mag_squared_ms = ((x_k(1, 0) - 1.2 * Wind(0, 0)) * (x_k(1, 0) - 1.2 * Wind(0, 0))) + x_k(4, 0) * x_k(4, 0) + x_k(7, 0) * x_k(7, 0);
    float vel_mag_squared_ms = ((x_k(1, 0) ) * (x_k(1, 0) )) + x_k(4, 0) * x_k(4, 0) + x_k(7, 0) * x_k(7, 0);

    float vel_magnitude_ms = pow(vel_mag_squared_ms, 0.5);

    float mach = vel_magnitude_ms / a;

    // approximating C_a (aerodynamic coeff.)
    int index = std::round(mach / 0.04);

    index = std::clamp(index, 0, (int)AERO_DATA_SIZE - 1);

    Ca = aero_data[index].CA_power_on;

    // aerodynamic force
    // Body frame
    float Fax = 0; // instead of mag square --> mag * vel_x
    if ((fsm > FSMState::STATE_IDLE))
    {
        Fax = -0.5 * rho * (vel_magnitude_ms) * float(Ca) * (pi * r * r) * x_k(1, 0);
    }
    float Fay = 0; // assuming no aerodynamic effects
    float Faz = 0; // assuming no aerodynamic effects

    // acceleration due to gravity
    float gx = g_global(0, 0);
    float gy = g_global(1, 0);
    float gz = g_global(2, 0);

    // thurst force, body frame
    Eigen::Matrix<float, 3, 1> Ft_body;
    EKF::getThrust(stage_timestamp, angles_rad, fsm, Ft_body);

    // body frame
    float Ftx = Ft_body(0, 0);
    float Fty = Ft_body(1, 0);
    float Ftz = Ft_body(2, 0);

    Eigen::Matrix<float, 3, 1> velocities_body;
    velocities_body << x_k(1, 0), x_k(4, 0), x_k(7, 0);

    GlobalToBody(angles_rad, velocities_body);
    float vx_body = velocities_body(0, 0);
    float vy_body = velocities_body(1, 0);
    float vz_body = velocities_body(2, 0);

    Eigen::Matrix<float, 3, 1> v_dot; // we compute everything in the body frame for accelerations, and then convert those accelerations to global frame
    v_dot << ((Fax + Ftx) / curr_mass_kg - (omega_rps.vy * vz_body - omega_rps.vz * vy_body) + x_k(2, 0)) / 2,
        ((Fay + Fty) / curr_mass_kg - (omega_rps.vz * vx_body - omega_rps.vx * vz_body) + x_k(5, 0) / 2),
        ((Faz + Ftz) / curr_mass_kg - (omega_rps.vx * vy_body - omega_rps.vy * vx_body) + x_k(8, 0) / 2);

    BodyToGlobal(angles_rad, v_dot);

    xdot << x_k(1, 0), v_dot(0, 0) + gx,
        0.0,

        x_k(4, 0), v_dot(1, 0) + gy,
        0.0,

        x_k(7, 0), v_dot(2, 0) + gz,
        0.0;

    // priori step
    x_priori = (xdot * dt) + x_k;

    float coeff = 0;
    if ((fsm > FSMState::STATE_IDLE))
    {
        coeff = -pi * Ca * (r * r) * rho / curr_mass_kg;
    }

    setF(dt, omega_rps.vx, omega_rps.vy, omega_rps.vz, coeff, vx_body, vy_body, vz_body);

    P_priori = (F_mat * P_k * F_mat.transpose()) + Q;
}


/**
 * @brief Update Kalman Gain and state estimate with current sensor data
 *
 * After receiving new sensor data, the Kalman filter updates the state estimate
 * and Kalman gain. The Kalman gain can be considered as a measure of how uncertain
 * the new sensor data is. After updating the gain, the state estimate is updated.
 *
 */
void EKF::update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState FSM_state)
{
    // if on pad -> take last 10 barometer measurements for init state
    if (FSM_state == FSMState::STATE_IDLE)
    {
        float sum = 0;
        float data[10];
        alt_buffer.readSlice(data, 0, 10);
        for (float i : data)
        {
            sum += i;
        }
        P_k(4, 4) = 1e-6f; // variance for vel_y
        P_k(7, 7) = 1e-6f; // variance for vel_z
        KalmanState kalman_state = (KalmanState){sum / 10.0f, 0, 0, 0, 0, 0, 0, 0, 0};
        setState(kalman_state);
    }
    // ignore alitiude measurements after apogee
    else if (FSM_state == FSMState::STATE_APOGEE)
    {
        H(1, 2) = 0;
    }

    // Kalman Gain
    Eigen::Matrix<float, 4, 4> S_k = Eigen::Matrix<float, 4, 4>::Zero();
    S_k = (((H * P_priori * H.transpose()) + R)).inverse();
    Eigen::Matrix<float, 9, 9> identity = Eigen::Matrix<float, 9, 9>::Identity();
    K = (P_priori * H.transpose()) * S_k;

    // Sensor Measurements
    Eigen::Matrix<float, 3, 1> sensor_accel_global_g = Eigen::Matrix<float, 3, 1>(Eigen::Matrix<float, 3, 1>::Zero());

    // accouting for sensor bias and coordinate frame transforms
    (sensor_accel_global_g)(0, 0) = -acceleration.ax + 0.045;
    (sensor_accel_global_g)(1, 0) = acceleration.ay - 0.065;
    (sensor_accel_global_g)(2, 0) = acceleration.az - 0.06;

    euler_t angles_rad = orientation.getEuler();
    // angles_rad.yaw = -angles_rad.yaw; // coordinate frame match

    BodyToGlobal(angles_rad, sensor_accel_global_g);

    float g_ms2;
    if ((FSM_state > FSMState::STATE_IDLE))
    {
        g_ms2 = gravity_ms2;
    }
    else
    {
        g_ms2 = 0;
    }

    // acceloremeter reports values in g's and measures specific force
    y_k(1, 0) = ((sensor_accel_global_g)(0)) * g_ms2;
    y_k(2, 0) = ((sensor_accel_global_g)(1)) * g_ms2;
    y_k(3, 0) = ((sensor_accel_global_g)(2)) * g_ms2;

    y_k(0, 0) = barometer.altitude; // meters

    // # Posteriori Update
    x_k = x_priori + K * (y_k - (H * x_priori));
    P_k = (identity - K * H) * P_priori;

    kalman_state.state_est_pos_x = x_k(0, 0);
    kalman_state.state_est_vel_x = x_k(1, 0);
    kalman_state.state_est_accel_x = x_k(2, 0);
    kalman_state.state_est_pos_y = x_k(3, 0);
    kalman_state.state_est_vel_y = x_k(4, 0);
    kalman_state.state_est_accel_y = x_k(5, 0);
    kalman_state.state_est_pos_z = x_k(6, 0);
    kalman_state.state_est_vel_z = x_k(7, 0);
    kalman_state.state_est_accel_z = x_k(8, 0);

    state.position = (Position){kalman_state.state_est_pos_x, kalman_state.state_est_pos_y, kalman_state.state_est_pos_z};
    state.velocity = (Velocity){kalman_state.state_est_vel_x, kalman_state.state_est_vel_y, kalman_state.state_est_vel_z};
    state.acceleration = (Acceleration){kalman_state.state_est_accel_x, kalman_state.state_est_accel_y, kalman_state.state_est_accel_z};

    // if (FSM_state > FSMState::STATE_IDLE)
    // {
    //     current_vel += (s_dt)*y_k(1, 0);
    //     Eigen::Matrix<float, 9, 1> measured_v = Eigen::Matrix<float, 9, 1>::Zero();
    //     measured_v(0, 0) = current_vel;
        // measured_v(0,0) = y_k(1) + (dt/2)*y_k(2);
        // Eigen::Matrix<float, 9, 1> err = Eigen::Matrix<float, 9, 1>::Zero();
        // err(0, 0) = measured_v(0, 0) - x_k(1, 0);
        // Wind = Wind_alpha * Wind + (1 - Wind_alpha) * err;
        // if (Wind.norm() > 15)
        // {
        //     Wind(0, 0) = 15.0;
        //     Wind(1, 0) = 0.0;
        //     Wind(2, 0) = 0.0;
        // }
    // }
}

/**
 * @brief Run Kalman filter calculations as long as FSM has passed IDLE
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 * @param sd Spectral density of the noise
 * @param &barometer Data of the current barometer
 * @param acceleration Current acceleration
 * @param &orientation Current orientation
 * @param current_state Current FSM_state
 */
void EKF::tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState FSM_state)
{
    if (FSM_state >= FSMState::STATE_IDLE) //
    {
        if (FSM_state != last_fsm)
        {
            stage_timestamp = 0;
            last_fsm = FSM_state;
        }
        stage_timestamp += dt;
        // setF(dt, orientation.roll, orientation.pitch, orientation.yaw);
        setQ(dt, sd);
        priori(dt, orientation, FSM_state);
        update(barometer, acceleration, orientation, FSM_state);
    }
}

/**
 * @brief Getter for state X
 *
 * @return the current state, see sensor_data.h for KalmanData
 */
KalmanData EKF::getState()
{
    return state;
}

/**
 * @brief Sets state vector x
 *
 * @param state Wanted state vector
 */
void EKF::setState(KalmanState state)
{
    this->state.position.px = state.state_est_pos_x;
    this->state.position.py = state.state_est_pos_y;
    this->state.position.pz = state.state_est_pos_z;
    this->state.acceleration.ax = state.state_est_accel_x;
    this->state.acceleration.ay = state.state_est_accel_y;
    this->state.acceleration.az = state.state_est_accel_z;
    this->state.velocity.vx = state.state_est_vel_x;
    this->state.velocity.vy = state.state_est_vel_y;
    this->state.velocity.vz = state.state_est_vel_z;
}

/**
 * @brief Sets the Q matrix given time step and spectral density.
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 * @param sd Spectral density of the noise
 *
 * The Q matrix is the covariance matrix for the process noise and is
 * updated based on the time taken per cycle of the Kalman Filter Thread.
 */
void EKF::
    setQ(float dt, float sd)
{
    Q(0, 0) = pow(dt, 5) / 20;
    Q(0, 1) = pow(dt, 4) / 8;
    Q(0, 2) = pow(dt, 3) / 6;
    Q(1, 1) = pow(dt, 3) / 3;
    Q(1, 2) = pow(dt, 2) / 2;
    Q(2, 2) = dt;
    Q(1, 0) = Q(0, 1);
    Q(2, 0) = Q(0, 2);
    Q(2, 1) = Q(1, 2);
    Q(3, 3) = pow(dt, 5) / 20;
    Q(3, 4) = pow(dt, 4) / 8;
    Q(3, 5) = pow(dt, 3) / 6;
    Q(4, 4) = pow(dt, 3) / 3;
    Q(4, 5) = pow(dt, 2) / 2;
    Q(5, 5) = dt;
    Q(4, 3) = Q(3, 4);
    Q(5, 3) = Q(3, 5);
    Q(5, 4) = Q(4, 5);

    Q(6, 6) = pow(dt, 5) / 20;
    Q(6, 7) = pow(dt, 4) / 8;
    Q(6, 8) = pow(dt, 3) / 6;
    Q(7, 7) = pow(dt, 3) / 3;
    Q(7, 8) = pow(dt, 2) / 2;
    Q(8, 8) = dt;
    Q(7, 6) = Q(6, 7);
    Q(8, 6) = Q(6, 8);
    Q(8, 7) = Q(7, 8);

    Q *= sd;
}

/**
 * @brief Sets the F matrix given time step.
 *
 * @param dt Time step calculated by the Kalman Filter Thread
 *
 * The F matrix is the state transition matrix and is defined
 * by how the states change over time and also depends on the
 * current state of the rocket.
 */
void EKF::setF(float dt, float w_x, float w_y, float w_z, float coeff, float v_x, float v_y, float v_z)

{
    F_mat.setIdentity(); // start from identity

    // For x
    // F_mat(0, 1) = dt;
    // F_mat(0, 2) = 0.5f * dt * dt;
    // F_mat(1, 2) = dt;

    // // For y
    // F_mat(3, 4) = dt;
    // F_mat(3, 5) = 0.5f * dt * dt;
    // F_mat(4, 5) = dt;

    // // For z
    // F_mat(6, 7) = dt;
    // F_mat(6, 8) = 0.5f * dt * dt;
    // F_mat(7, 8) = dt;
    F_mat.setZero();

    F_mat(0, 1) = 1.0f;

    F_mat(1, 1) = coeff * v_x;
    F_mat(1, 2) = 0.5f;
    F_mat(1, 4) = 0.5f * (coeff * v_y + w_z);
    F_mat(1, 7) = 0.5f * (coeff * v_z - w_y);

    F_mat(3, 4) = 1.0f;

    F_mat(4, 1) = -0.5f * w_z;
    F_mat(4, 5) = 0.5f;
    F_mat(4, 7) = 0.5f * w_x;

    F_mat(6, 7) = 1.0f;

    F_mat(7, 1) = 0.5f * w_y;
    F_mat(7, 4) = -0.5f * w_x;
    F_mat(7, 8) = 0.5f;
}

/**
 * @brief Returns the approximate thrust force from the motor given the thurst curve
 *
 * @param timestamp Time since most recent ignition
 * @param angles Current orientation of the rocket
 * @param FSM_state Current FSM state
 *
 * @return Thrust force in the body frame
 *
 * The thrust force is calculated by interpolating the thrust curve data which is stored in an ordered map (see top of file).
 * The thrust curve data is different for the booster and sustainer stages, so the function checks the FSM state to determine
 * which thrust curve to use. The time since ignition is also important to consider so that is reset once we reach a new stage.
 * The thrust force is then rotated into the body frame using the BodyToGlobal function.
 */
void EKF::getThrust(float timestamp, const euler_t &angles, FSMState FSM_state, Eigen::Vector3f &thrust_out)
{
    // Pick which motor thrust curve to use
    const std::map<float, float> *thrust_curve = nullptr;

    if (FSM_state == STATE_FIRST_BOOST){
        thrust_curve = &motor_data.at("Booster"); // Booster
    }
    else if (FSM_state == STATE_SECOND_BOOST)
        thrust_curve = &motor_data.at("Sustainer"); // Sustainer
    else
    {
        thrust_out.setZero();
        return; // No thrust before ignition
    }

    // Handle case where timestamp is before or after available data
    if (timestamp <= thrust_curve->begin()->first)
    {
        thrust_out = Eigen::Vector3f(thrust_curve->begin()->second, 0.f, 0.f);
    }
    else if (timestamp >= thrust_curve->rbegin()->first)
    {
        thrust_out.setZero(); // assume motor burned out after curve ends
    }
    else
    {
        // Find interpolation interval
        auto it_upper = thrust_curve->lower_bound(timestamp);
        auto it_lower = std::prev(it_upper);

        float x0 = it_lower->first;
        float y0 = it_lower->second;
        float x1 = it_upper->first;
        float y1 = it_upper->second;

        float interpolated_thrust = linearInterpolation(x0, y0, x1, y1, timestamp);
        thrust_out = Eigen::Vector3f(interpolated_thrust, 0.f, 0.f);
    }

    // Rotate from body to global
    // BodyToGlobal(angles, thrust_out);
}

EKF ekf;