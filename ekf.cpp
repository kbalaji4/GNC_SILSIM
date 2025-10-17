#include "ekf.h"
#include <Eigen/Dense>
#include <cmath>

const float GRAVITY = 9.80665f;

// Aerodynamic data initialization
const aero_coeff_t EKF::aero_data[25] = {
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
    {1, 0, 0.941214699, 24.24136926, 126.2763132}
};

// Motor thrust curves initialization
const std::map<float, float> EKF::M685W_data = {
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

const std::map<float, float> EKF::O5500X_data = {
    {0.044, 7112.245},
    {0.063, 6734.694},
    {0.078, 6897.959},
    {0.094, 6612.245},
    {0.109, 6765.306},
    {0.125, 6540.816},
    {0.147, 6581.633},
    {0.194, 6520.408},
    {0.35, 6795.918},
    {0.428, 7091.837},
    {0.563, 7285.714},
    {0.694, 7408.163},
    {0.988, 7581.633},
    {1.266, 7622.449},
    {1.491, 7724.49},
    {1.581, 7653.061},
    {1.641, 7540.816},
    {1.684, 7500.0},
    {1.716, 7336.735},
    {1.784, 7224.49},
    {1.938, 6785.714},
    {2.138, 6326.531},
    {2.491, 5897.959},
    {2.6, 5704.082},
    {2.919, 3540.816},
    {3.022, 3408.163},
    {3.138, 2887.755},
    {3.3, 2234.694},
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
    {3.997, 0.0}
};

const std::map<std::string, std::map<float, float>> EKF::motor_data = {
    {"Booster", O5500X_data},
    {"Sustainer", M685W_data}
};

EKF::EKF() : KalmanFilter()
{
    state = KalmanData();
}

Eigen::Matrix3f EKF::eulerToRotation(float yaw, float pitch, float roll)
{
    yaw *= M_PI / 180.0f;
    pitch *= M_PI / 180.0f;
    roll *= M_PI / 180.0f;

    Eigen::Matrix3f Rz;
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw),  cos(yaw), 0,
          0,          0,        1;

    Eigen::Matrix3f Ry;
    Ry << cos(pitch), 0, sin(pitch),
          0,          1, 0,
         -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3f Rx;
    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);

    return Rz * Ry * Rx;
}

void EKF::initialize(RocketSystems *args)
{
    Orientation orientation = args->rocket_data.orientation.getRecentUnsync();

    // Initial altitude average
    float alt_sum = 0;
    for (int i = 0; i < 20; i++) {
        Barometer b = args->rocket_data.barometer.getRecent();
        alt_sum += b.altitude;
        // THREAD_SLEEP(50);
    }
    float alt0 = alt_sum / 20.0f;

    x_k.setZero();
    x_k(0) = alt0;  // x = up/down
    P_k.setIdentity();
    P_k *= 5.0f;    // initial covariance

    // Q: process noise
    float sigma_a = 1.0f;  // m/s²
    Q.setZero();
    for (int i = 0; i < 3; ++i) {
        Q(i, i) = 0.25 * pow(s_dt,4) * sigma_a*sigma_a;
        Q(i+3, i+3) = pow(s_dt,2) * sigma_a*sigma_a;
        Q(i+6, i+6) = sigma_a*sigma_a;
    }

    // R: measurement noise
    R.setZero();
    R(0,0) = 4.0f;      // barometer
    R(1,1) = 2.0f;      // accel X noise
    R(2,2) = 2.0f;      // accel Y noise
    R(3,3) = 2.0f;      // accel Z noise

    H.setZero();
    H(0,0) = 1.0f;      // position.x
}

void EKF::priori(float dt, Orientation &orientation, FSMState fsm)
{
    // Update stage timestamp for thrust calculations
    if (fsm != last_fsm) {
        stage_timestamp = 0.0f;  // Reset timestamp when FSM state changes
    } else {
        stage_timestamp += dt;
    }
    
    // Get current state
    Eigen::Matrix<float, 3, 1> position = x_k.segment<3>(0);
    Eigen::Matrix<float, 3, 1> velocity = x_k.segment<3>(3);
    Eigen::Matrix<float, 3, 1> acceleration = x_k.segment<3>(6);
    
    // Calculate forces
    Eigen::Matrix<float, 3, 1> total_force = Eigen::Matrix<float, 3, 1>::Zero();
    
    // 1. Gravity force (always downward in global frame)
    Eigen::Matrix<float, 3, 1> gravity_force;
    gravity_force << 0.0f, 0.0f, -gravity_ms2 * getCurrentMass(fsm);
    total_force += gravity_force;
    
    // 2. Thrust force (if in powered flight)
    if (fsm == FSMState::STATE_FIRST_BOOST || fsm == FSMState::STATE_SUSTAINER_IGNITION || fsm == FSMState::STATE_SECOND_BOOST) {
        euler_t euler_angles = orientation.getEuler();
        Eigen::Matrix<float, 3, 1> thrust_force = calculateThrustVector(stage_timestamp, euler_angles, fsm);
        total_force += thrust_force;
    }
    
    // 3. Drag force (opposes velocity)
    if (velocity.norm() > 0.1f) {  // Only calculate drag if moving
        Eigen::Matrix<float, 3, 1> drag_force = calculateDragForce(velocity, position(0));  // altitude is x-component
        total_force += drag_force;
    }
    
    // Calculate new acceleration from total force
    float current_mass = getCurrentMass(fsm);
    Eigen::Matrix<float, 3, 1> new_acceleration = total_force / current_mass;
    
    // State transition matrix (linearized around current state)
    Eigen::Matrix<float,9,9> F_mat = Eigen::Matrix<float,9,9>::Identity();
    for (int i = 0; i < 3; ++i) {
        F_mat(i, i+3) = dt;           // position = position + velocity * dt
        F_mat(i, i+6) = 0.5f * dt * dt; // position = position + 0.5 * acceleration * dt^2
        F_mat(i+3, i+6) = dt;         // velocity = velocity + acceleration * dt
    }
    
    // Predict next state
    x_priori = F_mat * x_k;
    
    // Update acceleration in the predicted state
    x_priori.segment<3>(6) = new_acceleration;
    
    // Update process noise covariance
    P_priori = F_mat * P_k * F_mat.transpose() + Q;
}

void EKF::update(Barometer baro, Acceleration accel_body, Orientation orientation, FSMState fsm)
{
    (void)fsm;
    Eigen::Matrix<float,4,1> z;
    z(0) = baro.altitude;

    // rotation matrix
    euler_t e = orientation.getEuler();
    Eigen::Matrix3f R_be = eulerToRotation(e.yaw, e.pitch, e.roll);

    // accel in body frame
    //Eigen::Vector3f a_pred = R_be.transpose() * Eigen::Vector3f(0, 0, -GRAVITY) +  x_priori.segment<3>(6);
    Eigen::Vector3f a_pred = x_priori.segment<3>(6);

    z(1) = accel_body.ax;
    z(2) = accel_body.ay;
    z(3) = accel_body.az;

    // Predicted measurement
    Eigen::Matrix<float,4,1> z_pred;
    z_pred(0) = x_priori(0);    // altitude
    z_pred(1) = a_pred(0);
    z_pred(2) = a_pred(1);
    z_pred(3) = a_pred(2);

    // Jacobian =
    H.setZero();
    H(0,0) = 1.0f;  // baro altitude
    H(1,6) = 1.0f;  // accel x
    H(2,7) = 1.0f;  // accel y
    H(3,8) = 1.0f;  // accel z

    Eigen::Matrix<float,4,4> S = H * P_priori * H.transpose() + R;
    Eigen::Matrix<float,9,4> K = P_priori * H.transpose() * S.inverse();

    Eigen::Matrix<float,4,1> y = z - z_pred; 
    x_k = x_priori + K * y;
    P_k = (Eigen::Matrix<float,9,9>::Identity() - K * H) * P_priori;
}

// Was throwing errors so i added this
void EKF::priori()
{
    Orientation default_orientation;
    default_orientation.has_data = false;
    priori(s_dt, default_orientation, last_fsm);
}

KalmanData EKF::getState()
{
    KalmanData result;
    result.position.px = x_k(0);
    result.position.py = x_k(1);
    result.position.pz = x_k(2);
    result.velocity.vx = x_k(3);
    result.velocity.vy = x_k(4);
    result.velocity.vz = x_k(5);
    result.acceleration.ax = x_k(6);
    result.acceleration.ay = x_k(7);
    result.acceleration.az = x_k(8);
    result.altitude = x_k(0);
    return result;
}

void EKF::setState(KalmanState state)
{
    x_k(0) = state.state_est_pos_x;
    x_k(1) = state.state_est_pos_y;
    x_k(2) = state.state_est_pos_z;
    x_k(3) = state.state_est_vel_x;
    x_k(4) = state.state_est_vel_y;
    x_k(5) = state.state_est_vel_z;
    x_k(6) = state.state_est_accel_x;
    x_k(7) = state.state_est_accel_y;
    x_k(8) = state.state_est_accel_z;
}

void EKF::tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state)
{
    priori(dt, orientation, state);
    update(barometer, acceleration, orientation, state);
    last_fsm = state;
}

// Placeholder implementations for other methods
void EKF::setQ(float dt, float sd)
{
    (void)dt; (void)sd; 
}

void EKF::setF(float dt, FSMState fsm, float w_x, float w_y, float w_z)
{
    (void)dt; (void)fsm; (void)w_x; (void)w_y; (void)w_z; 
}

void EKF::getThrust(float timestamp, euler_t angles, FSMState FSM_state, Eigen::Matrix<float, 3, 1> &to_modify)
{
    (void)timestamp; (void)angles; (void)FSM_state; (void)to_modify; 
}

void EKF::BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> &x_k, Eigen::Matrix<float, 3, 1> &to_modify)
{
    (void)angles; (void)x_k; (void)to_modify; 
}

void EKF::GlobalToBody(euler_t angles, Eigen::Matrix<float, 3, 1> &to_modify)
{
    (void)angles; (void)to_modify;
}

float EKF::linearInterpolation(float x0, float y0, float x1, float y1, float x)
{
    if (x1 == x0) return y0;
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

// Rocket dynamics implementation
float EKF::getThrustMagnitude(float timestamp, FSMState fsm_state)
{
    const std::map<float, float>* thrust_curve = nullptr;
    
    // Select appropriate thrust curve based on FSM state
    if (fsm_state == FSMState::STATE_FIRST_BOOST) {
        thrust_curve = &O5500X_data;  // Booster
    } else if (fsm_state == FSMState::STATE_SUSTAINER_IGNITION || fsm_state == FSMState::STATE_SECOND_BOOST) {
        thrust_curve = &M685W_data;   // Sustainer
    } else {
        return 0.0f;  // No thrust in other states
    }
    
    // Handle edge cases
    if (timestamp <= 0.0f) return 0.0f;
    
    // Find the thrust value using linear interpolation
    auto it = thrust_curve->lower_bound(timestamp);
    
    if (it == thrust_curve->begin()) {
        return it->second;
    }
    if (it == thrust_curve->end()) {
        return std::prev(it)->second;
    }
    
    auto it_prev = std::prev(it);
    return linearInterpolation(it_prev->first, it_prev->second, it->first, it->second, timestamp);
}

float EKF::getDragCoefficient(float mach)
{
    // Find the appropriate aerodynamic coefficients using linear interpolation
    if (mach <= aero_data[0].mach) {
        return aero_data[0].CA_power_on;
    }
    if (mach >= aero_data[24].mach) {
        return aero_data[24].CA_power_on;
    }
    
    // Linear interpolation between data points
    for (int i = 0; i < 24; i++) {
        if (mach >= aero_data[i].mach && mach <= aero_data[i+1].mach) {
            return linearInterpolation(
                aero_data[i].mach, aero_data[i].CA_power_on,
                aero_data[i+1].mach, aero_data[i+1].CA_power_on,
                mach
            );
        }
    }
    
    return aero_data[0].CA_power_on;  // Default fallback
}

Eigen::Matrix<float, 3, 1> EKF::calculateThrustVector(float timestamp, euler_t angles, FSMState fsm_state)
{
    Eigen::Matrix<float, 3, 1> thrust_vector = Eigen::Matrix<float, 3, 1>::Zero();
    
    float thrust_magnitude = getThrustMagnitude(timestamp, fsm_state);
    if (thrust_magnitude <= 0.0f) {
        return thrust_vector;
    }
    
    // Convert Euler angles to rotation matrix
    Eigen::Matrix3f R = eulerToRotation(angles.yaw, angles.pitch, angles.roll);
    
    // Thrust direction in body frame (assuming thrust is along +Z axis in body frame)
    Eigen::Matrix<float, 3, 1> thrust_body = Eigen::Matrix<float, 3, 1>::Zero();
    thrust_body(2) = thrust_magnitude;  // Thrust in +Z direction (upward in body frame)
    
    // Transform thrust from body frame to global frame
    thrust_vector = R * thrust_body;
    
    return thrust_vector;
}

Eigen::Matrix<float, 3, 1> EKF::calculateDragForce(Eigen::Matrix<float, 3, 1> velocity, float altitude)
{
    Eigen::Matrix<float, 3, 1> drag_force = Eigen::Matrix<float, 3, 1>::Zero();
    
    float velocity_magnitude = velocity.norm();
    if (velocity_magnitude < 0.1f) {  // Avoid division by zero
        return drag_force;
    }
    
    // Calculate Mach number
    float mach = velocity_magnitude / speed_of_sound;
    
    // Get drag coefficient
    float Cd = getDragCoefficient(mach);
    
    // Simple air density model (exponential decay with altitude)
    float air_density = air_density_sea_level * exp(-altitude / 8400.0f);  // Scale height ~8.4km
    
    // Calculate drag force magnitude
    float reference_area = pi * rocket_radius * rocket_radius;
    float drag_magnitude = 0.5f * air_density * velocity_magnitude * velocity_magnitude * reference_area * Cd;
    
    // Drag force opposes velocity direction
    drag_force = -drag_magnitude * velocity / velocity_magnitude;
    
    return drag_force;
}

float EKF::getCurrentMass(FSMState fsm_state)
{
    if (fsm_state == FSMState::STATE_FIRST_BOOST) {
        return mass_full;  // Booster + Sustainer
    } else if (fsm_state == FSMState::STATE_SUSTAINER_IGNITION || fsm_state == FSMState::STATE_SECOND_BOOST) {
        return mass_sustainer;  // Sustainer only
    } else {
        return mass_sustainer;  // Default to sustainer mass
    }
}

EKF ekf;