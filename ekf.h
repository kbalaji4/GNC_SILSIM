#pragma once

#include "kalman_filter.h"
#include "sensor_data.h"
#include "Buffer.h"
#include <map>
#include <string>

#define NUM_STATES 9
#define NUM_SENSOR_INPUTS 4
#define ALTITUDE_BUFFER_SIZE 10

// Aerodynamic coefficient structure
typedef struct
{
    float mach;
    float alpha;
    float CA_power_on;
    float CN;
    float CP;
} aero_coeff_t;

class EKF : public KalmanFilter<NUM_STATES, NUM_SENSOR_INPUTS>
{
public:
    EKF();
    void initialize(RocketSystems* args) override;
    void priori() override;
    void priori(float dt, Orientation &orientation, FSMState fsm); 
    void update(Barometer barometer, Acceleration acceleration, Orientation orientation, FSMState state) override;
    Eigen::Matrix3f eulerToRotation(float yaw, float pitch, float roll);
    void setQ(float dt, float sd);
    void setF(float dt, FSMState fsm, float w_x, float w_y, float w_z); 
    void getThrust(float timestamp, euler_t angles, FSMState FSM_state,  Eigen::Matrix<float, 3, 1> &to_modify);
    void BodyToGlobal(euler_t angles, Eigen::Matrix<float, 3, 1> &x_k,  Eigen::Matrix<float, 3, 1> &to_modify);
    void GlobalToBody(euler_t angles, Eigen::Matrix<float, 3, 1> &to_modify);

    KalmanData getState() override;
    void setState(KalmanState state) override;

    float linearInterpolation(float x0, float y0, float x1, float y1, float x);
    
    // New rocket dynamics functions
    float getThrustMagnitude(float timestamp, FSMState fsm_state);
    float getDragCoefficient(float mach);
    Eigen::Matrix<float, 3, 1> calculateThrustVector(float timestamp, euler_t angles, FSMState fsm_state);
    Eigen::Matrix<float, 3, 1> calculateDragForce(Eigen::Matrix<float, 3, 1> velocity, float altitude);
    float getCurrentMass(FSMState fsm_state);
    Eigen::Matrix<float,9,9> calculateJacobian(float dt, Eigen::Matrix<float, 3, 1> position, 
                                               Eigen::Matrix<float, 3, 1> velocity, 
                                               Orientation &orientation, FSMState fsm);

    void tick(float dt, float sd, Barometer &barometer, Acceleration acceleration, Orientation &orientation, FSMState state);
   
    bool should_reinit = false;
private:
    float s_dt = 0.05f;
    float spectral_density_ = 13.0f;
    float kalman_apo = 0;
    float Ca = 0;
    float Cn = 0;
    float Cp = 0;
    Eigen::Matrix<float,3,1> gravity = Eigen::Matrix<float,3,1>::Zero();
    KalmanState kalman_state;
    FSMState last_fsm = FSMState::STATE_IDLE;
    float stage_timestamp = 0;

    Eigen::Matrix<float, 3, 1> init_accel = Eigen::Matrix<float, 3, 1>::Zero();
    Buffer<float, ALTITUDE_BUFFER_SIZE> alt_buffer;
    KalmanData state;
    
    // Rocket dynamics constants and data
    static constexpr float pi = 3.14159268f;
    static constexpr float speed_of_sound = 343.0f;  // m/s
    static constexpr float air_density_sea_level = 1.225f;  // kg/m³
    static constexpr float rocket_radius = 0.03935f;  // m
    static constexpr float height_full = 4.457f;  // m
    static constexpr float height_sustainer = 2.029f;  // m
    static constexpr float mass_full = 33.84f;  // kg
    static constexpr float mass_sustainer = 10.93f;  // kg
    static constexpr float gravity_ms2 = 9.81f;  // m/s²
    
    // Aerodynamic data
    static const aero_coeff_t aero_data[25];
    
    // Motor thrust curves
    static const std::map<float, float> M685W_data;  // Sustainer
    static const std::map<float, float> O5500X_data;  // Booster
    static const std::map<std::string, std::map<float, float>> motor_data;
};

extern EKF ekf;