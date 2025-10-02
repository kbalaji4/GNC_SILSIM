#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include "ekf.h"
#include "sensor_data.h"
#include "systems.h"

// CSV Data structure to hold parsed flight data
struct FlightData {
    std::string sensor;
    int file_number;
    float timestamp;
    LowGData lowg;
    HighGData highg;
    Barometer barometer;
    Continuity continuity;
    Voltage voltage;
    GPS gps;
    Magnetometer magnetometer;
    Orientation orientation;
    LowGLSMData lowglsm;
    FSMState fsm;
    KalmanData kalman;
    Pyro pyro;
};

class CSVReader {
private:
    std::vector<std::string> headers;
    std::map<std::string, int> header_index;
    
    static bool isValidNumber(const std::string &s) {
        if (s.empty()) return false;
        if (s == "NaN" || s == "nan" || s == "NAN") return false;
        return true;
    }
    
    static float toFloatOr(const std::string &s, float fallback) {
        if (!isValidNumber(s)) return fallback;
        try { return std::stof(s); } catch (...) { return fallback; }
    }
    
public:
    std::vector<FlightData> readCSV(const std::string& filename) {
        std::vector<FlightData> data;
        std::ifstream file(filename);
        std::string line;
        
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return data;
        }
        
        // Read headers
        if (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string header;
            int index = 0;
            while (std::getline(ss, header, ',')) {
                headers.push_back(header);
                header_index[header] = index++;
            }
        }
        
        // Read data rows
        FlightData last_valid{};
        last_valid.sensor = "";
        last_valid.file_number = 0;
        last_valid.timestamp = 0.0f;
        while (std::getline(file, line)) {
            FlightData row;
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> values;
            
            while (std::getline(ss, cell, ',')) {
                values.push_back(cell);
            }
            
            if (values.size() < headers.size()) {
                continue; // Skip incomplete rows
            }
            
            // Parse the data (forward-fill any missing/NaN values using last_valid)
            try {
                // start from last snapshot
                row = last_valid;

                // meta
                {
                    std::string v = getValue(values, "sensor");
                    if (!v.empty() && v != "NaN") row.sensor = v;
                }
                {
                    std::string v = getValue(values, "file number");
                    if (!v.empty() && v != "NaN") { try { row.file_number = std::stoi(v); } catch (...) {} }
                }
                row.timestamp = toFloatOr(getValue(values, "timestamp"), row.timestamp);
                
                // LowG data
                row.lowg.ax = toFloatOr(getValue(values, "lowg.ax"), row.lowg.ax);
                row.lowg.ay = toFloatOr(getValue(values, "lowg.ay"), row.lowg.ay);
                row.lowg.az = toFloatOr(getValue(values, "lowg.az"), row.lowg.az);
                
                // HighG data
                row.highg.ax = toFloatOr(getValue(values, "highg.ax"), row.highg.ax);
                row.highg.ay = toFloatOr(getValue(values, "highg.ay"), row.highg.ay);
                row.highg.az = toFloatOr(getValue(values, "highg.az"), row.highg.az);
                
                // Barometer data
                row.barometer.temperature = toFloatOr(getValue(values, "barometer.temperature"), row.barometer.temperature);
                row.barometer.pressure = toFloatOr(getValue(values, "barometer.pressure"), row.barometer.pressure);
                row.barometer.altitude = toFloatOr(getValue(values, "barometer.altitude"), row.barometer.altitude);
                
                // Orientation data
                {
                    std::string v = getValue(values, "orientation.has_data");
                    if (!v.empty() && v != "NaN") { try { row.orientation.has_data = std::stoi(v) != 0; } catch (...) {} }
                }
                {
                    std::string v = getValue(values, "orientation.reading_type");
                    if (!v.empty() && v != "NaN") row.orientation.reading_type = v;
                }
                row.orientation.yaw = toFloatOr(getValue(values, "orientation.yaw"), row.orientation.yaw);
                row.orientation.pitch = toFloatOr(getValue(values, "orientation.pitch"), row.orientation.pitch);
                row.orientation.roll = toFloatOr(getValue(values, "orientation.roll"), row.orientation.roll);
                
                row.orientation.orientation_velocity.vx = toFloatOr(getValue(values, "orientation.orientation_velocity.vx"), row.orientation.orientation_velocity.vx);
                row.orientation.orientation_velocity.vy = toFloatOr(getValue(values, "orientation.orientation_velocity.vy"), row.orientation.orientation_velocity.vy);
                row.orientation.orientation_velocity.vz = toFloatOr(getValue(values, "orientation.orientation_velocity.vz"), row.orientation.orientation_velocity.vz);
                
                row.orientation.angular_velocity.vx = toFloatOr(getValue(values, "orientation.angular_velocity.vx"), row.orientation.angular_velocity.vx);
                row.orientation.angular_velocity.vy = toFloatOr(getValue(values, "orientation.angular_velocity.vy"), row.orientation.angular_velocity.vy);
                row.orientation.angular_velocity.vz = toFloatOr(getValue(values, "orientation.angular_velocity.vz"), row.orientation.angular_velocity.vz);
                
                // FSM State
                {
                    std::string fsm_str = getValue(values, "fsm");
                    if (!fsm_str.empty() && fsm_str != "NaN") {
                        if (fsm_str == "STATE_SAFE") row.fsm = FSMState::STATE_SAFE;
                        else if (fsm_str == "STATE_PYRO_TEST") row.fsm = FSMState::STATE_PYRO_TEST;
                        else if (fsm_str == "STATE_IDLE") row.fsm = FSMState::STATE_IDLE;
                        else if (fsm_str == "STATE_FIRST_BOOST") row.fsm = FSMState::STATE_FIRST_BOOST;
                        else if (fsm_str == "STATE_BURNOUT") row.fsm = FSMState::STATE_BURNOUT;
                        else if (fsm_str == "STATE_COAST") row.fsm = FSMState::STATE_COAST;
                        else if (fsm_str == "STATE_APOGEE") row.fsm = FSMState::STATE_APOGEE;
                        else if (fsm_str == "STATE_DROGUE_DEPLOY") row.fsm = FSMState::STATE_DROGUE_DEPLOY;
                        else if (fsm_str == "STATE_DROGUE") row.fsm = FSMState::STATE_DROGUE;
                        else if (fsm_str == "STATE_MAIN_DEPLOY") row.fsm = FSMState::STATE_MAIN_DEPLOY;
                        else if (fsm_str == "STATE_MAIN") row.fsm = FSMState::STATE_MAIN;
                        else if (fsm_str == "STATE_LANDED") row.fsm = FSMState::STATE_LANDED;
                        else if (fsm_str == "STATE_SUSTAINER_IGNITION") row.fsm = FSMState::STATE_SUSTAINER_IGNITION;
                        else if (fsm_str == "STATE_SECOND_BOOST") row.fsm = FSMState::STATE_SECOND_BOOST;
                        else if (fsm_str == "STATE_FIRST_SEPARATION") row.fsm = FSMState::STATE_FIRST_SEPARATION;
                    }
                }

                // update last_valid snapshot
                last_valid = row;
                
                data.push_back(row);
            } catch (const std::exception& e) {
                // Skip rows with parsing errors
                continue;
            }
        }
        
        file.close();
        return data;
    }
    
private:
    std::string getValue(const std::vector<std::string>& values, const std::string& header) {
        auto it = header_index.find(header);
        if (it != header_index.end() && it->second < values.size()) {
            return values[it->second];
        }
        return "0"; // Default value
    }
};

class EKFSimulator {
private:
    EKF ekf;
    RocketSystems rocket_systems;
    std::vector<FlightData> flight_data;
    std::vector<KalmanData> results;
    float start_time;
    
    static const char* fsmToString(FSMState s) {
        switch (s) {
            case FSMState::STATE_SAFE: return "STATE_SAFE";
            case FSMState::STATE_PYRO_TEST: return "STATE_PYRO_TEST";
            case FSMState::STATE_IDLE: return "STATE_IDLE";
            case FSMState::STATE_FIRST_BOOST: return "STATE_FIRST_BOOST";
            case FSMState::STATE_BURNOUT: return "STATE_BURNOUT";
            case FSMState::STATE_COAST: return "STATE_COAST";
            case FSMState::STATE_APOGEE: return "STATE_APOGEE";
            case FSMState::STATE_DROGUE_DEPLOY: return "STATE_DROGUE_DEPLOY";
            case FSMState::STATE_DROGUE: return "STATE_DROGUE";
            case FSMState::STATE_MAIN_DEPLOY: return "STATE_MAIN_DEPLOY";
            case FSMState::STATE_MAIN: return "STATE_MAIN";
            case FSMState::STATE_LANDED: return "STATE_LANDED";
            case FSMState::STATE_SUSTAINER_IGNITION: return "STATE_SUSTAINER_IGNITION";
            case FSMState::STATE_SECOND_BOOST: return "STATE_SECOND_BOOST";
            case FSMState::STATE_FIRST_SEPARATION: return "STATE_FIRST_SEPARATION";
        }
        return "STATE_SAFE";
    }
    
public:
    EKFSimulator(const std::string& csv_filename) {
        CSVReader reader;
        flight_data = reader.readCSV(csv_filename);
        std::cout << "Loaded " << flight_data.size() << " data points" << std::endl;
        
        if (!flight_data.empty()) {
            start_time = flight_data[0].timestamp;
        }
    }
    
    void runSimulation() {
        if (flight_data.empty()) {
            std::cerr << "No flight data available" << std::endl;
            return;
        }
        
        std::cout << "Starting EKF simulation..." << std::endl;
        
        // Initialize the EKF
        initializeEKF();
        
        float last_timestamp = start_time;
        
        for (size_t i = 0; i < flight_data.size(); i++) {
            const FlightData& data = flight_data[i];
            
            // Calculate time step
            float dt = (data.timestamp - last_timestamp) / 1000.0f; // Convert to seconds
            if (dt < 0 || dt > 1.0f) dt = 0.05f; // Clamp to reasonable values
            
            // Update sensor buffers
            updateSensorBuffers(data);
            
            // Create sensor data structures
            Barometer current_barom = data.barometer;
            Acceleration current_accel = {data.highg.ax, data.highg.ay, data.highg.az};
            Orientation current_orientation = data.orientation;
            FSMState current_fsm = data.fsm;
            
            // Run EKF tick
            ekf.tick(dt, 13.0f, current_barom, current_accel, current_orientation, current_fsm);
            
            // Get and store results
            KalmanData current_state = ekf.getState();
            results.push_back(current_state);
            
            // Print progress every 1000 iterations
            if (i % 1000 == 0) {
                std::cout << "Processed " << i << " data points" << std::endl;
                std::cout << "Time: " << (data.timestamp - start_time) / 1000.0f << "s" << std::endl;
                std::cout << "Position: (" << current_state.position.px << ", " 
                         << current_state.position.py << ", " << current_state.position.pz << ")" << std::endl;
                std::cout << "Velocity: (" << current_state.velocity.vx << ", " 
                         << current_state.velocity.vy << ", " << current_state.velocity.vz << ")" << std::endl;
                std::cout << "Acceleration: (" << current_state.acceleration.ax << ", " 
                         << current_state.acceleration.ay << ", " << current_state.acceleration.az << ")" << std::endl;
                std::cout << "---" << std::endl;
            }
            
            last_timestamp = data.timestamp;
        }
        
        std::cout << "Simulation completed. Processed " << results.size() << " data points." << std::endl;
    }
    
    void saveResults(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not create output file " << filename << std::endl;
            return;
        }
        
        // Write header
        file << "timestamp,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,altitude,fsm" << std::endl;
        
        // Write data
        for (size_t i = 0; i < results.size() && i < flight_data.size(); i++) {
            const KalmanData& result = results[i];
            const FlightData& data = flight_data[i];
            
            file << std::fixed << std::setprecision(6)
                 << (data.timestamp - start_time) / 1000.0f << ","
                 << result.position.px << ","
                 << result.position.py << ","
                 << result.position.pz << ","
                 << result.velocity.vx << ","
                 << result.velocity.vy << ","
                 << result.velocity.vz << ","
                 << result.acceleration.ax << ","
                 << result.acceleration.ay << ","
                 << result.acceleration.az << ","
                 << result.altitude << ","
                 << fsmToString(data.fsm) << std::endl;
        }
        
        file.close();
        std::cout << "Results saved to " << filename << std::endl;
    }
    
private:
    void initializeEKF() {
        // Initialize sensor buffers with first few data points
        for (int i = 0; i < std::min(30, (int)flight_data.size()); i++) {
            updateSensorBuffers(flight_data[i]);
        }
        
        // Initialize the EKF
        ekf.initialize(&rocket_systems);
        std::cout << "EKF initialized" << std::endl;
    }
    
    void updateSensorBuffers(const FlightData& data) {
        rocket_systems.rocket_data.barometer.push(data.barometer);
        rocket_systems.rocket_data.orientation.push(data.orientation);
        rocket_systems.rocket_data.high_g.push(data.highg);
        rocket_systems.rocket_data.low_g.push(data.lowg);
        rocket_systems.rocket_data.fsm_state.push(data.fsm);
    }
};

int main(int argc, char* argv[]) {
    std::string csv_filename = "MIDAS Sustainer (Trimmed CSV).csv";
    std::string output_filename = "ekf_results.csv";
    
    if (argc > 1) {
        csv_filename = argv[1];
    }
    if (argc > 2) {
        output_filename = argv[2];
    }
    
    std::cout << "EKF Flight Data Simulator" << std::endl;
    std::cout << "Input file: " << csv_filename << std::endl;
    std::cout << "Output file: " << output_filename << std::endl;
    std::cout << "---" << std::endl;
    
    try {
        EKFSimulator simulator(csv_filename);
        simulator.runSimulation();
        simulator.saveResults(output_filename);
        
        std::cout << "Simulation completed successfully!" << std::endl;
        std::cout << "Results saved to: " << output_filename << std::endl;
        std::cout << "You can now plot the results using the provided Python script." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
