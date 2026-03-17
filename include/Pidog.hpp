#pragma once

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>
#include <Eigen/Dense>

// Forward declarations pour tes futurs wrappers matériels
class Robot; 
class Sh3001; 
class Ultrasonic;

class Pidog {
public:
    // Constantes de structure
    static constexpr double LEG = 42.0;
    static constexpr double FOOT = 76.0;
    static constexpr double BODY_LENGTH = 117.0;
    static constexpr double BODY_WIDTH = 98.0;
    
    // Vitesses
    static constexpr int HEAD_DPS = 300;
    static constexpr int LEGS_DPS = 428;
    static constexpr int TAIL_DPS = 500;

    Pidog();
    ~Pidog();

    // Méthodes de contrôle
    void start_action_threads();
    void close();
    void stop_and_lie(int speed = 85);
    
    // Mouvements
    void legs_move(const std::vector<std::vector<double>>& target_angles, bool immediately = true, int speed = 50);
    void head_move(const std::vector<std::vector<double>>& target_yrps, double roll_comp = 0, double pitch_comp = 0, bool immediately = true, int speed = 50);
    
    // Cinématique (Traduction de la logique mathématique Python)
    void set_pose(double x, double y, double z);
    void set_rpy(double roll, double pitch, double yaw, bool pid = false);
    std::vector<double> pose2legs_angle();

private:
    // --- Mathématiques et État ---
    Eigen::MatrixXd BODY_STRUCT;
    Eigen::MatrixXd leg_point_struc;
    Eigen::Vector3d pose;
    Eigen::Vector3d rpy;
    
    double body_height;
    double roll_error_integral, pitch_error_integral;
    double roll_last_error, pitch_last_error;

    // --- Matériel (Pointeurs vers les instances matérielles) ---
    Robot* legs_hw;
    Robot* head_hw;
    Robot* tail_hw;
    Sh3001* imu_hw;
    Ultrasonic* ultrasonic_hw;

    // --- Multithreading ---
    std::atomic<bool> exit_flag;
    
    std::thread legs_thread;
    std::mutex legs_mutex;
    std::condition_variable legs_cv;
    std::queue<std::vector<double>> legs_action_buffer;
    int legs_speed;

    std::thread imu_thread;
    
    // Méthodes internes
    void _legs_action_thread();
    void _imu_thread();
    std::pair<double, double> fieldcoord2polar(double y, double z);
    std::map<std::string, std::vector<std::vector<std::vector<double>>>> actions_dict;
};