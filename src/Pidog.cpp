#include "Pidog.hpp"
#include <iostream>
#include <cmath>
#include <chrono>

using namespace Eigen;
using namespace std;

Pidog::Pidog() : exit_flag(false), body_height(80.0), legs_speed(90) {
    // Initialisation des structures matricielles
    BODY_STRUCT = MatrixXd(3, 4);
    BODY_STRUCT << -BODY_WIDTH / 2,  BODY_WIDTH / 2, -BODY_WIDTH / 2,  BODY_WIDTH / 2,
                   -BODY_LENGTH / 2, -BODY_LENGTH / 2,  BODY_LENGTH / 2,  BODY_LENGTH / 2,
                    0,               0,               0,               0;

    leg_point_struc = BODY_STRUCT; // Valeur initiale
    pose << 0.0, 0.0, body_height;
    rpy << 0.0, 0.0, 0.0;

    // TODO: Instancier ici tes classes matérielles (RobotHat C++)
    // legs_hw = new Robot({2, 3, 7, 8, 0, 1, 10, 11});
}

Pidog::~Pidog() {
    close();
    // cleanup des pointeurs matériels
}

void Pidog::close() {
    exit_flag = true;
    legs_cv.notify_all(); // Réveille les threads endormis
    
    if (legs_thread.joinable()) legs_thread.join();
    if (imu_thread.joinable()) imu_thread.join();
    
    std::cout << "Pidog éteint proprement." << std::endl;
}

// --- Traduction Mathématique Exacte de pose2coords et pose2legs_angle ---

std::vector<double> Pidog::pose2legs_angle() {
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    // Matrices de rotation (Eigen)
    Matrix3d rotx, roty, rotz;
    rotx << cos(roll), 0, -sin(roll),
            0,         1,  0,
            sin(roll), 0,  cos(roll);

    roty << 1,  0,           0,
            0,  cos(-pitch), -sin(-pitch),
            0,  sin(-pitch),  cos(-pitch);

    rotz << cos(yaw), -sin(yaw), 0,
            sin(yaw),  cos(yaw), 0,
            0,         0,        1;

    Matrix3d rot_mat = rotx * roty * rotz;
    MatrixXd AB = MatrixXd::Zero(3, 4);

    for (int i = 0; i < 4; ++i) {
        AB.col(i) = -pose - (rot_mat * BODY_STRUCT.col(i)) + leg_point_struc.col(i);
    }

    MatrixXd body_coor_mat = leg_point_struc - AB;
    
    std::vector<double> final_angles;

    for (int i = 0; i < 4; ++i) {
        double y = leg_point_struc(1, i) - body_coor_mat(1, i);
        double z = body_coor_mat(2, i) - leg_point_struc(2, i);

        auto [leg_angle, foot_angle] = fieldcoord2polar(y, z);
        
        foot_angle -= 90.0;
        
        // Côtés gauche et droit opposés (indice impair = côté droit généralement)
        if (i % 2 != 0) {
            leg_angle = -leg_angle;
            foot_angle = -foot_angle;
        }
        
        final_angles.push_back(leg_angle);
        final_angles.push_back(foot_angle);
    }

    return final_angles;
}

std::pair<double, double> Pidog::fieldcoord2polar(double y, double z) {
    double u = std::sqrt(y*y + z*z);
    
    double cos_angle1 = (std::pow(FOOT, 2) + std::pow(LEG, 2) - std::pow(u, 2)) / (2 * FOOT * LEG);
    cos_angle1 = std::clamp(cos_angle1, -1.0, 1.0);
    double beta = std::acos(cos_angle1);

    double angle1 = std::atan2(y, z);
    double cos_angle2 = (std::pow(LEG, 2) + std::pow(u, 2) - std::pow(FOOT, 2)) / (2 * LEG * u);
    cos_angle2 = std::clamp(cos_angle2, -1.0, 1.0);
    double angle2 = std::acos(cos_angle2);
    
    double alpha = angle2 + angle1 + rpy(1); // rpy(1) est le pitch

    // Conversion en degrés
    alpha = alpha * 180.0 / M_PI;
    beta = beta * 180.0 / M_PI;

    return {alpha, beta};
}

// --- Logique de Thread (Moderne C++) ---

void Pidog::_legs_action_thread() {
    while (!exit_flag) {
        std::vector<double> current_angles;
        
        {
            std::unique_lock<std::mutex> lock(legs_mutex);
            // On attend qu'il y ait une action ou que le programme s'arrête (très économe en CPU comparé au sleep)
            legs_cv.wait(lock, [this]{ return !legs_action_buffer.empty() || exit_flag; });
            
            if (exit_flag) break;
            
            current_angles = legs_action_buffer.front();
            legs_action_buffer.pop();
        }

        // Appel matériel
        // if (legs_hw) legs_hw->servo_move(current_angles, legs_speed);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Délai entre les actions
    }
}