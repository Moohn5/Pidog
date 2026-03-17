#include "Robot.hpp"

using namespace std::chrono;

Robot::Robot(const std::vector<int>& pin_list) : pin_num(pin_list.size()) {
    name = "other";
    
    // Initialisation des vecteurs
    offset = new_list(0.0); // À remplacer par la lecture d'un fichier de config
    servo_positions = new_list(0.0);
    origin_positions = new_list(0.0);
    direction = new_list(1.0);

    // Instanciation des servos
    for (int pin : pin_list) {
        servo_list.emplace_back(Servo(pin));
    }

    // Mise en position initiale avec offset
    for (int i = 0; i < pin_num; ++i) {
        servo_list[i].set_angle(offset[i] + servo_positions[i]);
        std::this_thread::sleep_for(milliseconds(150));
    }
}

std::vector<double> Robot::new_list(double default_value) {
    return std::vector<double>(pin_num, default_value);
}

void Robot::servo_write_raw(const std::vector<double>& angle_list) {
    for (int i = 0; i < pin_num; ++i) {
        servo_list[i].set_angle(angle_list[i]);
    }
}

void Robot::servo_write_all(const std::vector<double>& angles) {
    std::vector<double> rel_angles(pin_num);
    for (int i = 0; i < pin_num; ++i) {
        rel_angles[i] = direction[i] * (origin_positions[i] + angles[i] + offset[i]);
    }
    servo_write_raw(rel_angles);
}

// Le coeur du mouvement fluide (Traduction exacte de ton Python)
void Robot::servo_move(const std::vector<double>& targets, int speed, double bpm) {
    speed = std::clamp(speed, 0, 100);
    double step_time = 10.0; // ms
    
    std::vector<double> delta(pin_num);
    std::vector<double> absdelta(pin_num);
    double max_delta = 0.0;

    for (int i = 0; i < pin_num; ++i) {
        delta[i] = targets[i] - servo_positions[i];
        absdelta[i] = std::abs(delta[i]);
        if (absdelta[i] > max_delta) {
            max_delta = absdelta[i];
        }
    }

    if (max_delta == 0.0) {
        std::this_thread::sleep_for(milliseconds(static_cast<int>(step_time)));
        return;
    }

    double total_time;
    if (bpm > 0) {
        total_time = (60.0 / bpm) * 1000.0;
    } else {
        total_time = -9.9 * speed + 1000.0;
    }

    double current_max_dps = (max_delta / total_time) * 1000.0;

    if (current_max_dps > max_dps) {
        total_time = (max_delta / max_dps) * 1000.0;
    }

    int max_step = static_cast<int>(total_time / step_time);
    if (max_step == 0) max_step = 1; // Sécurité anti division par 0

    std::vector<double> steps(pin_num);
    for (int i = 0; i < pin_num; ++i) {
        steps[i] = delta[i] / max_step;
    }

    for (int s = 0; s < max_step; ++s) {
        auto start_timer = high_resolution_clock::now();

        for (int j = 0; j < pin_num; ++j) {
            servo_positions[j] += steps[j];
        }
        servo_write_all(servo_positions);

        auto servo_move_time = duration_cast<milliseconds>(high_resolution_clock::now() - start_timer).count();
        int delay = static_cast<int>(step_time - servo_move_time);
        
        if (delay > 0) {
            std::this_thread::sleep_for(milliseconds(delay));
        }
    }
}