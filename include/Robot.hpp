#pragma once

#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <thread>
#include <cmath>
#include "Servo.hpp"

class Robot {
public:
    int max_dps = 428;

    Robot(const std::vector<int>& pin_list);
    
    void servo_write_raw(const std::vector<double>& angle_list);
    void servo_write_all(const std::vector<double>& angles);
    void servo_move(const std::vector<double>& targets, int speed = 50, double bpm = 0.0);
    
    void reset();
    void set_offset(const std::vector<double>& offset_list);

private:
    std::string name;
    int pin_num;
    std::vector<Servo> servo_list;
    std::vector<double> offset;
    std::vector<double> servo_positions;
    std::vector<double> origin_positions;
    std::vector<double> direction;

    std::vector<double> new_list(double default_value);
    void load_offsets_from_file(); // Remplace fileDB
};