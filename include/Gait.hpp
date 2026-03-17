#pragma once
#include <vector>
#include <cmath>

// Structure pour stocker une position [Y, Z] d'une patte
struct Point {
    double y;
    double z;
};

// Type pour une "pose" complète (les 4 pattes)
using Pose = std::vector<Point>;

class Gait {
public:
    static constexpr int FORWARD = 1;
    static constexpr int BACKWARD = -1;
    static constexpr int LEFT = -1;
    static constexpr int STRAIGHT = 0;
    static constexpr int RIGHT = 1;

protected:
    double step_y_func(int leg, int step, int step_count, double leg_step_width, double leg_origin, int fb);
    double step_z_func(int step, int step_count, double z_origin, double leg_step_height);
};

class Trot : public Gait {
public:
    Trot(int fb, int lr);
    std::vector<Pose> get_coords();

private:
    int fb, lr;
    double z_origin = 80.0;
    double leg_step_height = 20.0;
    std::vector<double> leg_origin;
    std::vector<double> leg_step_width;
    std::vector<double> section_length;
    std::vector<double> step_down_length;
};

class Walk : public Gait {
public:
    Walk(int fb, int lr);
    std::vector<Pose> get_coords();

private:
    int fb, lr;
    double z_origin = 80.0;
    double leg_step_height = 20.0;
    std::vector<double> leg_origin;
    std::vector<double> leg_step_width;
    std::vector<double> section_length;
    std::vector<double> step_down_length;
};
