#pragma once
#include <vector>
#include <cmath>

struct Point {
    double y;
    double z;
};

using Pose = std::vector<Point>;

class Gait {
public:
    static constexpr int FORWARD = 1;
    static constexpr int BACKWARD = -1;
    static constexpr int LEFT = -1;
    static constexpr int STRAIGHT = 0;
    static constexpr int RIGHT = 1;

protected:
    double step_y_func(int step, int step_count, double leg_step_width, double leg_origin, int fb) {
        double theta = step * M_PI / (step_count - 1);
        return leg_origin + (leg_step_width * (std::cos(theta) - fb) / 2.0 * fb);
    }

    double step_z_func(int step, int step_count, double z_origin, double leg_step_height) {
        return z_origin - (leg_step_height * step / (step_count - 1));
    }
};

class Trot : public Gait {
public:
    Trot(int fb, int lr);
    std::vector<Pose> get_coords();
private:
    int fb, lr;
    double z_origin = 80.0;
    std::vector<double> leg_origin, leg_step_width, section_length, step_down_length;
};

class Walk : public Gait {
public:
    Walk(int fb, int lr);
    std::vector<Pose> get_coords();
private:
    int fb, lr;
    double z_origin = 80.0;
    std::vector<double> leg_origin, leg_step_width, section_length, step_down_length;
};
