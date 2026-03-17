#pragma once

#include "PWM.hpp"
#include <algorithm>

class Servo : public PWM {
public:
    static constexpr int MAX_PW = 2500;
    static constexpr int MIN_PW = 500;
    static constexpr int FREQ = 50;
    static constexpr int PERIOD = 4095;

    Servo(int channel) : PWM(channel) {
        set_period(PERIOD);
        double prescaler_val = CLOCK / FREQ / PERIOD;
        set_prescaler(static_cast<int>(prescaler_val));
    }

    void set_angle(double angle) {
        angle = std::clamp(angle, -90.0, 90.0);
        double pw_time = map_value(angle, -90.0, 90.0, MIN_PW, MAX_PW);
        set_pulse_width_time(pw_time);
    }

private:
    void set_pulse_width_time(double pw_time) {
        pw_time = std::clamp(pw_time, (double)MIN_PW, (double)MAX_PW);
        double pwr = pw_time / 20000.0;
        int value = static_cast<int>(pwr * PERIOD);
        set_pulse_width(value); // Appel de la méthode héritée de PWM
    }

    double map_value(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};