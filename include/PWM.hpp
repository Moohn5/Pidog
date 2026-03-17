#pragma once

#include "I2C.hpp"
#include <vector>
#include <cmath>
#include <stdexcept>

class PWM : public I2C {
public:
    static constexpr uint8_t REG_CHN = 0x20;
    static constexpr uint8_t REG_PSC = 0x40;
    static constexpr uint8_t REG_ARR = 0x44;
    static constexpr uint8_t REG_PSC2 = 0x50;
    static constexpr uint8_t REG_ARR2 = 0x54;
    static constexpr double CLOCK = 72000000.0;

    PWM(int channel);

    void set_freq(double freq);
    void set_prescaler(int prescaler);
    void set_period(int arr);
    void set_pulse_width(int pulse_width);

private:
    int _channel;
    int _timer_index;
    int _pulse_width;
    double _freq;
    int _prescaler;

    // Le tableau global python "timer" devient une variable statique en C++
    static std::vector<int> timer_arr;

    void _i2c_write_custom(uint8_t reg, uint16_t value);
};