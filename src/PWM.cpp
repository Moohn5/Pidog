#include "PWM.hpp"
#include <iostream>
#include <limits>

// Initialisation de la variable statique (équivalent de timer = [{"arr": 1} for _ in range(7)])
std::vector<int> PWM::timer_arr(7, 1);

PWM::PWM(int channel) : I2C(0x14), _channel(channel), _pulse_width(0), _freq(50.0), _prescaler(0) {
    // Note: Python essaie 0x14, 0x15, 0x16. Pour simplifier en C++, on force 0x14 (l'adresse standard du Robot Hat).
    
    if (_channel < 0 || _channel > 19) {
        throw std::invalid_argument("Le canal PWM doit être entre 0 et 19");
    }

    if (_channel < 16) {
        _timer_index = _channel / 4;
    } else if (_channel == 16 || _channel == 17) {
        _timer_index = 4;
    } else if (_channel == 18) {
        _timer_index = 5;
    } else if (_channel == 19) {
        _timer_index = 6;
    }

    set_freq(50.0);
}

void PWM::_i2c_write_custom(uint8_t reg, uint16_t value) {
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF; // value_h
    data[1] = value & 0xFF;        // value_l
    write_block_data(reg, data, 2);
}

void PWM::set_freq(double freq) {
    _freq = freq;
    
    int st = static_cast<int>(std::sqrt(CLOCK / _freq)) - 5;
    if (st <= 0) st = 1;

    int best_psc = st;
    int best_arr = 1;
    double min_error = std::numeric_limits<double>::max();

    for (int psc = st; psc < st + 10; ++psc) {
        int arr = static_cast<int>(CLOCK / _freq / psc);
        double current_freq = CLOCK / psc / arr;
        double error = std::abs(_freq - current_freq);

        if (error < min_error) {
            min_error = error;
            best_psc = psc;
            best_arr = arr;
        }
    }

    set_prescaler(best_psc);
    set_period(best_arr);
}

void PWM::set_prescaler(int prescaler) {
    _prescaler = prescaler;
    _freq = CLOCK / _prescaler / timer_arr[_timer_index];
    
    uint8_t reg = (_timer_index < 4) ? (REG_PSC + _timer_index) : (REG_PSC2 + _timer_index - 4);
    _i2c_write_custom(reg, _prescaler - 1);
}

void PWM::set_period(int arr) {
    timer_arr[_timer_index] = arr;
    _freq = CLOCK / _prescaler / timer_arr[_timer_index];
    
    uint8_t reg = (_timer_index < 4) ? (REG_ARR + _timer_index) : (REG_ARR2 + _timer_index - 4);
    _i2c_write_custom(reg, timer_arr[_timer_index]);
}

void PWM::set_pulse_width(int pulse_width) {
    _pulse_width = pulse_width;
    uint8_t reg = REG_CHN + _channel;
    _i2c_write_custom(reg, _pulse_width);
}