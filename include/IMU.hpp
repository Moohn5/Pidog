#pragma once
#include "I2C.hpp"
#include <vector>
#include <string>

class IMU : public I2C {
public:
    IMU(std::string config_path = "sh3001.config");
    
    struct Data { std::vector<double> acc; std::vector<double> gyro; };

    bool init();
    Data get_data();
    double get_temp();

private:
    void module_reset();
    void acc_config();
    void gyro_config();
    int16_t bytes_to_int16(uint8_t msb, uint8_t lsb);

    std::vector<double> acc_offset = {0,0,0};
    std::vector<double> gyro_offset = {0,0,0};
    
    // Registres SH3001
    static constexpr uint8_t ADDR = 0x36;
    static constexpr uint8_t REG_ACC_X = 0x00;
    static constexpr uint8_t REG_CHIP_ID = 0x0F;
};
