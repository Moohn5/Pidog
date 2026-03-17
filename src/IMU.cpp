#include "IMU.hpp"
#include <thread>
#include <chrono>

IMU::IMU(std::string config_path) : I2C(ADDR) {
    if (!init()) {
        throw std::runtime_error("SH3001 (IMU) non détecté sur le bus I2C");
    }
}

int16_t IMU::bytes_to_int16(uint8_t msb, uint8_t lsb) {
    return static_cast<int16_t>((msb << 8) | lsb);
}

bool IMU::init() {
    uint8_t id = read_byte_data(REG_CHIP_ID);
    if (id != 0x61) return false;

    module_reset();
    acc_config();
    gyro_config();
    return true;
}

IMU::Data IMU::get_data() {
    uint8_t buffer[12];
    read_block_data(REG_ACC_X, buffer, 12);
    
    Data d;
    d.acc = { (double)bytes_to_int16(buffer[1], buffer[0]), 
              (double)bytes_to_int16(buffer[3], buffer[2]), 
              (double)bytes_to_int16(buffer[5], buffer[4]) };
              
    d.gyro = { (double)bytes_to_int16(buffer[7], buffer[6]), 
               (double)bytes_to_int16(buffer[9], buffer[8]), 
               (double)bytes_to_int16(buffer[11], buffer[10]) };
    return d;
}

void IMU::module_reset() {
    write_byte_data(0x7F, 0x73); // Soft reset selon ton Python
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}
// ... (Les autres fonctions de config suivent la même logique de write_byte_data)
