#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>

class I2C {
public:
    static constexpr int RETRY = 5;

    I2C(int address, int bus = 1) : _address(address), _bus(bus), _fd(-1) {
        std::string device = "/dev/i2c-" + std::to_string(bus);
        _fd = open(device.c_str(), O_RDWR);
        if (_fd < 0) {
            throw std::runtime_error("Erreur: Impossible d'ouvrir le bus I2C " + device);
        }
        if (ioctl(_fd, I2C_SLAVE, _address) < 0) {
            throw std::runtime_error("Erreur: Impossible de se connecter à l'adresse I2C");
        }
    }

    ~I2C() {
        if (_fd >= 0) close(_fd);
    }

    bool write_byte_data(uint8_t reg, uint8_t data) {
        for (int i = 0; i < RETRY; ++i) {
            if (i2c_smbus_write_byte_data(_fd, reg, data) >= 0) return true;
        }
        return false;
    }

    int read_byte_data(uint8_t reg) {
        for (int i = 0; i < RETRY; ++i) {
            int result = i2c_smbus_read_byte_data(_fd, reg);
            if (result >= 0) return result;
        }
        return -1;
    }

    // (Tu pourras ajouter write_word_data, i2c_block_data etc. sur le même modèle)

private:
    int _address;
    int _bus;
    int _fd;
};