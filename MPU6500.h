#ifndef __MPU6500_H__
#define __MPU6500_H__

#include <SPI.h>

class MPU6500 {
public:
    MPU6500(uint8_t cs_pin = 48, bool debug = true);
    
    bool init();
    bool read_sensors();
    
    // 获取数据
    float get_accel_x() const { return _accel_x; }
    float get_accel_y() const { return _accel_y; }
    float get_accel_z() const { return _accel_z; }
    float get_gyro_x() const { return _gyro_x; }
    float get_gyro_y() const { return _gyro_y; }
    float get_gyro_z() const { return _gyro_z; }
    float get_temperature() const { return _temperature; }
    
    // 调试信息
    uint8_t get_last_whoami() const { return _last_whoami; }
    
private:
    // SPI通信
    uint8_t read_register(uint8_t reg);
    void write_register(uint8_t reg, uint8_t value);
    void read_registers(uint8_t reg, uint8_t* buffer, uint8_t length);
    uint8_t _mpu6500_spi_mode = SPI_MODE3;  // MPU6500使用模式3
    
    // 引脚
    uint8_t _cs_pin;
    bool _debug;
    
    // 传感器数据
    float _accel_x, _accel_y, _accel_z;
    float _gyro_x, _gyro_y, _gyro_z;
    float _temperature;
    
    // 调试信息
    uint8_t _last_whoami;
    bool _initialized;
    
    // 寄存器地址
    static const uint8_t REG_WHO_AM_I = 0x75;
    static const uint8_t REG_PWR_MGMT_1 = 0x6B;
    static const uint8_t REG_GYRO_CONFIG = 0x1B;
    static const uint8_t REG_ACCEL_CONFIG = 0x1C;
    static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
};

#endif // __MPU6500_H__