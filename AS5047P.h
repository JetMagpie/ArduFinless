#ifndef __AS5047P_H__
#define __AS5047P_H__

#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

class AS5047P {
public:
    AS5047P(uint8_t cs_pin = 49);
    
    bool init();
    bool read_angle();
    
    // 获取数据
    float get_angle_deg() const { return _angle_deg; }
    float get_angle_rad() const { return _angle_rad; }
    uint16_t get_raw_angle() const { return _raw_angle; }
    bool get_error() const { return _error_flag; }
    
private:
    // SPI读取角度函数
    uint16_t spi_read_angle_raw();
    
    // 引脚
    uint8_t _cs_pin;
    
    // SPI设置
    uint32_t _spi_speed = 1000000;  // 1MHz SPI速度
    uint8_t _as5047p_spi_mode = SPI_MODE1;  // AS5047P使用模式1
    
    // 传感器数据
    float _angle_deg;
    float _angle_rad;
    uint16_t _raw_angle;
    bool _error_flag;
    
    // 分辨率
    static const uint16_t RESOLUTION = 16384;  // 14位分辨率
};

#endif // __AS5047P_H__