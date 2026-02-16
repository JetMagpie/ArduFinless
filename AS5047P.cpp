#include "AS5047P.h"

AS5047P::AS5047P(uint8_t cs_pin)
    : _cs_pin(cs_pin),
      _angle_deg(0.0f),
      _angle_rad(0.0f),
      _raw_angle(0),
      _error_flag(false)
{
}

bool AS5047P::init() {
    Serial.println("=== AS5047P Initialization===");
    
    // 设置CS引脚
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);
    
    // 初始化SPI（确保SPI已初始化）
    SPI.begin();
    
    // 测试读取
    uint16_t test_value = spi_read_angle_raw();
    if (test_value == 0xFFFF || test_value == 0x0000) {
        Serial.println("AS5047P no response");
        return false;
    }
    
    Serial.print("AS5047P Initialized, angle: ");
    Serial.println((test_value * 360.0f) / RESOLUTION);
    Serial.println("========================");
    
    return true;
}

uint16_t AS5047P::spi_read_angle_raw() {
    // 切换到AS5047P的SPI模式
    SPI.setDataMode(_as5047p_spi_mode);
    SPI.setBitOrder(MSBFIRST);
    
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(1);
    
    // 发送读取角度命令（NOP命令0xFFFF）
    uint16_t command = 0xFFFF;
    uint16_t response = SPI.transfer16(command);
    
    delayMicroseconds(1);
    digitalWrite(_cs_pin, HIGH);
    
    // 返回14位角度值（去掉错误标志位）
    return response & 0x3FFF;
}

bool AS5047P::read_angle() {
    _raw_angle = spi_read_angle_raw();
    
    // 检查是否读取成功（0x3FFF是最大值，不应该出现）
    if (_raw_angle == 0xFFFF || _raw_angle == 0x0000) {
        _error_flag = true;
        return false;
    }
    
    _error_flag = false;
    
    // 转换为角度（0-360度）
    _angle_deg = (_raw_angle * 360.0f) / RESOLUTION;
    _angle_rad = (_raw_angle * 2.0f * PI) / RESOLUTION;
    
    return true;
}