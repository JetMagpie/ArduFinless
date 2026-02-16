#include "MPU6500.h"

MPU6500::MPU6500(uint8_t cs_pin, bool debug)
    : _cs_pin(cs_pin), _debug(debug),
      _accel_x(0), _accel_y(0), _accel_z(0),
      _gyro_x(0), _gyro_y(0), _gyro_z(0),
      _temperature(0), _last_whoami(0), _initialized(false)
{
}

uint8_t MPU6500::read_register(uint8_t reg) {
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(5);  // CS建立时间（电平转换器需要）
    
    SPI.transfer(reg | 0x80);  // 读命令
    delayMicroseconds(2);      // 电平转换器延迟
    
    uint8_t value = SPI.transfer(0x00);
    delayMicroseconds(2);
    
    digitalWrite(_cs_pin, HIGH);
    delayMicroseconds(5);  // CS释放时间
    
    return value;
}

void MPU6500::write_register(uint8_t reg, uint8_t value) {
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(5);
    
    SPI.transfer(reg & 0x7F);  // 写命令
    delayMicroseconds(2);
    
    SPI.transfer(value);
    delayMicroseconds(2);
    
    digitalWrite(_cs_pin, HIGH);
    delayMicroseconds(5);
}

void MPU6500::read_registers(uint8_t reg, uint8_t* buffer, uint8_t length) {
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(5);
    
    SPI.transfer(reg | 0x80);
    delayMicroseconds(2);
    
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = SPI.transfer(0x00);
        delayMicroseconds(2);  // 每个字节之间的延迟
    }
    
    digitalWrite(_cs_pin, HIGH);
    delayMicroseconds(5);
}

bool MPU6500::init() {
    if (_debug) {
        Serial.println("MPU6500 initialization");
    }
    
    // 初始化SPI - 使用诊断确认的设置
    SPI.begin();
    
    // 关键设置：SPI模式3，100kHz（根据诊断结果）
    SPI.setDataMode(_mpu6500_spi_mode);          // CPOL=1, CPHA=1
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV128); // 100kHz
    
    // 设置CS引脚
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);
    
    delay(100);  // 给电平转换器和MPU6500稳定时间
    
    // 测试通信 - 尝试读取WHO_AM_I
    if (_debug) Serial.println("Reading WHO_AM_I...");
    
    for (int attempt = 1; attempt <= 3; attempt++) {
        _last_whoami = read_register(REG_WHO_AM_I);
        
        if (_debug) {
            Serial.print("Attempt ");
            Serial.print(attempt);
            Serial.print(": WHO_AM_I = 0x");
            Serial.println(_last_whoami, HEX);
        }
        
        if (_last_whoami == 0x70) {
            if (_debug) Serial.println("MPU6500 Found");
            break;
        }
        
        delay(100);
    }
    
    if (_last_whoami != 0x70) {
        if (_debug) {
            Serial.println("Failed to detect MPU6500");
        }
        return false;
    }
    
    // 复位设备
    if (_debug) Serial.println("Resetting MPU6500...");
    write_register(REG_PWR_MGMT_1, 0x80);  // 复位
    delay(100);
    
    // 等待复位完成
    uint8_t timeout = 0;
    while (read_register(REG_PWR_MGMT_1) & 0x80) {
        delay(10);
        timeout++;
        if (timeout > 50) {
            if (_debug) Serial.println("Reset timeout");
            return false;
        }
    }
    
    // 配置设备
    if (_debug) Serial.println("Configuring MPU6500");
    
    // 唤醒设备，使用陀螺仪X轴作为时钟源
    write_register(REG_PWR_MGMT_1, 0x01);
    delay(10);
    
    // 设置陀螺仪量程 ±2000dps
    write_register(REG_GYRO_CONFIG, 0x18);
    delay(10);
    
    // 设置加速度计量程 ±8g
    write_register(REG_ACCEL_CONFIG, 0x10);
    delay(10);
    
    // 验证配置
    if (_debug) {
        uint8_t gyro_cfg = read_register(REG_GYRO_CONFIG);
        uint8_t accel_cfg = read_register(REG_ACCEL_CONFIG);
        
        Serial.print("Gyro configuration: 0x");
        Serial.println(gyro_cfg, HEX);
        Serial.print("Accelerometer configuration: 0x");
        Serial.println(accel_cfg, HEX);
    }
    
    _initialized = true;
    
    if (_debug) {
        Serial.println("MPU6500 Initialized");
    }
    
    return true;
}

bool MPU6500::read_sensors() {
    if (!_initialized) {
        if (_debug) Serial.println("MPU6500 not initialized");
        return false;
    }

    SPI.setDataMode(_mpu6500_spi_mode); //切换SPI模式
    
    uint8_t buffer[14];  // 加速度计(6) + 温度(2) + 陀螺仪(6)
    
    // 一次性读取所有传感器数据
    read_registers(REG_ACCEL_XOUT_H, buffer, 14);
    
    // 解析加速度计数据
    int16_t accel_x = (buffer[0] << 8) | buffer[1];
    int16_t accel_y = (buffer[2] << 8) | buffer[3];
    int16_t accel_z = (buffer[4] << 8) | buffer[5];
    
    // 转换为g（±8g量程：4096 LSB/g）
    _accel_x = accel_x / 4096.0f;
    _accel_y = accel_y / 4096.0f;
    _accel_z = accel_z / 4096.0f;
    
    // 解析温度数据
    int16_t temp = (buffer[6] << 8) | buffer[7];
    _temperature = (temp / 340.0f) + 36.53f;
    
    // 解析陀螺仪数据
    int16_t gyro_x = (buffer[8] << 8) | buffer[9];
    int16_t gyro_y = (buffer[10] << 8) | buffer[11];
    int16_t gyro_z = (buffer[12] << 8) | buffer[13];
    
    // 转换为dps（±2000dps量程：16.4 LSB/dps）
    _gyro_x = gyro_x / 16.4f;
    _gyro_y = gyro_y / 16.4f;
    _gyro_z = gyro_z / 16.4f;
    
    return true;
}