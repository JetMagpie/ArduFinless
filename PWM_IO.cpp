#include "PWM_IO.h"
#include <avr/interrupt.h>
#include <util/atomic.h>

// 初始化静态实例指针
PWM_IO* PWM_IO::instance = nullptr;

// 构造函数
PWM_IO::PWM_IO() {
    // 初始化输入相关变量
    ch1_rise_time = 0;
    ch1_pulse_width = DEFAULT_PWM_WIDTH;
    ch1_valid = false;
    ch1_last_update = 0;
    
    ch2_rise_time = 0;
    ch2_pulse_width = DEFAULT_PWM_WIDTH;
    ch2_valid = false;
    ch2_last_update = 0;
    
    ch3_rise_time = 0;
    ch3_pulse_width = DEFAULT_PWM_WIDTH;
    ch3_valid = false;
    ch3_last_update = 0;
    
    ch4_rise_time = 0;
    ch4_pulse_width = DEFAULT_PWM_WIDTH;
    ch4_valid = false;
    ch4_last_update = 0;
    
    // 初始化输出相关变量为默认值
    ch1_out_width = DEFAULT_PWM_WIDTH;
    ch2_out_width = DEFAULT_PWM_WIDTH;
    ch3_out_width = DEFAULT_PWM_WIDTH;
    ch4_out_width = DEFAULT_PWM_WIDTH;
    
    // 设置实例指针
    instance = this;
}

// 初始化函数
void PWM_IO::begin() {
    // 配置输入引脚为输入模式，启用上拉电阻
    pinMode(CH1_IN_PIN, INPUT);
    pinMode(CH2_IN_PIN, INPUT);
    pinMode(CH3_IN_PIN, INPUT);
    pinMode(CH4_IN_PIN, INPUT);
    
    // 配置输出引脚
    pinMode(CH1_OUT_PIN, OUTPUT);
    pinMode(CH2_OUT_PIN, OUTPUT);
    pinMode(CH3_OUT_PIN, OUTPUT);
    pinMode(CH4_OUT_PIN, OUTPUT);
    
    // 配置定时器
    configureTimer1();
    configureTimer4();
    
    // 配置中断（使用CHANGE模式）
    // CH1 (Pin 2) - 外部中断0
    attachInterrupt(digitalPinToInterrupt(CH1_IN_PIN), isrCH1_Change, CHANGE);
    
    // CH2 (Pin 3) - 外部中断1
    attachInterrupt(digitalPinToInterrupt(CH2_IN_PIN), isrCH2_Change, CHANGE);
    
    // CH3 (Pin 18) - 外部中断5
    attachInterrupt(digitalPinToInterrupt(CH3_IN_PIN), isrCH3_Change, CHANGE);
    
    // CH4 (Pin 19) - 外部中断4
    attachInterrupt(digitalPinToInterrupt(CH4_IN_PIN), isrCH4_Change, CHANGE);
    
    // 记录初始时间
    uint32_t now = millis();
    ch1_last_update = now;
    ch2_last_update = now;
    ch3_last_update = now;
    ch4_last_update = now;
    
    // 确保中断使能
    sei();
}

// 配置Timer1用于输出50Hz PWM（引脚11、12）
void PWM_IO::configureTimer1() {
    cli();
    
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    
    // 模式14：快速PWM，TOP=ICR1
    // COM1A1:1 COM1B1:1 - 非反相模式，引脚11和12输出PWM
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // 预分频8
    
    // 舵机需要10ms周期，使用0.5us分辨率：10000us / 0.5us = 20000
    ICR1 = 20000;  // TOP值，对应10ms周期
    
    // 初始占空比设置为0（舵机不输出）
    OCR1A = 0xFFFF;  // 引脚12
    OCR1B = 0xFFFF;  // 引脚11
    
    sei();
}

// 配置Timer4用于输出50Hz PWM（引脚7、8）
void PWM_IO::configureTimer4() {
    cli();
    
    TCCR4A = 0;
    TCCR4B = 0;
    TCNT4 = 0;
    
    // 模式14：快速PWM，TOP=ICR4
    TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM41);
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41);  // 预分频8
    
    ICR4 = 20000;  // TOP值，对应10ms周期
    
    // 初始占空比设置为0（舵机不输出）
    OCR4A = 0xFFFF;  // 引脚6（CH5，未使用）
    OCR4B = 0xFFFF;  // 引脚7（CH4）
    OCR4C = 0xFFFF;  // 引脚8（CH3）
    
    sei();
}

// 约束脉冲宽度到800-2200μs范围
uint16_t PWM_IO::constrainPulse(uint16_t pulse_us) {
    if (pulse_us < MIN_PWM_WIDTH) return MIN_PWM_WIDTH;
    if (pulse_us > MAX_PWM_WIDTH) return MAX_PWM_WIDTH;
    return pulse_us;
}

// 将脉冲宽度（μs）转换为OCR值（0.5μs单位）
uint16_t PWM_IO::pulseToOCR(uint16_t pulse_us) {
    // 约束范围
    pulse_us = constrainPulse(pulse_us);
    
    // 转换为0.5us单位：pulse_us * 2
    return pulse_us << 1;
}

// 设置输出PWM宽度
void PWM_IO::setCH1_OUT(uint16_t highTime_us) {
    ch1_out_width = constrainPulse(highTime_us);
    uint16_t ocr_value = pulseToOCR(ch1_out_width);
    
    cli();
    OCR1B = ocr_value;  // 引脚11，定时器1B
    sei();
}

void PWM_IO::setCH2_OUT(uint16_t highTime_us) {
    ch2_out_width = constrainPulse(highTime_us);
    uint16_t ocr_value = pulseToOCR(ch2_out_width);
    
    cli();
    OCR1A = ocr_value;  // 引脚12，定时器1A
    sei();
}

void PWM_IO::setCH3_OUT(uint16_t highTime_us) {
    ch3_out_width = constrainPulse(highTime_us);
    uint16_t ocr_value = pulseToOCR(ch3_out_width);
    
    cli();
    OCR4C = ocr_value;  // 引脚8，定时器4C
    sei();
}

void PWM_IO::setCH4_OUT(uint16_t highTime_us) {
    ch4_out_width = constrainPulse(highTime_us);
    uint16_t ocr_value = pulseToOCR(ch4_out_width);
    
    cli();
    OCR4B = ocr_value;  // 引脚7，定时器4B
    sei();
}

// 启用输出通道
void PWM_IO::enableOutput(uint8_t channel) {
    switch(channel) {
        case 1:  // CH1 (引脚11)
            TCCR1A |= (1 << COM1B1);
            break;
        case 2:  // CH2 (引脚12)
            TCCR1A |= (1 << COM1A1);
            break;
        case 3:  // CH3 (引脚8)
            TCCR4A |= (1 << COM4C1);
            break;
        case 4:  // CH4 (引脚7)
            TCCR4A |= (1 << COM4B1);
            break;
    }
}

// 禁用输出通道
void PWM_IO::disableOutput(uint8_t channel) {
    switch(channel) {
        case 1:  // CH1 (引脚11)
            TCCR1A &= ~(1 << COM1B1);
            digitalWrite(CH1_OUT_PIN, LOW);
            break;
        case 2:  // CH2 (引脚12)
            TCCR1A &= ~(1 << COM1A1);
            digitalWrite(CH2_OUT_PIN, LOW);
            break;
        case 3:  // CH3 (引脚8)
            TCCR4A &= ~(1 << COM4C1);
            digitalWrite(CH3_OUT_PIN, LOW);
            break;
        case 4:  // CH4 (引脚7)
            TCCR4A &= ~(1 << COM4B1);
            digitalWrite(CH4_OUT_PIN, LOW);
            break;
    }
}

// 中断服务函数实现（CHANGE模式）
void PWM_IO::isrCH1_Change() {
    if (instance) {
        if (digitalRead(CH1_IN_PIN)) {
            instance->ch1_rise_time = micros();
        } else {
            uint32_t pulse_width = micros() - instance->ch1_rise_time;
            if (pulse_width >= MIN_PWM_WIDTH && pulse_width <= MAX_PWM_WIDTH) {
                instance->ch1_pulse_width = pulse_width;
                instance->ch1_last_update = millis();
                instance->ch1_valid = true;
            }
        }
    }
}

void PWM_IO::isrCH2_Change() {
    if (instance) {
        if (digitalRead(CH2_IN_PIN)) {
            instance->ch2_rise_time = micros();
        } else {
            uint32_t pulse_width = micros() - instance->ch2_rise_time;
            if (pulse_width >= MIN_PWM_WIDTH && pulse_width <= MAX_PWM_WIDTH) {
                instance->ch2_pulse_width = pulse_width;
                instance->ch2_last_update = millis();
                instance->ch2_valid = true;
            }
        }
    }
}

void PWM_IO::isrCH3_Change() {
    if (instance) {
        if (digitalRead(CH3_IN_PIN)) {
            instance->ch3_rise_time = micros();
        } else {
            uint32_t pulse_width = micros() - instance->ch3_rise_time;
            if (pulse_width >= MIN_PWM_WIDTH && pulse_width <= MAX_PWM_WIDTH) {
                instance->ch3_pulse_width = pulse_width;
                instance->ch3_last_update = millis();
                instance->ch3_valid = true;
            }
        }
    }
}

void PWM_IO::isrCH4_Change() {
    if (instance) {
        if (digitalRead(CH4_IN_PIN)) {
            instance->ch4_rise_time = micros();
        } else {
            uint32_t pulse_width = micros() - instance->ch4_rise_time;
            if (pulse_width >= MIN_PWM_WIDTH && pulse_width <= MAX_PWM_WIDTH) {
                instance->ch4_pulse_width = pulse_width;
                instance->ch4_last_update = millis();
                instance->ch4_valid = true;
            }
        }
    }
}

// 检查超时（需要在loop中调用）
void PWM_IO::checkTimeouts() {
    uint32_t now = millis();
    
    // 检查CH1是否超时
    if (now - ch1_last_update > TIMEOUT_MS) {
        ch1_valid = false;
        ch1_pulse_width = DEFAULT_PWM_WIDTH;
    }
    
    // 检查CH2是否超时
    if (now - ch2_last_update > TIMEOUT_MS) {
        ch2_valid = false;
        ch2_pulse_width = DEFAULT_PWM_WIDTH;
    }
    
    // 检查CH3是否超时
    if (now - ch3_last_update > TIMEOUT_MS) {
        ch3_valid = false;
        ch3_pulse_width = DEFAULT_PWM_WIDTH;
    }
    
    // 检查CH4是否超时
    if (now - ch4_last_update > TIMEOUT_MS) {
        ch4_valid = false;
        ch4_pulse_width = DEFAULT_PWM_WIDTH;
    }
}

// 获取输入PWM宽度
uint16_t PWM_IO::getCH1_IN() const {
    uint16_t val;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        val = ch1_pulse_width;
    }
    return val;
}

uint16_t PWM_IO::getCH2_IN() const {
    uint16_t val;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        val = ch2_pulse_width;
    }
    return val;
}

uint16_t PWM_IO::getCH3_IN() const {
    uint16_t val;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    val = ch3_pulse_width;
    }
    return val;
}

uint16_t PWM_IO::getCH4_IN() const {
    uint16_t val;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        val = ch4_pulse_width;
    }
    return val;
}

// 检查输入是否有效
bool PWM_IO::isCH1_Valid() const {
    return ch1_valid;
}

bool PWM_IO::isCH2_Valid() const {
    return ch2_valid;
}

bool PWM_IO::isCH3_Valid() const {
    return ch3_valid;
}

bool PWM_IO::isCH4_Valid() const {
    return ch4_valid;
}

// 重置输入通道为默认值
void PWM_IO::resetCH1_IN() {
    ch1_pulse_width = DEFAULT_PWM_WIDTH;
    ch1_valid = false;
    ch1_last_update = millis();
}

void PWM_IO::resetCH2_IN() {
    ch2_pulse_width = DEFAULT_PWM_WIDTH;
    ch2_valid = false;
    ch2_last_update = millis();
}

void PWM_IO::resetCH3_IN() {
    ch3_pulse_width = DEFAULT_PWM_WIDTH;
    ch3_valid = false;
    ch3_last_update = millis();
}

void PWM_IO::resetCH4_IN() {
    ch4_pulse_width = DEFAULT_PWM_WIDTH;
    ch4_valid = false;
    ch4_last_update = millis();
}

// 设置所有输出为默认值1500
void PWM_IO::resetAllOutputs() {
    setCH1_OUT(DEFAULT_PWM_WIDTH);
    setCH2_OUT(DEFAULT_PWM_WIDTH);
    setCH3_OUT(DEFAULT_PWM_WIDTH);
    setCH4_OUT(DEFAULT_PWM_WIDTH);
}