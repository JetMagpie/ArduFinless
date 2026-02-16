#ifndef PWM_IO_H
#define PWM_IO_H

#include <Arduino.h>

// PWM输入通道定义
#define CH1_IN_PIN 2    // 外部中断0
#define CH2_IN_PIN 3    // 外部中断1
#define CH3_IN_PIN 18   // 外部中断5
#define CH4_IN_PIN 19   // 外部中断4

// PWM输出通道定义
#define CH1_OUT_PIN 11  // 定时器1B（引脚11）
#define CH2_OUT_PIN 12  // 定时器1A（引脚12）
#define CH3_OUT_PIN 8   // 定时器4C（引脚8）
#define CH4_OUT_PIN 7   // 定时器4B（引脚7）

// PWM频率定义（100Hz用于舵机）
#define PWM_FREQ 100     // 100Hz
#define PWM_PERIOD_US 10000  // 100Hz对应的周期10000μs

// 默认值定义（1500μs，舵机中位）
#define DEFAULT_PWM_WIDTH 1500  // 默认1500μs

// 舵机PWM范围定义
#define MIN_PWM_WIDTH 800
#define MAX_PWM_WIDTH 2200

class PWM_IO {
public:
    // 构造函数
    PWM_IO();
    
    // 初始化函数
    void begin();
    
    // 获取PWM输入高电平宽度（μs），无效时返回1500
    uint16_t getCH1_IN() const;
    uint16_t getCH2_IN() const;
    uint16_t getCH3_IN() const;
    uint16_t getCH4_IN() const;
    
    // 设置PWM输出高电平宽度（μs）
    void setCH1_OUT(uint16_t highTime_us);
    void setCH2_OUT(uint16_t highTime_us);
    void setCH3_OUT(uint16_t highTime_us);
    void setCH4_OUT(uint16_t highTime_us);
    
    // 检查输入是否有效
    bool isCH1_Valid() const;
    bool isCH2_Valid() const;
    bool isCH3_Valid() const;
    bool isCH4_Valid() const;
    
    // 重置输入通道为默认值1500
    void resetCH1_IN();
    void resetCH2_IN();
    void resetCH3_IN();
    void resetCH4_IN();
    
    // 设置所有输出为默认值1500
    void resetAllOutputs();
    
    // 超时检查函数（在loop中调用）
    void checkTimeouts();
    
    // 获取默认值
    uint16_t getDefaultWidth() const { return DEFAULT_PWM_WIDTH; }
    
    // 启用/禁用输出通道
    void enableOutput(uint8_t channel);
    void disableOutput(uint8_t channel);
    
private:
    // 输入相关变量
    volatile uint32_t ch1_rise_time;
    volatile uint16_t ch1_pulse_width;
    volatile bool ch1_valid;
    volatile uint32_t ch1_last_update;
    
    volatile uint32_t ch2_rise_time;
    volatile uint16_t ch2_pulse_width;
    volatile bool ch2_valid;
    volatile uint32_t ch2_last_update;
    
    volatile uint32_t ch3_rise_time;
    volatile uint16_t ch3_pulse_width;
    volatile bool ch3_valid;
    volatile uint32_t ch3_last_update;
    
    volatile uint32_t ch4_rise_time;
    volatile uint16_t ch4_pulse_width;
    volatile bool ch4_valid;
    volatile uint32_t ch4_last_update;
    
    // 输出相关变量
    uint16_t ch1_out_width;
    uint16_t ch2_out_width;
    uint16_t ch3_out_width;
    uint16_t ch4_out_width;
    
    // 定时器配置
    void configureTimer1();
    void configureTimer4();
    
    // 中断服务函数（CHANGE模式）
    static void isrCH1_Change();
    static void isrCH2_Change();
    static void isrCH3_Change();
    static void isrCH4_Change();
    
    // 辅助函数
    uint16_t constrainPulse(uint16_t pulse_us);
    uint16_t pulseToOCR(uint16_t pulse_us);
    
    // 超时时间定义（200ms没有新信号则认为无效）
    static const uint32_t TIMEOUT_MS = 200;
    
    // 全局实例指针
    static PWM_IO* instance;
};

#endif