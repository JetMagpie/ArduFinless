/*
 * 主程序：阻力舵飞翼控制程序
 * 功能：
 * 1. 读取AS5047P磁编码器角度并处理过零点
 * 2. 读取MPU6500陀螺仪Z轴角速度
 * 3. 读取PWM输入并归一化
 * 4. 所有传感器输入经过低通滤波
 * 5. 根据阻力舵控制律计算右左输出值
 * 6. 输出PWM控制信号
 */

#include <Arduino.h>
#include <SPI.h>
#include "PWM_IO.h"
#include "MPU6500.h"
#include "AS5047P.h" 
#include "LowPassFilter.h"

// ==================== 引脚定义 ====================
#define ENCODER_CS_PIN 49  // AS5047P的CS引脚
#define IMU_CS_PIN 48      // MPU6500的CS引脚

// ==================== 全局对象 ====================
PWM_IO pwmIO;
MPU6500 imu(IMU_CS_PIN, false); // 设置为false关闭调试输出
AS5047P encoder(ENCODER_CS_PIN);      // 使用AS5047P库

// ==================== 低通滤波器实例 ====================
// 滤波器参数
const float SAMPLE_TIME = 0.01f;  // 采样时间10ms (100Hz)
float beta_cutoff_freq = 20.0f;    // 角度截止频率 (Hz)
float gyro_cutoff_freq = 20.0f;   // 角速度截止频率 (Hz)
float yaw_cutoff_freq = 80.0f;     // Yaw输入截止频率 (Hz)
float brake_cutoff_freq = 80.0f;   // Brake输入截止频率 (Hz)

// 创建滤波器实例
LowPassFilter beta_filter(beta_cutoff_freq, SAMPLE_TIME);
LowPassFilter gyro_filter(gyro_cutoff_freq, SAMPLE_TIME);
LowPassFilter yaw_filter(yaw_cutoff_freq, SAMPLE_TIME);
LowPassFilter brake_filter(brake_cutoff_freq, SAMPLE_TIME);

// ==================== 校准参数 ====================
float encoder_calibration = -217.5f;  // 编码器角度校准量（度）
float gyro_calibration = 1.4f;     // 陀螺仪角速度校准量（度/秒）

// ==================== 控制参数 ====================
float Yaw_gain = 1.0f;     // 摇杆增益
float Cn_beta = 6.0f;      // 角度反馈增益
float Cn_damper = 1.0f;    // 阻尼增益

// ==================== 输出限制 ====================
// 右输出PWM限制（单位：微秒）
uint16_t right_lim_inf = 1520;  // 最小脉冲宽度
uint16_t right_lim_sup = 1980;  // 最大脉冲宽度

// 左输出PWM限制（单位：微秒）
uint16_t left_lim_inf = 980;  // 最小脉冲宽度
uint16_t left_lim_sup = 1200;  // 最大脉冲宽度

// ==================== 函数声明 ====================
float readEncoderAngle(float calibration);
float readGyroRate(float calibration);
void readPWMInputs(float& Yaw, float& Brake);
void calculateOutputs(float Yaw, float Brake, float Beta, float Yaw_Rate, 
                     float Yaw_gain, float Cn_beta, float Cn_damper,
                     float& OUT_right, float& OUT_left);
void setPWMOutputs(float OUT_right, float OUT_left,
                   uint16_t right_lim_inf, uint16_t right_lim_sup,
                   uint16_t left_lim_inf, uint16_t left_lim_sup);
float normalizeAngle(float angle_deg);
void printDebugInfo(float Beta, float Beta_filtered, float Yaw_Rate, float Yaw_Rate_filtered,
                    float Yaw, float Yaw_filtered, float Brake, float Brake_filtered,
                    float OUT_right, float OUT_left);

// ==================== 主程序 ====================

void setup() {
  // 初始化串口通信
  Serial.begin(115200);
 
  // 初始化PWM输入输出
  pwmIO.begin();
  Serial.println("PWM IO initialized");
  
  // 初始化SPI
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128);  // 设置100kHz时钟
  
  // 初始化磁编码器
  if (encoder.init()) {
    Serial.println("AS5047P initialized");
  } else {
    Serial.println("AS5047P initialization failed");
  }
  
  // 初始化陀螺仪
  if (imu.init()) {
    Serial.println("MPU6500 initialized");
  } else {
    Serial.println("MPU6500 initialization failed");
  }
  
  // 启用所有PWM输出通道
  pwmIO.enableOutput(1);
  pwmIO.enableOutput(2);
  pwmIO.enableOutput(3);
  pwmIO.enableOutput(4);
  
  Serial.println("System initialized");
  Serial.println("===================================");
  Serial.println("Beta | Yaw_Rate | Yaw | Brake | OUT_right | OUT_left");
  Serial.println("----------------------------------------------------");
  delay(1000);
}

void loop() {
  static uint32_t last_print_time = 0;
  static uint32_t last_update_time = 0;
  
  //检查PWM输入超时
  pwmIO.checkTimeouts();
  
  //读取编码器原始角度（Beta）
  float Beta_raw = readEncoderAngle(encoder_calibration);
  
  // 读取陀螺仪原始角速度（Yaw_Rate）
  float Yaw_Rate_raw = readGyroRate(gyro_calibration);
  
  // 读取PWM原始输入（Yaw和Brake）
  float Yaw_raw = 0.0f, Brake_raw = 0.0f;
  readPWMInputs(Yaw_raw, Brake_raw);
  
  // 应用低通滤波
  float Beta = beta_filter.update(Beta_raw);
  float Yaw_Rate = gyro_filter.update(Yaw_Rate_raw);
  float Yaw = /*yaw_filter.update(*/Yaw_raw/*)*/;
  float Brake = brake_filter.update(Brake_raw);
  
  //计算输出值
  float OUT_right = 0.0f, OUT_left = 0.0f;
  calculateOutputs(Yaw, Brake, Beta, Yaw_Rate, 
                  Yaw_gain, Cn_beta, Cn_damper,
                  OUT_right, OUT_left);
  
  //设置PWM输出
  setPWMOutputs(OUT_right, OUT_left,
                right_lim_inf, right_lim_sup,
                left_lim_inf, left_lim_sup);
 
  // 控制循环频率（100Hz）
  uint32_t current_time = millis();
  unsigned long delay_time = min(10,max(0,10-(current_time-last_update_time)));
  delay(delay_time);
  last_update_time = current_time;

  // 定期打印调试信息
  if (current_time - last_print_time >= 50) { // 每50ms打印一次
    printDebugInfo(Beta_raw, Beta, Yaw_Rate_raw, Yaw_Rate,Yaw_raw, Yaw, Brake_raw, Brake,OUT_right, OUT_left);
    last_print_time = current_time;
  }
}

// ==================== 功能函数实现 ====================

/**
 * @brief 读取编码器角度并处理过零点
 * @param calibration 角度校准量（度）
 * @return 处理后的角度（-180到+180度）
 */
float readEncoderAngle(float calibration) {
  if (encoder.read_angle()) {
    float angle_deg = encoder.get_angle_deg();
    
    // 加上校准量
    angle_deg += calibration;
    
    // 处理过零点，确保角度在[-180, 180]范围
    angle_deg = normalizeAngle(angle_deg);
    
    return angle_deg;
  }
  
  // 读取失败时返回0
  Serial.println("AS5047P Error");
  return 0.0f;
}

/**
 * @brief 读取陀螺仪Z轴角速度
 * @param calibration 角速度校准量（度/秒）
 * @return 处理后的角速度（度/秒）
 */
float readGyroRate(float calibration) {
  if (imu.read_sensors()) {
    float gyro_z = imu.get_gyro_z();
    
    // 加上校准量
    gyro_z += calibration;
    
    return gyro_z;
  }
  
  // 读取失败时返回0
  Serial.println("MPU6500 Error");
  return 0.0f;
}

/**
 * @brief 读取PWM输入并归一化
 * @param Yaw 输出：Yaw通道归一化值[-1, 1]
 * @param Brake 输出：Brake通道归一化值[-1, 1]
 */
void readPWMInputs(float& Yaw, float& Brake) {
  // 读取CH1（Yaw）
  if (pwmIO.isCH1_Valid()) {
    uint16_t pulse_width1 = pwmIO.getCH1_IN();
    // 归一化：800-2200 -> -1到1，1500为零点
    Yaw = (pulse_width1 - 1500.0f) / 350.0f;
    // 限制在[-1, 1]范围
    Yaw = constrain(Yaw, -1.0f, 1.0f);
  } else {
    Yaw = 0.0f; // 输入无效时返回0
  }
  
  // 读取CH2（Brake）
  if (pwmIO.isCH2_Valid()) {
    uint16_t pulse_width2 = pwmIO.getCH2_IN();
    // 归一化：800-2200 -> -1到1，1500为零点
    Brake = (pulse_width2 - 1500.0f) / 350.0f;
    // 限制在[-1, 1]范围
    Brake = constrain(Brake, -1.0f, 1.0f);
  } else {
    Brake = 0.0f; // 输入无效时返回0
  }
}

/**
 * @brief 计算输出值
 * @param Yaw 摇杆输入[-1, 1]
 * @param Brake 刹车输入[-1, 1]
 * @param Beta 角度输入（度）
 * @param Yaw_Rate 角速度输入（度/秒）
 * @param Yaw_gain 摇杆增益
 * @param Cn_beta 角度反馈增益
 * @param Cn_damper 阻尼增益
 * @param OUT_right 输出：右输出值[0, 1]
 * @param OUT_left 输出：左输出值[0, 1]
 */
void calculateOutputs(float Yaw, float Brake, float Beta, float Yaw_Rate, 
                     float Yaw_gain, float Cn_beta, float Cn_damper,
                     float& OUT_right, float& OUT_left) {
  
  //对Beta进行归一化：-180到180 -> -1到1
  float Beta_norm = Beta / 180.0f;
  Beta_norm = constrain(Beta_norm, -1.0f, 1.0f);
  
  //对Yaw_Rate进行归一化：-180到180度/秒 -> -1到1
  float Yaw_Rate_norm = Yaw_Rate / 180.0f;
  Yaw_Rate_norm = constrain(Yaw_Rate_norm, -1.0f, 1.0f);
  
  //计算控制项
  float control_term = - Yaw * Yaw_gain - Beta_norm * Cn_beta + Yaw_Rate_norm * Cn_damper;
  
  //限制控制项在[-1, 1]范围
  float control_clamped = constrain(control_term,-1.0f,1.0f);
  
  //处理刹车项
  float brake_clamped = constrain(Brake,0.0f,1.0f);
  
  //计算输出
  OUT_left = constrain(-control_clamped + brake_clamped,0.0f,1.0f);
  OUT_right = constrain(control_clamped + brake_clamped,0.0f,1.0f);
}

/**
 * @brief 设置PWM输出
 * @param OUT_right 右输出值[0, 1]
 * @param OUT_left 左输出值[0, 1]
 * @param right_lim_inf 右输出下限（微秒）
 * @param right_lim_sup 右输出上限（微秒）
 * @param left_lim_inf 左输出下限（微秒）
 * @param left_lim_sup 左输出上限（微秒）
 */
void setPWMOutputs(float OUT_right, float OUT_left,
                   uint16_t right_lim_inf, uint16_t right_lim_sup,
                   uint16_t left_lim_inf, uint16_t left_lim_sup) {
  
  //确保输出值在[0, 1]范围
  OUT_right = constrain(OUT_right,0.0f,1.0f);
  OUT_left = constrain(OUT_left,0.0f,1.0f);
  
  //映射到PWM脉冲宽度
  uint16_t right_pulse = map(OUT_right * 1000, 1000, 0, right_lim_inf, right_lim_sup);
  uint16_t left_pulse = map(OUT_left * 1000, 0, 1000, left_lim_inf, left_lim_sup);
  
  // 设置PWM输出
  pwmIO.setCH3_OUT(right_pulse);  // CH3输出右信号
  pwmIO.setCH4_OUT(left_pulse); // CH4输出左信号
}

/**
 * @brief 将角度归一化到[-180, 180]范围
 * @param angle_deg 输入角度（度）
 * @return 归一化后的角度（-180到+180度）
 */
float normalizeAngle(float angle_deg) {
  //先将角度转换到[0, 360)范围
  angle_deg = fmod(angle_deg, 360.0f);
  if (angle_deg < 0) {
    angle_deg += 360.0f;
  }
  
  //转换到[-180, 180]范围
  if (angle_deg > 180.0f) {
    angle_deg -= 360.0f;
  }
  
  return angle_deg;
}

/**
 * @brief 打印调试信息到串口
 */
void printDebugInfo(float Beta_raw, float Beta_filtered, float Yaw_Rate_raw, float Yaw_Rate_filtered,
                    float Yaw_raw, float Yaw_filtered, float Brake_raw, float Brake_filtered,
                    float OUT_right, float OUT_left) {
  Serial.print("Beta: ");
  Serial.print(Beta_raw, 1);
  Serial.print("(");
  Serial.print(Beta_filtered, 1);
  Serial.print(") | Yaw_Rate: ");
  Serial.print(Yaw_Rate_raw, 2);
  Serial.print("(");
  Serial.print(Yaw_Rate_filtered, 2);
  Serial.print(") | Yaw: ");
  Serial.print(Yaw_raw, 3);
  Serial.print("(");
  Serial.print(Yaw_filtered, 3);
  Serial.print(") | Brake: ");
  Serial.print(Brake_raw, 3);
  Serial.print("(");
  Serial.print(Brake_filtered, 3);
  Serial.print(") | L_OUT: ");
  Serial.print(OUT_right, 3);
  Serial.print(" | R_OUT: ");
  Serial.print(OUT_left, 3);
  Serial.println();
}