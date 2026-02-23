/*
 * 主程序：阻力舵飞翼控制程序（带串口调参与EEPROM存储）
 * 功能：
 * 1. 读取AS5047P磁编码器角度并处理过零点
 * 2. 读取MPU6500陀螺仪Z轴角速度
 * 3. 读取PWM输入并归一化
 * 4. 所有传感器输入经过低通滤波
 * 5. 根据阻力舵控制律计算右左输出值
 * 6. 输出PWM控制信号
 * 7. 串口调参：report/get/set/help/params
 * 8. 参数自动保存到EEPROM（启动时加载）
 */

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include "PWM_IO.h"
#include "MPU6500.h"
#include "AS5047P.h" 
#include "LowPassFilter.h"

// ==================== 引脚定义 ====================
#define ENCODER_CS_PIN 49  // AS5047P的CS引脚
#define IMU_CS_PIN 48      // MPU6500的CS引脚

// ==================== 全局对象 ====================
PWM_IO pwmIO;
MPU6500 imu(IMU_CS_PIN, false); // 使用MPU6500库
AS5047P encoder(ENCODER_CS_PIN); // 使用AS5047P库

// ==================== 低通滤波器实例 ====================
const float SAMPLE_TIME = 0.01f;       // 采样时间10ms (100Hz)
float beta_cutoff_freq = 50.0f;        // 角度截止频率 (Hz)
float gyro_cutoff_freq = 50.0f;        // 角速度截止频率 (Hz)
float yaw_cutoff_freq = 80.0f;         // Yaw输入截止频率 (Hz)
float brake_cutoff_freq = 80.0f;       // Brake输入截止频率 (Hz)

LowPassFilter beta_filter(beta_cutoff_freq, SAMPLE_TIME);
LowPassFilter gyro_filter(gyro_cutoff_freq, SAMPLE_TIME);
LowPassFilter yaw_filter(yaw_cutoff_freq, SAMPLE_TIME);
LowPassFilter brake_filter(brake_cutoff_freq, SAMPLE_TIME);

// ==================== 可调参数（将存入EEPROM）====================
// 这些全局变量将在加载EEPROM后被赋值
uint16_t right_lim_inf = 1600;   // 右输出最小脉冲宽度（微秒）
uint16_t right_lim_sup = 1800;   // 右输出最大脉冲宽度（微秒）
uint16_t left_lim_inf = 1200;     // 左输出最小脉冲宽度（微秒）
uint16_t left_lim_sup = 1400;    // 左输出最大脉冲宽度（微秒）
float encoder_calibration = 0.0f;  // 编码器角度校准量（度）
float gyro_calibration = 0.0f;       // 陀螺仪角速度校准量（度/秒）
float Cn_beta = 6.0f;                 // 角度反馈增益
float Cn_damper = 1.0f;               // 阻尼增益

// 固定参数
float Yaw_gain = 1.0f;                 // 摇杆增益

// ==================== EEPROM 存储相关 ====================
#define EEPROM_MAGIC 0xDEADBEEF        // 用于校验EEPROM是否有效

// 参数结构体（紧凑排列，无填充）
struct SystemParams {
    uint16_t right_lim_inf;
    uint16_t right_lim_sup;
    uint16_t left_lim_inf;
    uint16_t left_lim_sup;
    float encoder_calibration;
    float gyro_calibration;
    float Cn_beta;
    float Cn_damper;
} __attribute__((packed));

// 配置结构体（包含魔法数）
struct Config {
    uint32_t magic;
    SystemParams params;
} __attribute__((packed));

// 参数名称与范围（用于串口调参）
const char* param_names[] = {
    "right_lim_inf",
    "right_lim_sup", 
    "left_lim_inf",
    "left_lim_sup",
    "encoder_calibration",
    "gyro_calibration",
    "Cn_beta",
    "Cn_damper"
};

struct ParamRange {
    float min;
    float max;
};

const ParamRange param_ranges[] = {
    {800, 2200},   // right_lim_inf
    {800, 2200},   // right_lim_sup
    {800, 2200},   // left_lim_inf
    {800, 2200},   // left_lim_sup
    {-360.0, 360.0},     // encoder_calibration
    {-10.0, 10.0},       // gyro_calibration
    {-50.0, 50.0},       // Cn_beta
    {-50.0, 50.0}        // Cn_damper
};
const int NUM_PARAMS = sizeof(param_names) / sizeof(char*);

// 默认参数值（与初始化值一致）
const SystemParams DEFAULT_PARAMS = {
    1600,   // right_lim_inf
    1800,   // right_lim_sup
    1200,    // left_lim_inf
    1400,   // left_lim_sup
    0.0f, // encoder_calibration
    0.0f,    // gyro_calibration
    6.0f,    // Cn_beta
    1.0f     // Cn_damper
};

// 串口命令相关
volatile bool report_enabled = false;   // 默认关闭调试打印
char serial_buffer[64];
uint8_t serial_index = 0;

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
void printDebugInfo(float Beta_raw, float Beta_filtered, float Yaw_Rate_raw, float Yaw_Rate_filtered,
                    float Yaw_raw, float Yaw_filtered, float Brake_raw, float Brake_filtered,
                    float OUT_right, float OUT_left);

// 新增串口与EEPROM函数
void loadParamsFromEEPROM();
void saveParamsToEEPROM();
void processSerialInput();
void processSerialCommand();
bool validateParam(int index, float value);
void printAllParams();

// ==================== 主程序 ====================

void setup() {
  // 初始化串口通信
  Serial.begin(115200);
  delay(100);
  
  // 从EEPROM加载参数（会更新全局变量）
  loadParamsFromEEPROM();
 
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
  Serial.println("Serial commands: report on/off, get <param>, set <param> <value>, help, params");
  Serial.println("Data reporting is OFF by default. Use 'report on' to enable.");
  Serial.println("===================================");
  delay(1000);
}

void loop() {
  static uint32_t last_print_time = 0;
  static uint32_t last_update_time = 0;
  
  // 处理串口输入（调参命令）
  processSerialInput();
  
  // 检查PWM输入超时
  pwmIO.checkTimeouts();
  
  // 读取编码器原始角度（Beta）
  float Beta_raw = readEncoderAngle(encoder_calibration);
  
  // 读取陀螺仪原始角速度（Yaw_Rate）
  float Yaw_Rate_raw = readGyroRate(gyro_calibration);
  
  // 读取PWM原始输入（Yaw和Brake）
  float Yaw_raw = 0.0f, Brake_raw = 0.0f;
  readPWMInputs(Yaw_raw, Brake_raw);
  
  // 应用低通滤波
  float Beta = beta_filter.update(Beta_raw);
  float Yaw_Rate = gyro_filter.update(Yaw_Rate_raw);
  float Yaw = Yaw_raw;  // Yaw输入暂不过滤（保留原程序注释状态）
  float Brake = brake_filter.update(Brake_raw);
  
  // 计算输出值
  static float OUT_right = 0.0f;
  static float OUT_left = 0.0f;
  calculateOutputs(Yaw, Brake, Beta, Yaw_Rate, 
                  Yaw_gain, Cn_beta, Cn_damper,
                  OUT_right, OUT_left);
  
  // 设置PWM输出（使用当前全局限幅参数）
  setPWMOutputs(OUT_right, OUT_left,
                right_lim_inf, right_lim_sup,
                left_lim_inf, left_lim_sup);
 
  // 控制循环频率（100Hz）
  uint32_t current_time = millis();
  unsigned long delay_time = min(10, max(0, 10 - (current_time - last_update_time)));
  delay(delay_time);
  last_update_time = current_time;

  // 仅当报告启用时打印调试信息（每50ms）
  if (report_enabled && (current_time - last_print_time >= 50)) {
    printDebugInfo(Beta_raw, Beta, Yaw_Rate_raw, Yaw_Rate,
                   Yaw_raw, Yaw, Brake_raw, Brake,
                   OUT_right, OUT_left);
    last_print_time = current_time;
  }
}

// ==================== 原功能函数实现 ====================

float readEncoderAngle(float calibration) {
  if (encoder.read_angle()) {
    float angle_deg = encoder.get_angle_deg();
    angle_deg += calibration;
    angle_deg = normalizeAngle(angle_deg);
    return angle_deg;
  }
  Serial.println("AS5047P Error");
  return 0.0f;
}

float readGyroRate(float calibration) {
  if (imu.read_sensors()) {
    float gyro_z = imu.get_gyro_z();
    gyro_z += calibration;
    return gyro_z;
  }
  Serial.println("MPU6500 Error");
  return 0.0f;
}

void readPWMInputs(float& Yaw, float& Brake) {
  if (pwmIO.isCH1_Valid()) {
    uint16_t pulse_width1 = pwmIO.getCH1_IN();
    Yaw = (pulse_width1 - 1500.0f) / 350.0f;
    Yaw = constrain(Yaw, -1.0f, 1.0f);
  } else {
    Yaw = 0.0f;
  }
  
  if (pwmIO.isCH2_Valid()) {
    uint16_t pulse_width2 = pwmIO.getCH2_IN();
    Brake = (pulse_width2 - 1500.0f) / 350.0f;
    Brake = constrain(Brake, -1.0f, 1.0f);
  } else {
    Brake = 0.0f;
  }
}

void calculateOutputs(float Yaw, float Brake, float Beta, float Yaw_Rate, 
                     float Yaw_gain, float Cn_beta, float Cn_damper,
                     float& OUT_right, float& OUT_left) {
  float Beta_norm = Beta / 180.0f;
  Beta_norm = constrain(Beta_norm, -1.0f, 1.0f);
  
  float Yaw_Rate_norm = Yaw_Rate / 180.0f;
  Yaw_Rate_norm = constrain(Yaw_Rate_norm, -1.0f, 1.0f);
  
  float control_term = - Yaw * Yaw_gain - Beta_norm * Cn_beta + Yaw_Rate_norm * Cn_damper;
  float control_clamped = constrain(control_term, -1.0f, 1.0f);
  
  float brake_clamped = constrain(Brake, 0.0f, 1.0f);

  OUT_right = constrain(control_clamped + brake_clamped, 0.0f, 1.0f);
  OUT_left = constrain(-control_clamped + brake_clamped, 0.0f, 1.0f);
}

void setPWMOutputs(float OUT_right, float OUT_left,
                   uint16_t right_lim_inf, uint16_t right_lim_sup,
                   uint16_t left_lim_inf, uint16_t left_lim_sup) {
  OUT_right = constrain(OUT_right, 0.0f, 1.0f);
  OUT_left = constrain(OUT_left, 0.0f, 1.0f);
  
  uint16_t right_pulse = map(OUT_right * 1000, 1000, 0, right_lim_inf, right_lim_sup);
  uint16_t left_pulse = map(OUT_left * 1000, 0, 1000, left_lim_inf, left_lim_sup);
  
  pwmIO.setCH3_OUT(right_pulse);
  pwmIO.setCH4_OUT(left_pulse);
}

float normalizeAngle(float angle_deg) {
  angle_deg = fmod(angle_deg, 360.0f);
  if (angle_deg < 0) angle_deg += 360.0f;
  if (angle_deg > 180.0f) angle_deg -= 360.0f;
  return angle_deg;
}

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
  Serial.print(") | R_OUT: ");
  Serial.print(OUT_right, 3);
  Serial.print(" | L_OUT: ");
  Serial.print(OUT_left, 3);
  Serial.println();
}

// ==================== 串口与EEPROM函数 ====================

void loadParamsFromEEPROM() {
  Config config;
  EEPROM.get(0, config);
  
  if (config.magic == EEPROM_MAGIC) {
    bool valid = true;
    
    // 逐个验证参数
    if (config.params.right_lim_inf < param_ranges[0].min || config.params.right_lim_inf > param_ranges[0].max) valid = false;
    if (config.params.right_lim_sup < param_ranges[1].min || config.params.right_lim_sup > param_ranges[1].max) valid = false;
    if (config.params.left_lim_inf < param_ranges[2].min || config.params.left_lim_inf > param_ranges[2].max) valid = false;
    if (config.params.left_lim_sup < param_ranges[3].min || config.params.left_lim_sup > param_ranges[3].max) valid = false;
    if (config.params.encoder_calibration < param_ranges[4].min || config.params.encoder_calibration > param_ranges[4].max) valid = false;
    if (config.params.gyro_calibration < param_ranges[5].min || config.params.gyro_calibration > param_ranges[5].max) valid = false;
    if (config.params.Cn_beta < param_ranges[6].min || config.params.Cn_beta > param_ranges[6].max) valid = false;
    if (config.params.Cn_damper < param_ranges[7].min || config.params.Cn_damper > param_ranges[7].max) valid = false;
    
    if (valid) {
      right_lim_inf = config.params.right_lim_inf;
      right_lim_sup = config.params.right_lim_sup;
      left_lim_inf = config.params.left_lim_inf;
      left_lim_sup = config.params.left_lim_sup;
      encoder_calibration = config.params.encoder_calibration;
      gyro_calibration = config.params.gyro_calibration;
      Cn_beta = config.params.Cn_beta;
      Cn_damper = config.params.Cn_damper;
      
      Serial.println("Parameters loaded from EEPROM");
      return;
    } else {
      Serial.println("EEPROM data out of range, using defaults");
    }
  } else {
    Serial.println("EEPROM magic mismatch, using defaults");
  }
  
  // 使用默认参数并保存
  right_lim_inf = DEFAULT_PARAMS.right_lim_inf;
  right_lim_sup = DEFAULT_PARAMS.right_lim_sup;
  left_lim_inf = DEFAULT_PARAMS.left_lim_inf;
  left_lim_sup = DEFAULT_PARAMS.left_lim_sup;
  encoder_calibration = DEFAULT_PARAMS.encoder_calibration;
  gyro_calibration = DEFAULT_PARAMS.gyro_calibration;
  Cn_beta = DEFAULT_PARAMS.Cn_beta;
  Cn_damper = DEFAULT_PARAMS.Cn_damper;
  
  saveParamsToEEPROM();
}


void saveParamsToEEPROM() {
  Config config;
  config.magic = EEPROM_MAGIC;
  config.params.right_lim_inf = right_lim_inf;
  config.params.right_lim_sup = right_lim_sup;
  config.params.left_lim_inf = left_lim_inf;
  config.params.left_lim_sup = left_lim_sup;
  config.params.encoder_calibration = encoder_calibration;
  config.params.gyro_calibration = gyro_calibration;
  config.params.Cn_beta = Cn_beta;
  config.params.Cn_damper = Cn_damper;
  
  EEPROM.put(0, config);
  Serial.println("Parameters saved to EEPROM");
}

bool validateParam(int index, float value) {
  if (index < 0 || index >= NUM_PARAMS) return false;
  return (value >= param_ranges[index].min && value <= param_ranges[index].max);
}

void processSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (serial_index > 0) {
        serial_buffer[serial_index] = '\0';
        processSerialCommand();
        serial_index = 0;
      }
    } else if (serial_index < sizeof(serial_buffer) - 1) {
      serial_buffer[serial_index++] = c;
    }
  }
}

void processSerialCommand() {
  char* token = strtok(serial_buffer, " ");
  if (token == NULL) return;
  
  if (strcmp(token, "report") == 0) {
    token = strtok(NULL, " ");
    if (token != NULL) {
      if (strcmp(token, "on") == 0) {
        report_enabled = true;
        Serial.println("Report enabled");
      } else if (strcmp(token, "off") == 0) {
        report_enabled = false;
        Serial.println("Report disabled");
      }
    }
  }
  else if (strcmp(token, "get") == 0) {
    token = strtok(NULL, " ");
    if (token != NULL) {
      for (int i = 0; i < NUM_PARAMS; i++) {
        if (strcmp(token, param_names[i]) == 0) {
          float val;
          switch (i) {
            case 0: val = right_lim_inf; break;
            case 1: val = right_lim_sup; break;
            case 2: val = left_lim_inf; break;
            case 3: val = left_lim_sup; break;
            case 4: val = encoder_calibration; break;
            case 5: val = gyro_calibration; break;
            case 6: val = Cn_beta; break;
            case 7: val = Cn_damper; break;
            default: val = 0;
          }
          Serial.print(param_names[i]);
          Serial.print(" = ");
          Serial.println(val, 4);
          return;
        }
      }
      Serial.print("Unknown parameter: ");
      Serial.println(token);
    }
  }
  else if (strcmp(token, "set") == 0) {
    token = strtok(NULL, " ");
    if (token != NULL) {
      char* param_name = token;
      token = strtok(NULL, " ");
      if (token != NULL) {
        float new_value = atof(token);
        
        for (int i = 0; i < NUM_PARAMS; i++) {
          if (strcmp(param_name, param_names[i]) == 0) {
            if (!validateParam(i, new_value)) {
              Serial.print("Value out of range! Valid range: ");
              Serial.print(param_ranges[i].min);
              Serial.print(" to ");
              Serial.println(param_ranges[i].max);
              return;
            }
            
            // 更新对应全局变量
            switch (i) {
              case 0: right_lim_inf = (uint16_t)round(new_value); break;
              case 1: right_lim_sup = (uint16_t)round(new_value); break;
              case 2: left_lim_inf = (uint16_t)round(new_value); break;
              case 3: left_lim_sup = (uint16_t)round(new_value); break;
              case 4: encoder_calibration = new_value; break;
              case 5: gyro_calibration = new_value; break;
              case 6: Cn_beta = new_value; break;
              case 7: Cn_damper = new_value; break;
            }
            
            // 保存到EEPROM
            saveParamsToEEPROM();
            
            Serial.print("Set ");
            Serial.print(param_names[i]);
            Serial.print(" to ");
            Serial.println(new_value, 4);
            return;
          }
        }
        Serial.print("Unknown parameter: ");
        Serial.println(param_name);
      }
    }
  }
  else if (strcmp(token, "help") == 0) {
    Serial.println("Available commands:");
    Serial.println("  report on/off - Enable/disable data reporting");
    Serial.println("  get <param>   - Get parameter value");
    Serial.println("  set <param> <value> - Set parameter value");
    Serial.println("  params        - List all parameters with ranges");
    Serial.println("  help          - Show this help");
  }
  else if (strcmp(token, "params") == 0) {
    printAllParams();
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(token);
    Serial.println("Type 'help' for available commands");
  }
}

void printAllParams() {
  Serial.println("Available parameters (current value | range):");
  for (int i = 0; i < NUM_PARAMS; i++) {
    float val;
    switch (i) {
      case 0: val = right_lim_inf; break;
      case 1: val = right_lim_sup; break;
      case 2: val = left_lim_inf; break;
      case 3: val = left_lim_sup; break;
      case 4: val = encoder_calibration; break;
      case 5: val = gyro_calibration; break;
      case 6: val = Cn_beta; break;
      case 7: val = Cn_damper; break;
      default: val = 0;
    }
    Serial.print("  ");
    Serial.print(param_names[i]);
    Serial.print(" = ");
    Serial.print(val, 4);
    Serial.print("  [");
    Serial.print(param_ranges[i].min);
    Serial.print(", ");
    Serial.print(param_ranges[i].max);
    Serial.println("]");
  }
}