#ifndef __LOW_PASS_FILTER_H__
#define __LOW_PASS_FILTER_H__

class LowPassFilter {
public:
    LowPassFilter(float cutoff_freq, float sample_time);
    
    // 设置参数
    void set_cutoff_freq(float cutoff_freq);
    void set_sample_time(float sample_time);
    void set_initial_value(float initial_value);
    
    // 更新滤波器
    float update(float input);
    
    // 获取当前输出
    float get_output() const { return _output; }
    
    // 重新计算系数
    void update_coefficients();
    
private:
    float _cutoff_freq;   // 截止频率 (Hz)
    float _sample_time;   // 采样时间 (秒)
    float _alpha;         // 滤波器系数
    float _output;        // 当前输出值
};

#endif // __LOW_PASS_FILTER_H__