#include "LowPassFilter.h"
#include <math.h>

LowPassFilter::LowPassFilter(float cutoff_freq, float sample_time)
    : _cutoff_freq(cutoff_freq), _sample_time(sample_time), _output(0.0f) {
    update_coefficients();
}

void LowPassFilter::set_cutoff_freq(float cutoff_freq) {
    _cutoff_freq = cutoff_freq;
    update_coefficients();
}

void LowPassFilter::set_sample_time(float sample_time) {
    _sample_time = sample_time;
    update_coefficients();
}

void LowPassFilter::set_initial_value(float initial_value) {
    _output = initial_value;
}

void LowPassFilter::update_coefficients() {
    if (_cutoff_freq <= 0.0f || _sample_time <= 0.0f) {
        _alpha = 1.0f;  // 无滤波
        return;
    }
    
    // 一阶低通滤波器系数计算
    // alpha = dt / (dt + 1/(2*pi*f_cutoff))
    float rc = 1.0f / (2.0f * M_PI * _cutoff_freq);
    _alpha = _sample_time / (_sample_time + rc);
}

float LowPassFilter::update(float input) {
    // 一阶低通滤波器公式: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    _output = _alpha * input + (1.0f - _alpha) * _output;
    return _output;
}