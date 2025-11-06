// InlineCurrent.h
#pragma once

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

class CurrSense {
public:
    CurrSense(gpio_num_t pinA, gpio_num_t pinB, gpio_num_t pinC = GPIO_NUM_NC);
    void init();
    void updatePhaseCurrents();
    float getCurrentA() const { return current_a_; }
    float getCurrentB() const { return current_b_; }
    float getCurrentC() const { return current_c_; }

private:
    static constexpr float SHUNT_RESISTOR = 0.005f;  // 5mΩ 分流电阻
    static constexpr float AMP_GAIN       = 50.0f;   // 运放增益

    static adc_oneshot_unit_handle_t adc_unit_; // ADC1 单元句柄（共享）

    gpio_num_t pinA_, pinB_, pinC_;
    adc_channel_t adc_chan_a_, adc_chan_b_, adc_chan_c_;

    float shunt_resistor_;
    float amp_gain_;
    float volts_to_amps_ratio_;
    float gain_a_, gain_b_, gain_c_;

    float offset_ia_, offset_ib_, offset_ic_;
    float current_a_, current_b_, current_c_;

    float readADCVoltageInline(adc_channel_t channel);
    void configureADCInline();
    void calibrateOffsets();
};

