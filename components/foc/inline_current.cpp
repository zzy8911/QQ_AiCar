// InlineCurrent.cpp
#include "inline_current.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// ADC 参数定义
#define _ADC_VOLTAGE      3.3f
#define _ADC_RESOLUTION   4095.0f
#define _ADC_CONV         (_ADC_VOLTAGE / _ADC_RESOLUTION)

#define _isset(a)         ((a) != (GPIO_NUM_NC))

#define TAG     "INLINE_CURR"

// 静态变量：ADC 单元句柄（共享）
adc_oneshot_unit_handle_t CurrSense::adc_unit_ = nullptr;

// 构造函数：灵活引脚配置
CurrSense::CurrSense(gpio_num_t pinA, gpio_num_t pinB, gpio_num_t pinC)
    : pinA_(pinA)
    , pinB_(pinB)
    , pinC_(pinC)
    , current_a_(0.0f)
    , current_b_(0.0f)
    , current_c_(0.0f)
{
    // 初始化 ADC 通道映射（根据 GPIO 自动匹配 ADC channel）
    auto get_adc_channel = [](gpio_num_t gpio) -> adc_channel_t {
        switch (gpio) {
            case GPIO_NUM_1:  return ADC_CHANNEL_0;
            case GPIO_NUM_2:  return ADC_CHANNEL_1;
            case GPIO_NUM_3:  return ADC_CHANNEL_2;
            case GPIO_NUM_4:  return ADC_CHANNEL_3;
            case GPIO_NUM_5:  return ADC_CHANNEL_4;
            case GPIO_NUM_6:  return ADC_CHANNEL_5;
            case GPIO_NUM_7:  return ADC_CHANNEL_6;
            case GPIO_NUM_8:  return ADC_CHANNEL_7;
            case GPIO_NUM_9:  return ADC_CHANNEL_8;
            case GPIO_NUM_10: return ADC_CHANNEL_9;
            default:
                ESP_LOGE(TAG, "GPIO %d not supported for ADC1!\n", gpio);
                return ADC_CHANNEL_0; // 无效占位
        }
    };

    adc_chan_a_ = get_adc_channel(pinA_);
    adc_chan_b_ = get_adc_channel(pinB_);
    adc_chan_c_ = _isset(pinC_) ? get_adc_channel(pinC_) : ADC_CHANNEL_0;

    // 电流检测参数
    shunt_resistor_ = SHUNT_RESISTOR;
    amp_gain_ = AMP_GAIN;
    volts_to_amps_ratio_ = 1.0f / shunt_resistor_ / amp_gain_;

    // 增益符号：根据你的硬件接线调整（负相位）
    gain_a_ = -volts_to_amps_ratio_;
    gain_b_ = -volts_to_amps_ratio_;
    gain_c_ = volts_to_amps_ratio_;  // 示例：C 相为正
}

// 配置 ADC 通道
void CurrSense::configureADCInline()
{
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,  // 0 ~ 3.3V
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    gpio_set_direction(pinA_, GPIO_MODE_INPUT);
    gpio_set_direction(pinB_, GPIO_MODE_INPUT);
    if (_isset(pinC_)) gpio_set_direction(pinC_, GPIO_MODE_INPUT);

    esp_err_t err;

    err = adc_oneshot_config_channel(adc_unit_, adc_chan_a_, &config);
    if (err != ESP_OK) ESP_LOGE(TAG, "Failed to config ADC channel A (GPIO%d)\n", pinA_);

    err = adc_oneshot_config_channel(adc_unit_, adc_chan_b_, &config);
    if (err != ESP_OK) ESP_LOGE(TAG, "Failed to config ADC channel B (GPIO%d)\n", pinB_);

    if (_isset(pinC_)) {
        err = adc_oneshot_config_channel(adc_unit_, adc_chan_c_, &config);
        if (err != ESP_OK) ESP_LOGE(TAG, "Failed to config ADC channel C (GPIO%d)\n", pinC_);
    }
}

// 读取 ADC 原始值并转为电压
float CurrSense::readADCVoltageInline(adc_channel_t channel)
{
    int raw;
    esp_err_t err = adc_oneshot_read(adc_unit_, channel, &raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed on channel %d\n", channel);
        return 0.0f;
    }
    return raw * _ADC_CONV;
}

// 校准零偏（调用前确保无电流）
void CurrSense::calibrateOffsets()
{
    const int calibration_rounds = 1000;
    offset_ia_ = offset_ib_ = offset_ic_ = 0.0f;

    for (int i = 0; i < calibration_rounds; i++) {
        offset_ia_ += readADCVoltageInline(adc_chan_a_);
        offset_ib_ += readADCVoltageInline(adc_chan_b_);
        if (_isset(pinC_)) offset_ic_ += readADCVoltageInline(adc_chan_c_);
        esp_rom_delay_us(1000); // 约 1ms
    }

    offset_ia_ /= calibration_rounds;
    offset_ib_ /= calibration_rounds;
    if (_isset(pinC_)) offset_ic_ /= calibration_rounds;
    ESP_LOGI(TAG, "Calibrated Offsets - IA: %.3f V, IB: %.3f V, IC: %.3f V", offset_ia_, offset_ib_, offset_ic_);
}

// 初始化：创建 ADC 单元（首次调用时），配置引脚，校准偏移
void CurrSense::init()
{
    // 只创建一次 ADC 单元
    if (!adc_unit_) {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
        };
        esp_err_t err = adc_oneshot_new_unit(&init_config, &adc_unit_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create ADC unit!\n");
            return;
        }
    }

    configureADCInline();
    calibrateOffsets();
}

// 获取三相电流（通过引用返回）
void CurrSense::updatePhaseCurrents()
{
    current_a_ = (readADCVoltageInline(adc_chan_a_) - offset_ia_) * gain_a_;
    current_b_ = (readADCVoltageInline(adc_chan_b_) - offset_ib_) * gain_b_;
    current_c_ = _isset(pinC_) ? (readADCVoltageInline(adc_chan_c_) - offset_ic_) * gain_c_ : 0.0f;
    // ESP_LOGI(TAG, "Currents - IA: %.3f A, IB: %.3f A, IC: %.3f A", current_a_, current_b_, current_c_);
}
