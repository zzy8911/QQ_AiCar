#include "esp32_adc_driver.h"

#if CONFIG_SIMPLEFOC_ADC_MODE_LEGACY
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "esp_attr.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_platform.h"

#define TAG     "ESP32S3_ADC"

static uint8_t __analogAttenuation = ADC_ATTEN_DB_12;
static uint8_t __analogWidth = ADC_WIDTH_BIT_12;
static uint8_t __analogReturnedWidth = ADC_WIDTH_BIT_12;
static uint8_t __analogClockDiv = 1;
static uint8_t __pin_attenuation[SOC_GPIO_PIN_COUNT];

// refer to https://documentation.espressif.com/esp32-s3_technical_reference_manual_cn.pdf, page 1378.
static int8_t digitalPinToAnalogChannel(uint8_t pin) {
    if (pin > 0 && pin <= 20) {
        // GPIOs 1-20 map to channels 0-19
        return pin - 1;
    } else
        return -1;
}

static inline uint16_t mapResolution(uint16_t value)
{
    uint8_t from = __analogWidth;
    if (from == __analogReturnedWidth) {
        return value;
    }
    if (from > __analogReturnedWidth) {
        return value >> (from  - __analogReturnedWidth);
    }
    return value << (__analogReturnedWidth - from);
}

void __analogSetClockDiv(uint8_t clockDiv){
    if(!clockDiv){
        clockDiv = 1;
    }
    __analogClockDiv = clockDiv;
}

void __analogSetAttenuation(adc_atten_t attenuation)
{
    __analogAttenuation = attenuation & 3;
}

void __analogInit(){
    static bool initialized = false;
    if(initialized){
        return;
    }
    initialized = true;
    __analogSetClockDiv(__analogClockDiv);
    for(int i=0; i<SOC_GPIO_PIN_COUNT; i++){
        __pin_attenuation[i] = ADC_ATTEN_DB_12;
    }
}

void __analogSetPinAttenuation(uint8_t pin, adc_atten_t attenuation)
{
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0 || attenuation > 3){
        return ;
    }
    if(channel > (SOC_ADC_MAX_CHANNEL_NUM - 1)){
        adc2_config_channel_atten(adc2_channel_t(channel - SOC_ADC_MAX_CHANNEL_NUM), attenuation);
    } else {
        adc1_config_channel_atten(adc1_channel_t(channel), attenuation);
    }
    __analogInit();
    if((__pin_attenuation[pin] != ADC_ATTEN_DB_12) || (attenuation != __analogAttenuation)){
        __pin_attenuation[pin] = attenuation;
    }
}

bool __adcAttachPin(uint8_t pin){
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        ESP_LOGE(TAG, "Pin %u is not ADC pin!", pin);
        return false;
    }
    __analogInit();

    pinMode(pin, INPUT);
    __analogSetPinAttenuation(pin, adc_atten_t((__pin_attenuation[pin] != ADC_ATTEN_DB_12)?__pin_attenuation[pin]:__analogAttenuation));
    return true;
}

void __analogReadResolution(uint8_t bits)
{
    if(!bits || bits > 16){
        return;
    }
    __analogReturnedWidth = bits;
}

uint16_t __analogReadRaw(uint8_t pin)
{
    int8_t channel = digitalPinToAnalogChannel(pin);
    int value = 0;
    esp_err_t r = ESP_OK;
    if(channel < 0){
        ESP_LOGE(TAG, "Pin %u is not ADC pin!", pin);
        return value;
    }
    __adcAttachPin(pin);
    if(channel > (SOC_ADC_MAX_CHANNEL_NUM - 1)){
        channel -= SOC_ADC_MAX_CHANNEL_NUM;
        r = adc2_get_raw(adc2_channel_t(channel), adc_bits_width_t(__analogWidth), &value);
        if ( r == ESP_OK ) {
            return value;
        } else if ( r == ESP_ERR_INVALID_STATE ) {
            ESP_LOGE(TAG, "GPIO%u: %s: ADC2 not initialized yet.", pin, esp_err_to_name(r));
        } else if ( r == ESP_ERR_TIMEOUT ) {
            ESP_LOGE(TAG, "GPIO%u: %s: ADC2 is in use by Wi-Fi. Please see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#adc-limitations for more info", pin, esp_err_to_name(r));
        } else {
            ESP_LOGE(TAG, "GPIO%u: %s", pin, esp_err_to_name(r));
        }
    } else {
        value = adc1_get_raw(adc1_channel_t(channel));
        return value;
    }
    return value;
}

uint16_t __analogRead(uint8_t pin)
{
    uint16_t value = __analogReadRaw(pin);
    return mapResolution(value);
}

uint16_t IRAM_ATTR adcRead(uint8_t pin)
{
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        ESP_LOGE(TAG, "ERROR: Not ADC pin: %d", pin);
        return false;//not adc pin
    }

    // start the ADC conversion
    if(channel > 9){
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS2_CTRL2_REG, SENS_SAR2_EN_PAD, (1 << (channel - 10)), SENS_SAR2_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
    } else {
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS1_CTRL2_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
    }

    uint16_t value = 0;

    if(channel > 9){
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DONE_SAR) == 0);
        // read the value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
    } else {
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DONE_SAR) == 0);
        // read teh value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    }

    return value;
}

// configure the ADC for the pin
bool IRAM_ATTR adcInit(uint8_t pin)
{
    static bool initialized = false;

    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        ESP_LOGE(TAG, "ERROR: Not ADC pin: %d", pin);
        return false;//not adc pin
    }

    if(! initialized){
        __analogSetAttenuation(ADC_ATTEN_DB_12);
        __analogReadResolution(12);
    }
    pinMode(pin, INPUT);
    __analogRead(pin); // necessary, to initialize the ADC
    __analogSetPinAttenuation(pin, ADC_ATTEN_DB_12);

    initialized = true;
    return true;
}

void adcStart()
{
    // Legacy 模式不需要启动
}
#endif

#if CONFIG_SIMPLEFOC_ADC_MODE_ONESHOT
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "soc/sens_reg.h"

#define TAG "ESP32S3_ADC"
static adc_oneshot_unit_handle_t adc1_handle = nullptr;
static adc_oneshot_unit_handle_t adc2_handle = nullptr;

// Pin -> channel 映射
static int8_t digitalPinToAnalogChannel(uint8_t pin) {
    if (pin > 0 && pin <= 20) {
        return pin - 1;  // GPIO1 -> ch0 ... GPIO20 -> ch19
    }
    return -1;
}

static uint8_t __analogReturnedWidth = 12;
static adc_atten_t __analogAttenuation = ADC_ATTEN_DB_12;

// 每个通道单独记录配置
struct AdcChannelCfg {
    bool initialized = false;
    adc_channel_t channel;
    adc_unit_t unit;
    adc_atten_t atten;
};
static AdcChannelCfg adc_cfg[SOC_GPIO_PIN_COUNT];

/*----------------------------------------------------------
    初始化 ADC 驱动
----------------------------------------------------------*/
static void ensure_adc_driver_initialized(adc_unit_t unit)
{
    if (unit == ADC_UNIT_1) {
        if (adc1_handle) return;

        adc_oneshot_unit_init_cfg_t cfg = {
            .unit_id = ADC_UNIT_1,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg, &adc1_handle));
        ESP_LOGI(TAG, "ADC1 Oneshot Driver Init");
    } else {
        if (adc2_handle) return;

        adc_oneshot_unit_init_cfg_t cfg = {
            .unit_id = ADC_UNIT_2,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg, &adc2_handle));
        ESP_LOGI(TAG, "ADC2 Oneshot Driver Init");
    }
}

/*----------------------------------------------------------
    adcInit(pin)：初始化指定 ADC pin
----------------------------------------------------------*/
bool IRAM_ATTR adcInit(uint8_t pin)
{
    int8_t ch = digitalPinToAnalogChannel(pin);
    if (ch < 0) {
        ESP_LOGE(TAG, "Not ADC pin: %d", pin);
        return false;
    }

    AdcChannelCfg &cfg = adc_cfg[pin];
    if (cfg.initialized) return true;

    cfg.initialized = true;
    cfg.channel = (adc_channel_t)(ch % 10);
    cfg.atten = __analogAttenuation;

    if (ch >= 10) cfg.unit = ADC_UNIT_2;
    else          cfg.unit = ADC_UNIT_1;

    // 初始化 unit（只第一次会创建）
    ensure_adc_driver_initialized(cfg.unit);

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = cfg.atten,
        .bitwidth = ADC_BITWIDTH_12
    };

    int raw = 0;
    if (cfg.unit == ADC_UNIT_1) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, cfg.channel, &ch_cfg));
        adc_oneshot_read(adc1_handle, cfg.channel, &raw); // 非常重要，它会初始化adc channel，必须调用一次，不然adcRead无效
    } else {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, cfg.channel, &ch_cfg));
        adc_oneshot_read(adc2_handle, cfg.channel, &raw);
    }

    return true;
}

void adcStart()
{
    // Oneshot 模式不需要启动
}

/*----------------------------------------------------------
    adcRead(pin)：读取 ADC（IRAM 安全，可在 ISR 用）
----------------------------------------------------------*/
uint16_t IRAM_ATTR adcRead(uint8_t pin)
{
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        ESP_LOGE(TAG, "ERROR: Not ADC pin: %d", pin);
        return false;//not adc pin
    }

    // start the ADC conversion
    if(channel > 9){
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS2_CTRL2_REG, SENS_SAR2_EN_PAD, (1 << (channel - 10)), SENS_SAR2_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_START_SAR_M);
    } else {
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS1_CTRL2_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_START_SAR_M);
    }

    uint16_t value = 0;

    if(channel > 9){
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DONE_SAR) == 0);
        // read the value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS2_CTRL2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
    } else {
        //wait for conversion
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DONE_SAR) == 0);
        // read teh value
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS1_CTRL2_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    }

    return value;
}

/*----------------------------------------------------------
    其他旧接口保持不变，但对应 new-driver
----------------------------------------------------------*/
void __analogSetAttenuation(adc_atten_t atten)
{
    __analogAttenuation = atten;
}

void __analogReadResolution(uint8_t bits)
{
    if (bits >= 9 && bits <= 16)
        __analogReturnedWidth = bits;
}
#endif

#if CONFIG_SIMPLEFOC_ADC_MODE_CONTINUOUS
#include <string.h>
#include <stdio.h>
#include <map>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"

static const char *TAG = "ADC_DMA";

// --------------------------
// 全局配置
// --------------------------
struct AdcChannelInfo {
    uint8_t pin = 0;             // 0 表示未注册
    volatile int16_t latest_value = -1;
};
static AdcChannelInfo g_adc_channel_map[SOC_ADC_MAX_CHANNEL_NUM] = {}; // 索引 = channel

// DMA 缓冲区
static uint8_t *dma_buffer = nullptr;
static uint32_t dma_buffer_size = 0;

// Continuous ADC handle
static adc_continuous_handle_t adc_handle = NULL;

// FreeRTOS 接收通知任务
static TaskHandle_t adc_task_handle = NULL;

// -----------------------------------------------------
//  ISR Callback: DMA 采样完成时触发
// -----------------------------------------------------
static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle,
                                       const adc_continuous_evt_data_t *edata,
                                       void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(adc_task_handle, &mustYield);
    return mustYield == pdTRUE;
}

// -----------------------------------------------------
// pin 注册（SimpleFOC init 时会多次调用）
// -----------------------------------------------------
bool adcInit(uint8_t pin)
{
    adc_unit_t unit;
    adc_channel_t ch;
    if (adc_continuous_io_to_channel(pin, &unit, &ch) != ESP_OK) return false;

    if (ch < SOC_ADC_MAX_CHANNEL_NUM) {
        g_adc_channel_map[ch].pin = pin;
        g_adc_channel_map[ch].latest_value = 0;
    }
    ESP_LOGI(TAG, "Registered pin %d -> channel %d", pin, ch);
    return true;
}

// -----------------------------------------------------
// ADC 初始化（只执行一次）
// -----------------------------------------------------
void adcStart(int sample_rate_per_channel)
{
    static bool adc_started = false;
    if (adc_started) return;     // 多次调用 Safe
    adc_started = true;

    ESP_LOGI(TAG, "Initializing ADC continuous...");

    // ---- 构建通道 pattern ----
    adc_digi_pattern_config_t patterns[SOC_ADC_MAX_CHANNEL_NUM] = {0};
    uint32_t pat_idx = 0;
    for (int ch = 0; ch < SOC_ADC_MAX_CHANNEL_NUM; ch++) {
        if (g_adc_channel_map[ch].pin != 0) {
            patterns[pat_idx].atten = ADC_ATTEN_DB_12;
            patterns[pat_idx].channel = ch;
            patterns[pat_idx].unit = ADC_UNIT_1;
            patterns[pat_idx].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
            ESP_LOGI(TAG, "Pattern %lu: channel=%d", pat_idx, ch);
            pat_idx++;
        }
    }
    dma_buffer_size = pat_idx * SOC_ADC_DIGI_RESULT_BYTES;
    dma_buffer = (uint8_t*)heap_caps_malloc(dma_buffer_size, MALLOC_CAP_DMA);

    // ---- 创建 ADC continuous handle ----
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 128,
        .conv_frame_size = dma_buffer_size,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_continuous_config_t adc_cfg = {
        .pattern_num    = pat_idx,
        .adc_pattern    = patterns,
        .sample_freq_hz = sample_rate_per_channel*pat_idx, // 总采样率, 每通道采样率 = sample_freq_hz / 通道数
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_cfg));

    // ---- 注册 callback ----
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));

    // ---- 启动 ADC ----
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    // 创建采样任务
    xTaskCreatePinnedToCore(
        [](void *) {
            uint32_t ret_len = 0;

            while (1) {
                // 等 DMA 完成
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                // 读数据（非阻塞）
                while (adc_continuous_read(adc_handle, dma_buffer, dma_buffer_size, &ret_len, 0) == ESP_OK) {
                    for (int i = 0; i < ret_len; i += SOC_ADC_DIGI_RESULT_BYTES) {
                        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&dma_buffer[i];

                        if (p->type2.unit != 0) continue; // 只处理 ADC1

                        uint8_t ch = p->type2.channel;
                        if (g_adc_channel_map[ch].pin != 0) {
                            g_adc_channel_map[ch].latest_value = p->type2.data;
                        }
                    }
                }
            }

            free(dma_buffer);
        },
        "adc_dma_task",
        4096,
        NULL,
        5,
        &adc_task_handle,
        0     // 避免和 FOC 冲突
    );

    ESP_LOGI(TAG, "ADC continuous started.");

    const TickType_t timeout = pdMS_TO_TICKS(200);  // 最多等待200ms
    TickType_t start = xTaskGetTickCount();
    bool ready = false;

    while (!ready && (xTaskGetTickCount() - start < timeout)) {
        ready = true;
        for (int ch = 0; ch < SOC_ADC_MAX_CHANNEL_NUM; ch++) {
            if (g_adc_channel_map[ch].pin != 0 &&
                g_adc_channel_map[ch].latest_value == -1) {  // 还没收到数据
                ready = false;
                break;
            }
        }
        if (!ready) vTaskDelay(1);
    }
    if (!ready) {
        ESP_LOGW(TAG, "Not all ADC channels ready after %lu ms", pdTICKS_TO_MS(timeout));
    } else {
        ESP_LOGI(TAG, "All ADC channels ready.");
    }
}

// -----------------------------------------------------
// 用户 API: SimpleFOC 调用的 adcRead(pin)
// -----------------------------------------------------
uint16_t adcRead(uint8_t pin)
{
    for (int ch = 0; ch < SOC_ADC_MAX_CHANNEL_NUM; ch++) {
        if (g_adc_channel_map[ch].pin == pin) {
            return g_adc_channel_map[ch].latest_value;
        }
    }
    return 0;
}
#endif
