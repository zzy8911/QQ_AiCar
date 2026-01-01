#include "wifi_board.h"
#include "audio_codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include "assets/lang_config.h"
#include "self_balancing/hal.h"
#include "mcp_server.h"
#include "../../self_balancing/motor.h"
#include "port/esp32_adc_driver.h"

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif

#define TAG "CompactWifiBoardLCD"

LV_FONT_DECLARE(font_puhui_30_4);
LV_FONT_DECLARE(font_awesome_20_4);

class CompactWifiBoardLCD : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    LcdDisplay* display_;

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = IMU_SDA,
            .scl_io_num = IMU_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeSpi() {
        // init encoder spi bus
        spi_bus_config_t buscfg = {
            .mosi_io_num = -1,
            .miso_io_num = ENCODER_MISO,
            .sclk_io_num = ENCODER_SCK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 1000,
        };
        spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
        // init display spi bus
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif

        esp_lcd_panel_reset(panel);


        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        lvgl_port_cfg_t port_cfg = {
            .task_priority = 4,
            .task_stack = 7168,
            .task_affinity = 0, // LVGL: CORE 0
            .task_max_sleep_ms = 500,
            .timer_period_ms = 5,
        };
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_30_4,
                                        .icon_font = &font_awesome_20_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                        .emoji_font = font_emoji_32_init(),
#else
                                        .emoji_font = DISPLAY_HEIGHT > 240 ? font_emoji_64_init() : font_emoji_32_init(),
#endif
                                    }, port_cfg);
    }



    void InitializeButtons() {
        boot_button_.OnLongPress([this]() {
            ESP_LOGI(TAG, "Boot button long pressed. Resetting WiFi configuration.");
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
        boot_button_.OnClick([this]() {
            ESP_LOGI(TAG, "Boot button clicked.");
            static int cnt = 0;
            if (cnt == 0)
                HAL::Init(GetI2cBus());
            if (cnt%2 == 0)
                Motor::getInstance().start();
            else
                Motor::getInstance().stop();
            cnt++;
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.rebot.control",
            "forward: 前进\n"
            "backward: 后退\n"
            "left: 左转\n"
            "right: 右转\n"
            "rotate: 旋转\n"
            "distance: 厘米\n"
            "degree: 角度\n"
            "turn_around: 转圈",
            PropertyList({
                Property("action", kPropertyTypeString),  // 动作类型
                Property("distance", kPropertyTypeInteger, 30, 1, 50),  // 可选参数，仅前进/后退需要
                Property("degree", kPropertyTypeInteger, 90)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                const std::string& action = properties["action"].value<std::string>();
                int distance = properties["distance"].value<int>();
                int degree = properties["degree"].value<int>();

                ESP_LOGI(TAG, "Received control action:%s, distance:%d, degree:%d", action.c_str(), distance, degree);

                if (action == "forward") {
                    Motor::getInstance().move(Motor::FORWARD, distance);
                } else if (action == "backward") {
                    Motor::getInstance().move(Motor::BACKWARD, distance);
                } else if (action == "left") {
                    Motor::getInstance().rotate(degree); // According to the right-hand rule, counter-clockwise is the positive direction.
                } else if (action == "right") {
                    Motor::getInstance().rotate(-degree); // According to the right-hand rule, clockwise is the negative direction.
                } else if (action == "rotate") {
                    Motor::getInstance().rotate(degree);
                } else if (action == "turn_around") {
                    Motor::getInstance().turnAround();
                } else {
                    ESP_LOGW(TAG, "Unknown action: %s", action.c_str());
                    return false;
                }

                return true;
            });
    }

public:
    CompactWifiBoardLCD() :
        boot_button_(BOOT_BUTTON_GPIO, false, 2000, 50) {
        InitializeI2c();
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeIot();
        InitializeTools();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
    }

    virtual Led* GetLed() override {
        static NoLed led;
        return &led;
    }

    virtual i2c_master_bus_handle_t GetI2cBus() override {
        return i2c_bus_;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            GPIO_NUM_NC, // PA is not used
            ES8311_CODEC_DEFAULT_ADDR,
            ES7210_CODEC_DEFAULT_ADDR,
            true);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        constexpr int BAT_R1 = 15000; // 上端分压电阻
        constexpr int BAT_R2 = 10000; // 下端分压电阻
        constexpr float BAT_DIV_RATIO =
            float(BAT_R1 + BAT_R2) / float(BAT_R2); // = 2.5

        /* ---------- 1. ADC → ADC电压 ---------- */
        float adc_raw = adcRead(BATTERY_ADC_GPIO);
        float vadc = adc_raw * (_ADC_VOLTAGE / _ADC_RESOLUTION);

        /* ---------- 2. ADC电压 → 电池电压 ---------- */
        float battery_vol = vadc * BAT_DIV_RATIO;

        /* ---------- 3. 2S OCV 表 ---------- */
        struct BatteryOcvPoint {
            float voltage;
            uint8_t percent;
        };
        static const BatteryOcvPoint kOcvTable[] = {
            {8.40f, 100},
            {8.00f, 80},
            {7.60f, 60},
            {7.40f, 40},
            {7.20f, 20},
            {6.60f, 0},
        };
        const int N = sizeof(kOcvTable) / sizeof(kOcvTable[0]);

        /* ---------- 4. 电压 → 百分比 ---------- */
        if (battery_vol >= kOcvTable[0].voltage) {
            level = 100;
        } else if (battery_vol <= kOcvTable[N - 1].voltage) {
            level = 0;
        } else {
            for (int i = 0; i < N - 1; i++) {
                if (battery_vol <= kOcvTable[i].voltage &&
                    battery_vol >  kOcvTable[i + 1].voltage) {

                    float v1 = kOcvTable[i].voltage;
                    float v2 = kOcvTable[i + 1].voltage;
                    int   p1 = kOcvTable[i].percent;
                    int   p2 = kOcvTable[i + 1].percent;

                    float ratio = (battery_vol - v2) / (v1 - v2);
                    level = p2 + ratio * (p1 - p2);
                    break;
                }
            }
        }

        /* ---------- 5. 充放电状态（占位） ---------- */
        charging = false;
        discharging = true;

        return true;
    }
};

DECLARE_BOARD(CompactWifiBoardLCD);
