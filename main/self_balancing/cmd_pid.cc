#include <esp_console.h>
#include <esp_log.h>
#include <argtable3/argtable3.h>

#include "motor.h"

static const char* TAG = "cmd_pid";

static struct {
    struct arg_str* pid;
    struct arg_str* param;
    struct arg_dbl* value;
    struct arg_end* end;
} pid_args;

static int do_pid_cmd(int argc, char** argv)
{
    Motor& m = Motor::getInstance();

    if (argc < 2) {
        ESP_LOGI(TAG, "Usage: pid <show|reset|stb|vel|steer> [p|i|d] [value]");
        return -1;
    }

    std::string sub = argv[1];

    // ====== SHOW ALL ======
    if (sub == "show") {
        auto* stb   = m.getPID(Motor::PIDType::STB);
        auto* vel   = m.getPID(Motor::PIDType::VEL);
        auto* steer = m.getPID(Motor::PIDType::STEER);

        ESP_LOGI(TAG, "===== PID Settings =====");
        ESP_LOGI(TAG, "STB:   P=%.3f I=%.3f D=%.3f", stb->P, stb->I, stb->D);
        ESP_LOGI(TAG, "VEL:   P=%.3f I=%.3f D=%.3f", vel->P, vel->I, vel->D);
        ESP_LOGI(TAG, "STEER: P=%.3f I=%.3f D=%.3f", steer->P, steer->I, steer->D);
        ESP_LOGI(TAG, "=========================");
        return 0;
    }

    // ====== RESET ALL ======
    if (sub == "reset") {
        m.getPID(Motor::PIDType::STB)->reset();
        m.getPID(Motor::PIDType::VEL)->reset();
        m.getPID(Motor::PIDType::STEER)->reset();
        ESP_LOGI(TAG, "All PID integrators reset.");
        return 0;
    }

    // ====== PID TYPE ======
    Motor::PIDType type;
    if      (sub == "stb")   type = Motor::PIDType::STB;
    else if (sub == "vel")   type = Motor::PIDType::VEL;
    else if (sub == "steer") type = Motor::PIDType::STEER;
    else {
        ESP_LOGI(TAG, "Unknown pid type: %s", sub.c_str());
        return -1;
    }

    IPID* pid = m.getPID(type);

    // ====== SHOW ONE PID ======
    if (argc == 2) {
        ESP_LOGI(TAG, "%s PID: P=%.3f I=%.3f D=%.3f",
               sub.c_str(), pid->P, pid->I, pid->D);
        return 0;
    }

    // ====== MODIFY PID TERM ======
    if (argc == 4) {
        std::string term = argv[2];
        float value = atof(argv[3]);

        if      (term == "p") pid->P = value;
        else if (term == "i") pid->I = value;
        else if (term == "d") pid->D = value;
        else {
            ESP_LOGI(TAG, "Unknown parameter: %s (must be p/i/d)", term.c_str());
            return -1;
        }

        if (term == "i" || term == "d") {
            pid->reset();
        }

        ESP_LOGI(TAG, "%s.%s = %.4f", sub.c_str(), term.c_str(), value);
        return 0;
    }

    ESP_LOGI(TAG, "Usage: pid %s [p|i|d] [value]", sub.c_str());
    return -1;
}

void register_pid_cmd()
{
    pid_args.pid   = arg_str1(NULL, NULL, "<pid>", "stb|vel|steer|show");
    pid_args.param = arg_str0(NULL, NULL, "<param>", "p|i|d");
    pid_args.value = arg_dbl0(NULL, NULL, "<value>", "value");
    pid_args.end   = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "pid",
        .help = "PID tuning: pid <stb|vel|steer> <p|i|d> <value>  OR  pid show",
        .hint = NULL,
        .func = &do_pid_cmd,
        .argtable = &pid_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}