#include "pid_tuner.h"
#include "wifi_board.h"

static const char *TAG = "PID_TUNER";

#ifdef CONFIG_ENABLE_CONSOLE
#include <esp_console.h>
#include <esp_log.h>
#include <argtable3/argtable3.h>

#include "motor.h"

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
        auto* stb   = m.getPIDParam(Motor::PIDType::STB);
        auto* vel   = m.getPIDParam(Motor::PIDType::VEL);
        auto* steer = m.getPIDParam(Motor::PIDType::STEER);

        ESP_LOGI(TAG, "===== PID Settings =====");
        ESP_LOGI(TAG, "STB:   P=%.3f I=%.3f D=%.3f", stb->P, stb->I, stb->D);
        ESP_LOGI(TAG, "VEL:   P=%.3f I=%.3f D=%.3f", vel->P, vel->I, vel->D);
        ESP_LOGI(TAG, "STEER: P=%.3f I=%.3f D=%.3f", steer->P, steer->I, steer->D);
        ESP_LOGI(TAG, "=========================");
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

    auto* pid = m.getPIDParam(type);

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
        m.updatePIDParam(type, pid->P, pid->I, pid->D);

        // if (term == "i" || term == "d") {
        //     pid->reset();
        // }

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
#endif

#ifdef CONFIG_PID_TUNER_ENABLE
#include <string.h>
#include <errno.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "cJSON.h"
#include "motor.h"
#include "esp_wifi.h"

// 遥测目标地址和 Socket
// 【动态目标】: 遥测数据目标地址，从接收到的命令中动态获取
static struct sockaddr_in pc_addr_target;
static bool pc_addr_target_valid = false;  // 标记是否已成功获取 PC 地址

// 支持的 key 映射
static const std::unordered_map<std::string, Motor::PIDType> pid_key_map = {
    {"stb",     Motor::PIDType::STB},
    {"vel",     Motor::PIDType::VEL},
    {"steer",   Motor::PIDType::STEER},
};

// helper: 从 cJSON 对象中尝试多个 key 取数（接受数字）
static bool get_number_by_keys(cJSON *obj, const std::vector<const char*> &keys, float &out) {
    if (!obj) return false;
    for (auto k : keys) {
        cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, k);
        if (it && cJSON_IsNumber(it)) {
            out = (float)it->valuedouble;
            return true;
        }
    }
    return false;
}

// 解析并应用单个 pid 对象（cJSON object），name 为 "stb"/"vel"/"steer" 等
static void parse_pid_obj_and_apply(const std::string &name, cJSON *obj) {
    Motor& m = Motor::getInstance();

    if (!obj || !cJSON_IsObject(obj)) return;

    Motor::PIDType type;
    auto it = pid_key_map.find(name);
    if (it != pid_key_map.end()) type = it->second;
    else {
        ESP_LOGW(TAG, "Unknown PID name: %s", name.c_str());
        return;
    }

    auto *pid = m.getPIDParam(type);
    get_number_by_keys(obj, {"p","kp","P","KP"}, pid->P);
    get_number_by_keys(obj, {"i","ki","I","KI"}, pid->I);
    get_number_by_keys(obj, {"d","kd","D","KD"}, pid->D);
    m.updatePIDParam(type, pid->P, pid->I, pid->D);

    ESP_LOGI(TAG, "Applied PID '%s' -> kp=%.6f ki=%.6f kd=%.6f",
             name.c_str(), pid->P, pid->I, pid->D);
}

// 周期性发送遥测数据
static void send_telemetry_data(int sock) {
    if (!pc_addr_target_valid) {
        ESP_LOGD(TAG, "PC target address not set yet. Skipping telemetry.");
        return;
    }

    Motor& m = Motor::getInstance();

    // 接口获取实时数据
    float current_pitch = m.getPitch();
    float current_motor_l_speed = m.getMotorLeftSpeed();
    float current_motor_r_speed = m.getMotorRightSpeed();
    // float current_ql = m.getCurrentQL();
    // float current_qr = m.getCurrentQR();

    // 创建 JSON 结构
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "cmd", "telemetry");

    cJSON *payload = cJSON_CreateObject();

    // 【发送 Pitch】
    cJSON_AddNumberToObject(payload, "pitch", current_pitch);
    cJSON_AddNumberToObject(payload, "motor_left_speed", current_motor_l_speed);
    cJSON_AddNumberToObject(payload, "motor_right_speed", current_motor_r_speed);

    // 发送电流
    // cJSON *currents = cJSON_CreateObject();
    // cJSON_AddNumberToObject(currents, "ql", current_ql);
    // cJSON_AddNumberToObject(currents, "qr", current_qr);
    // cJSON_AddItemToObject(payload, "current", currents);

    cJSON_AddItemToObject(root, "payload", payload);

    char *json_string = cJSON_PrintUnformatted(root);

    if (json_string) {
        // 使用记录的 pc_addr_target 发送数据
        ssize_t sent = sendto(sock, json_string, strlen(json_string), 0,
                              (const struct sockaddr*)&pc_addr_target, sizeof(pc_addr_target));
        if (sent < 0) {
            ESP_LOGE(TAG, "sendto failed to PC %s:%d: errno=%d",
                     inet_ntoa(pc_addr_target.sin_addr),
                     CONFIG_ESP32_LISTENING_PORT,
                     errno);
        } else {
            ESP_LOGD(TAG, "Telemetry sent %d bytes to PC.", (int)sent);
        }
        free(json_string);
    }

    cJSON_Delete(root);
}

static void send_pid_params(int sock)
{
    if (!pc_addr_target_valid) {
        ESP_LOGD(TAG, "PC target address not set yet. Skipping telemetry.");
        return;
    }

    Motor& m = Motor::getInstance();

    const auto *stb   = m.getPID(Motor::PIDType::STB);
    const auto *vel   = m.getPID(Motor::PIDType::VEL);
    const auto *steer = m.getPID(Motor::PIDType::STEER);
    float midpoint    = m.getMidpoint();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "cmd", "pid");

    cJSON *payload = cJSON_CreateObject();

    auto add_pid = [&](const char *name, const IPID *p) {
        cJSON *o = cJSON_CreateObject();
        cJSON_AddNumberToObject(o, "p", p->P);
        cJSON_AddNumberToObject(o, "i", p->I);
        cJSON_AddNumberToObject(o, "d", p->D);
        cJSON_AddItemToObject(payload, name, o);
    };

    add_pid("stb", stb);
    add_pid("vel", vel);
    add_pid("steer", steer);

    cJSON_AddNumberToObject(payload, "midpoint", midpoint);
    cJSON_AddItemToObject(root, "payload", payload);

    char *json = cJSON_PrintUnformatted(root);
    if (json) {
        sendto(sock, json, strlen(json), 0,
               (const struct sockaddr*)&pc_addr_target, sizeof(pc_addr_target));
        free(json);
    }

    cJSON_Delete(root);

    ESP_LOGI(TAG, "PID params sent to PC");
}

// 解析 JSON：支持两种格式（批量 & 单条）
static void handle_json_payload(const char *json_str, int sockfd, const struct sockaddr_in *src_addr, socklen_t src_len) {
    if (!json_str) return;
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGW(TAG, "cJSON_Parse failed");
        return;
    }

    // 优先识别批量命令 {"cmd":"set_pid", "payload":{...}}
    cJSON *cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");

    // 记录发送方 PC 的 IP 地址，用于遥测发送
    if (src_len == sizeof(struct sockaddr_in)) {
        // 记录 PC 的地址
        memcpy(&pc_addr_target, src_addr, src_len);
        // 确保目标端口使用配置宏
        pc_addr_target.sin_port = htons(CONFIG_PC_LISTENING_PORT);
        pc_addr_target_valid = true;

        ESP_LOGI(TAG, "PC Target IP recorded: %s:%d.",
                    inet_ntoa(pc_addr_target.sin_addr), CONFIG_PC_LISTENING_PORT);
    }

    // -------- get_pid --------
    if (strcmp(cmd->valuestring, "get_pid") == 0) {
        send_pid_params(sockfd);
    } else if (cmd && cJSON_IsString(cmd) && strcmp(cmd->valuestring, "set_pid") == 0) {
        cJSON *payload = cJSON_GetObjectItemCaseSensitive(root, "payload");
        if (payload && cJSON_IsObject(payload)) {
            // 遍历 payload 的成员（stb, vel, steer...）
            for (cJSON *c = payload->child; c; c = c->next) {
                if (c->string)
                    parse_pid_obj_and_apply(c->string, c);
            }
            // midpoing
            cJSON *midpoint_item = cJSON_GetObjectItemCaseSensitive(payload, "midpoint");
            if (midpoint_item && cJSON_IsNumber(midpoint_item)) {
                float midpoint = (float)midpoint_item->valuedouble;
                Motor::getInstance().setMidpoint(midpoint);
                ESP_LOGI(TAG, "Set balance midpoint to %.6f", midpoint);
            }
            // 发送 ACK（可选）
            if (sockfd >= 0 && src_addr) {
                cJSON *ack = cJSON_CreateObject();
                cJSON_AddStringToObject(ack, "cmd", "ack");
                cJSON_AddStringToObject(ack, "result", "ok");
                char *s = cJSON_PrintUnformatted(ack);
                if (s) {
                    sendto(sockfd, s, strlen(s), 0, (const struct sockaddr*)src_addr, src_len);
                    free(s);
                }
                cJSON_Delete(ack);
            }
        }
    } else {
        ESP_LOGW(TAG, "Unknown JSON format");
    }

    cJSON_Delete(root);
}

extern EventGroupHandle_t wifi_event_group_;
static bool is_wifi_connected()
{
    // 等待 WifiBoard 完成连接（STA 模式）
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group_,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        20000 / portTICK_PERIOD_MS   // 20 秒超时，避免卡死
    );

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "WiFi not connected -> Skip PID tuner (AP mode)");
        return false;
    }

    ESP_LOGI(TAG, "WiFi connected -> Starting UDP tuner");
    return true;
}

// UDP server task
static void pid_udp_task(void *arg) {
    if (is_wifi_connected() == false) {
        vTaskDelete(NULL);
        return;
    }

    int rx_len = 512;
    char *rxbuf = (char*)malloc(rx_len);
    if (!rxbuf) {
        ESP_LOGE(TAG, "malloc rxbuf failed");
        vTaskDelete(NULL);
        return;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        free(rxbuf);
        vTaskDelete(NULL);
        return;
    }

    // 【关键修改】: 设置 Socket 为非阻塞模式
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in server_addr;
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(CONFIG_ESP32_LISTENING_PORT);

    int err = bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "bind failed: errno=%d", errno);
        close(sock);
        free(rxbuf);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "PID tuner UDP server started on port %d, max msg %d", CONFIG_ESP32_LISTENING_PORT, rx_len);

    // 遥测周期：50ms (20Hz)
    const TickType_t xDelay = pdMS_TO_TICKS(50);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        /*
         * 1. 【接收逻辑 - 非阻塞轮询】
         * get_pid/set_pid
         */
        struct sockaddr_in src_addr;
        socklen_t addrlen = sizeof(src_addr);
        ssize_t len = recvfrom(sock, rxbuf, rx_len - 1, 0, (struct sockaddr*)&src_addr, &addrlen);
        if (len > 0) {
            // 收到数据，进行处理
            if (len >= rx_len) len = rx_len - 1;
            rxbuf[len] = '\0';

            ESP_LOGD(TAG, "Received %d bytes from %s:%d", (int)len, inet_ntoa(src_addr.sin_addr), ntohs(src_addr.sin_port));
            ESP_LOGI(TAG, "JSON: %s", rxbuf);

            handle_json_payload(rxbuf, sock, &src_addr, addrlen);
        } else if (len < 0 && errno != EWOULDBLOCK) {
            // EWOULDBLOCK 表示没有数据，是正常情况。
            // 只有其他错误才需要记录
            ESP_LOGE(TAG, "recvfrom error: errno=%d", errno);
        }

        // ----------------------------------------
        // 2. 【发送逻辑 - 周期发送】
        // ----------------------------------------
        send_telemetry_data(sock);

        // ----------------------------------------
        // 3. 【精确周期延时】
        // ----------------------------------------
        // vTaskDelayUntil 保证任务每隔 xDelay 精确运行一次，弥补任务运行时间带来的误差
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }

    close(sock);
    free(rxbuf);
    vTaskDelete(NULL);
}

// 启动函数
void pid_tuner_start(void) {
    xTaskCreate(pid_udp_task, "pid_udp_task", 4096, NULL, 2, NULL);
}
#endif