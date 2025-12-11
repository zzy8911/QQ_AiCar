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

    IPID* target = m.getPID(type);
    float v;
    if (get_number_by_keys(obj, {"p","kp","P","KP"}, v)) target->P = v;
    if (get_number_by_keys(obj, {"i","ki","I","KI"}, v)) target->I = v;
    if (get_number_by_keys(obj, {"d","kd","D","KD"}, v)) target->D = v;

    ESP_LOGI(TAG, "Applied PID '%s' -> kp=%.6f ki=%.6f kd=%.6f",
             name.c_str(), target->P, target->I, target->D);
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
    if (cmd && cJSON_IsString(cmd) && strcmp(cmd->valuestring, "set_pid") == 0) {
        cJSON *payload = cJSON_GetObjectItemCaseSensitive(root, "payload");
        if (payload && cJSON_IsObject(payload)) {
            // 遍历 payload 的成员（stb, vel, steer...）
            cJSON *child = payload->child;
            while (child) {
                if (child->string) {
                    parse_pid_obj_and_apply(child->string, child);
                }
                child = child->next;
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
            cJSON_Delete(root);
            return;
        }
    }

    ESP_LOGW(TAG, "Unknown JSON format");
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
        10000 / portTICK_PERIOD_MS   // 10 秒超时，避免卡死
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

    struct sockaddr_in server_addr;
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(CONFIG_PID_TUNER_UDP_PORT);

    int err = bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "bind failed: errno=%d", errno);
        close(sock);
        free(rxbuf);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "PID tuner UDP server started on port %d, max msg %d", CONFIG_PID_TUNER_UDP_PORT, rx_len);

    while (1) {
        struct sockaddr_in src_addr;
        socklen_t addrlen = sizeof(src_addr);
        ssize_t len = recvfrom(sock, rxbuf, rx_len - 1, 0, (struct sockaddr*)&src_addr, &addrlen);
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom error: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (len == 0) continue;

        if (len >= rx_len) len = rx_len - 1;
        rxbuf[len] = '\0';

        ESP_LOGD(TAG, "Received %d bytes from %s:%d", (int)len, inet_ntoa(src_addr.sin_addr), ntohs(src_addr.sin_port));
        ESP_LOGI(TAG, "JSON: %s", rxbuf);

        handle_json_payload(rxbuf, sock, &src_addr, addrlen);
    }

    close(sock);
    free(rxbuf);
    vTaskDelete(NULL);
}

// 启动函数
void pid_tuner_start(void) {
    xTaskCreate(pid_udp_task, "pid_udp_task", 4096, NULL, 5, NULL);
}
#endif