#include "background_task.h"

#include <esp_log.h>
#include <esp_task_wdt.h>

#define TAG "BackgroundTask"

BackgroundTask::BackgroundTask(uint32_t stack_size) {
    // 从 PSRAM 分配任务栈
    background_task_stack_ = (StackType_t*) heap_caps_malloc(
        stack_size * sizeof(StackType_t),
        MALLOC_CAP_SPIRAM
    );
    if (!background_task_stack_) {
        ESP_LOGE("BackgroundTask", "Failed to allocate PSRAM stack");
        return;
    }

    background_task_handle_ = xTaskCreateStatic(
        [](void* arg) {
            static_cast<BackgroundTask*>(arg)->BackgroundTaskLoop();
        },
        "background_task",
        stack_size,
        this,
        2,
        background_task_stack_,
        &background_task_tcb_
    );
    if (background_task_handle_ == nullptr) {
        ESP_LOGE("BackgroundTask", "Failed to create task");
    }
    if (esp_ptr_external_ram(background_task_stack_)) {
        ESP_LOGI("BackgroundTask", "Stack is in PSRAM");
    }
}

BackgroundTask::~BackgroundTask() {
    if (background_task_handle_ != nullptr) {
        vTaskDelete(background_task_handle_);
    }
    if (background_task_stack_) {
        heap_caps_free(background_task_stack_);
        background_task_stack_ = nullptr;
    }
}

bool BackgroundTask::Schedule(std::function<void()> callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (waiting_for_completion_ > 0) {
        return false;
    }
    if (active_tasks_ >= 30) {
        int free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        if (free_sram < 10000) {
            ESP_LOGW(TAG, "active_tasks_ == %d, free_sram == %u", active_tasks_, free_sram);
            return false;
        }
    }
    active_tasks_++;
    background_tasks_.emplace_back([this, cb = std::move(callback)]() {
        cb();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            active_tasks_--;
            if (background_tasks_.empty() && active_tasks_ == 0) {
                condition_variable_.notify_all();
            }
        }
    });
    condition_variable_.notify_all();
    return true;
}

void BackgroundTask::WaitForCompletion() {
    std::unique_lock<std::mutex> lock(mutex_);
    waiting_for_completion_++;
    condition_variable_.wait(lock, [this]() {
        return background_tasks_.empty() && active_tasks_ == 0;
    });
    waiting_for_completion_--;
}

void BackgroundTask::BackgroundTaskLoop() {
    ESP_LOGI(TAG, "background_task started");
    while (true) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_variable_.wait(lock, [this]() { return !background_tasks_.empty(); });
        
        std::list<std::function<void()>> tasks = std::move(background_tasks_);
        lock.unlock();

        for (auto& task : tasks) {
            task();
        }
    }
}
