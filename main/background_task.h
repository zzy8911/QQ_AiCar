#ifndef BACKGROUND_TASK_H
#define BACKGROUND_TASK_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mutex>
#include <list>
#include <condition_variable>
#include <atomic>
#include "esp_heap_caps.h"

class BackgroundTask {
public:
    BackgroundTask(const char* name, UBaseType_t priority, uint32_t stack_size = 4096 * 2);
    ~BackgroundTask();

    bool Schedule(std::function<void()> callback);
    void WaitForCompletion();

private:
    void BackgroundTaskLoop();

    const char* name_;
    StaticTask_t background_task_tcb_;
    TaskHandle_t background_task_handle_ = nullptr;
    StackType_t* background_task_stack_ = nullptr;

    std::mutex mutex_;
    std::condition_variable condition_variable_;
    std::list<std::function<void()>> background_tasks_;

    int active_tasks_ = 0;
    int waiting_for_completion_ = 0;
};

#endif
