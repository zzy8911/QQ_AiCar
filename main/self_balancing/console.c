#include "console.h"

#ifdef CONFIG_ENABLE_CONSOLE

static const char *TAG = "Console";

/**
 * @brief 初始化 Console 但不启动 REPL
 */
esp_err_t console_init_only(esp_console_repl_t **ret_repl, const char *prompt)
{
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = prompt;
    repl_cfg.max_cmdline_length = 128;

    esp_console_dev_uart_config_t dev_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    return esp_console_new_repl_uart(&dev_cfg, &repl_cfg, ret_repl);
}

int cmd_top(int argc, char** argv)
{
#if ( configUSE_TRACE_FACILITY == 1 )
    configRUN_TIME_COUNTER_TYPE ulTotalTime, ulStatsAsPercentage;
    char cStatus;
    ESP_LOGI("System", "------------------TASK INFO----------------------");
    ESP_LOGI("System", "TaskName       State   CPU%%    Prio    Stack  TaskNo. CoreID");
    UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
    TaskStatus_t * pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
    if( pxTaskStatusArray != NULL ) {
        uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalTime );
        ulTotalTime /= 100UL;
        for(UBaseType_t x = 0; x < uxArraySize; x++ ) {
            switch( pxTaskStatusArray[ x ].eCurrentState ) {
                case eRunning:   cStatus = 'X'; break;
                case eReady:     cStatus = 'R'; break;
                case eBlocked:   cStatus = 'B'; break;
                case eSuspended: cStatus = 'S'; break;
                case eDeleted:   cStatus = 'D'; break;
                case eInvalid:
                default:
                    cStatus = ( char ) 0x00;    break;
            }

            if (ulTotalTime > 0UL)
                ulStatsAsPercentage = pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalTime;
            else
                ulStatsAsPercentage = 0UL;
            ESP_LOGI("System", "%-16s %c\t%lu%%\t%u\t%u\t%u\t%d",
                pxTaskStatusArray[ x ].pcTaskName,
                cStatus,
                ulStatsAsPercentage,
                ( unsigned int ) pxTaskStatusArray[ x ].uxCurrentPriority,
                ( unsigned int ) pxTaskStatusArray[ x ].usStackHighWaterMark,
                ( unsigned int ) pxTaskStatusArray[ x ].xTaskNumber,
                ( int ) ( pxTaskStatusArray[ x ].xCoreID == tskNO_AFFINITY ) ? -1 : pxTaskStatusArray[ x ].xCoreID );
        }
        vPortFree( pxTaskStatusArray );
    }

    int dfree = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    int dtotal = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    int ifree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    int itotal = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI("System", "------------------HEAP INFO----------------------");
    ESP_LOGI("System", "[Default heap]  Free: %10d, Used: %10d, Total:%10d", dfree, dtotal-dfree, dtotal);
    ESP_LOGI("System", "[Internal heap] Free: %10d, Used: %10d, Total:%10d", ifree, itotal-ifree, itotal);
    ESP_LOGI("System", "-------------------------------------------------");
#endif
    return 0;
}

#if (CONFIG_HEAP_TASK_TRACKING)
#include "esp_heap_task_info.h"
#define MAX_TASK_NUM 30                         // Max number of per tasks info that it can store
#define MAX_BLOCK_NUM 30                        // Max number of per block info that it can store
static size_t s_prepopulated_num = 0;
static heap_task_totals_t *s_totals_arr = NULL;
static heap_task_block_t *s_block_arr = NULL;

int cmd_task_heap(int argc, char** argv)
{
    if (!s_totals_arr)
        s_totals_arr = (heap_task_totals_t *) calloc(MAX_TASK_NUM, sizeof(heap_task_totals_t));
    if (!s_block_arr)
        s_block_arr = (heap_task_block_t *) calloc(MAX_BLOCK_NUM, sizeof(heap_task_block_t));

    heap_task_info_params_t heap_info = {};
    heap_info.caps[0] = MALLOC_CAP_INTERNAL;        // Gets heap with MALLOC_CAP_INTERNAL capabilities
    heap_info.mask[0] = MALLOC_CAP_INTERNAL;
    heap_info.caps[1] = MALLOC_CAP_SPIRAM;       // Gets heap info with MALLOC_CAP_SPIRAM capabilities
    heap_info.mask[1] = MALLOC_CAP_SPIRAM;
    heap_info.tasks = NULL;                     // Passing NULL captures heap info for all tasks
    heap_info.num_tasks = 0;
    heap_info.totals = s_totals_arr;            // Gets task wise allocation details
    heap_info.num_totals = &s_prepopulated_num;
    heap_info.max_totals = MAX_TASK_NUM;        // Maximum length of "s_totals_arr"
    heap_caps_get_per_task_info(&heap_info);

    ESP_LOGI("System", "------------------TASK HEAP----------------------");
    ESP_LOGI("System", "Task            Internal   SPIRAM");
    for (int i = 0 ; i < *heap_info.num_totals; i++) {
        ESP_LOGI("System", "%-16s%-11d%-9d",
                heap_info.totals[i].task ? pcTaskGetName(heap_info.totals[i].task) : "Pre-Scheduler" ,
                heap_info.totals[i].size[0],
                heap_info.totals[i].size[1]);
    }
    ESP_LOGI("System", "-------------------------------------------------");

    return 0;
}

int cmd_task_heap_blocks(int argc, char** argv)
{
    if (!s_totals_arr)
        s_totals_arr = (heap_task_totals_t *) calloc(MAX_TASK_NUM, sizeof(heap_task_totals_t));
    if (!s_block_arr)
        s_block_arr = (heap_task_block_t *) calloc(MAX_BLOCK_NUM, sizeof(heap_task_block_t));

    heap_task_info_params_t heap_info = {};
    heap_info.caps[0] = MALLOC_CAP_INTERNAL;        // Gets heap with MALLOC_CAP_INTERNAL capabilities
    heap_info.mask[0] = MALLOC_CAP_INTERNAL;
    heap_info.caps[1] = MALLOC_CAP_SPIRAM;       // Gets heap info with MALLOC_CAP_SPIRAM capabilities
    heap_info.mask[1] = MALLOC_CAP_SPIRAM;
    heap_info.tasks = NULL;                     // Passing NULL captures heap info for all tasks
    heap_info.num_tasks = 0;
    heap_info.totals = s_totals_arr;            // Gets task wise allocation details
    heap_info.num_totals = &s_prepopulated_num;
    heap_info.max_totals = MAX_TASK_NUM;        // Maximum length of "s_totals_arr"
    heap_info.blocks = s_block_arr;             // Gets block wise allocation details. For each block, gets owner task, address and size
    heap_info.max_blocks = MAX_BLOCK_NUM;       // Maximum length of "s_block_arr"
    int num_b = heap_caps_get_per_task_info(&heap_info);
    ESP_LOGI("System", "------------------TASK HEAP----------------------");
    ESP_LOGI("System", "Task            Internal   SPIRAM");
    for (int i = 0 ; i < *heap_info.num_totals; i++) {
        ESP_LOGI("System", "%-16s%-11d%-9d",
                heap_info.totals[i].task ? pcTaskGetName(heap_info.totals[i].task) : "Pre-Scheduler" ,
                heap_info.totals[i].size[0],
                heap_info.totals[i].size[1]);
    }
    ESP_LOGI("System", "------------------BLOCK INFO----------------------");
    ESP_LOGI("System", "Addr            Size       Task");
    for (int i = 0 ; i < num_b; i++) {
        ESP_LOGI("System", "0x%08X\t%-10ld%s",
                (unsigned int)(heap_info.blocks[i].address),
                heap_info.blocks[i].size,
                heap_info.blocks[i].task ? pcTaskGetName(heap_info.blocks[i].task) : "Pre-Scheduler" );
    }
    ESP_LOGI("System", "-------------------------------------------------");
    return 0;
}
#endif

void register_settings_cmds(void)
{
    const esp_console_cmd_t cmds[] = {
        {"top", "Print the performance information", NULL, &cmd_top, NULL, NULL, NULL},
#if (CONFIG_HEAP_TASK_TRACKING)
        {"heap", "Print the task heap usage info", NULL, &cmd_task_heap, NULL, NULL, NULL},
        {"heapb", "Print the task heap block info", NULL, &cmd_task_heap_blocks, NULL, NULL, NULL},
#endif
    };

    for (int i = 0; i < sizeof(cmds)/sizeof(cmds[0]); ++i) {
        ESP_ERROR_CHECK( esp_console_cmd_register(&cmds[i]) );
    }
}

/**
 * @brief 初始化并启动 Console
 */
esp_err_t console_start(const char *prompt)
{
    esp_console_repl_t *repl = NULL;

    ESP_LOGI(TAG, "Init console…");

    ESP_ERROR_CHECK(console_init_only(&repl, prompt));

    // 注册命令
    register_settings_cmds();
    esp_console_register_help_command();

    ESP_LOGI(TAG, "Start console REPL…");

    return esp_console_start_repl(repl);
}

#endif  // CONFIG_ENABLE_CONSOLE
