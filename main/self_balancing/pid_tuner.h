#pragma once

#include "sdkconfig.h"

#ifdef CONFIG_ENABLE_CONSOLE
void register_pid_cmd();
#endif

#ifdef CONFIG_PID_TUNER_ENABLE
void pid_tuner_start(void);
#endif