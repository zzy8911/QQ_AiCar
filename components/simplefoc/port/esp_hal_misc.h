/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_platform.h"
#include "esp_hal_misc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Get time in ms since boot.
 *
 * @return number of microseconds since underlying timer has been started
 */

unsigned long micros();

/**
 * @brief Get time in us since boot.
 *
 * @return number of milliseconds since underlying timer has been started
 */
unsigned long millis();

/**
 * @brief Delay us.
 *
 * @param us microsecond
 */
void delayMicroseconds(uint32_t us);

/**
 * @brief Rtos ms delay function
 *
 * @param ms millisecond
 */
void delay(uint32_t ms);

/**
 * @brief Minimum function.
 *
 * @param a numbers that need to be compared
 * @param b numbers that need to be compared
 * @return minimum value
 */
float min(float a, float b);

/** 
 * Function implementing delay() function in milliseconds 
 * - blocking function
 * - hardware specific

 * @param ms number of milliseconds to wait
 */
void _delay(unsigned long ms);

/** 
 * Function implementing timestamp getting function in microseconds
 * hardware specific
 */
unsigned long _micros();

#ifdef __cplusplus
}
#endif
