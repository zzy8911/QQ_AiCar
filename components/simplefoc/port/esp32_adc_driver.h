#ifndef SIMPLEFOC_ESP32_HAL_ADC_DRIVER_H_
#define SIMPLEFOC_ESP32_HAL_ADC_DRIVER_H_

#include <cstdint>
#include "sdkconfig.h"

/*
 * Get ADC value for pin
 * */
uint16_t adcRead(uint8_t pin);

/*
 * Initialize ADC for pin
 */
bool adcInit(uint8_t pin);

#ifdef CONFIG_SIMPLEFOC_ADC_MODE_CONTINUOUS
void adcStart();
#endif

#endif