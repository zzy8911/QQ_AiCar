#include "../current_sense/hardware_api.h"

#include "esp32_adc_driver.h"
#include "esp_log.h"
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>


typedef struct ESP32MCPWMCurrentSenseParams {
  int pins[3];
  float adc_voltage_conv;
} ESP32MCPWMCurrentSenseParams;


/**
 *  Inline adc reading implementation 
*/
// function reading an ADC value and returning the read voltage
float _readADCVoltageInline(const int pinA, const void* cs_params){
  uint32_t raw_adc = adcRead(pinA);
  return raw_adc * ((ESP32MCPWMCurrentSenseParams*)cs_params)->adc_voltage_conv;
}

// function reading an ADC value and returning the read voltage
void* _configureADCInline(const void* driver_params, const int pinA, const int pinB, const int pinC){
  _UNUSED(driver_params);

  ESP32MCPWMCurrentSenseParams* params = new ESP32MCPWMCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (_ADC_VOLTAGE)/(_ADC_RESOLUTION)
  };

  for (int i = 0; i < 3; i++){
    if(_isset(params->pins[i])){
      if(!adcInit(params->pins[i])) {
        ESP_LOGE("InlineCurrentSense", "ERROR: Failed to initialise ADC pin: %d, maybe not an ADC pin?", params->pins[i]);
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
      }
    }
  }

  return params;
}

// function starting the ADC for inline current sensing
void _startADCInline(){
  adcStart();
}
