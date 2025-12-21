#ifndef PID_H
#define PID_H


#include "../port/esp_hal_misc.h"
#include "foc_utils.h"
#include "base_classes/IPid.h"

/**
 *  PID controller class
 */
class PIDController : public IPID
{
public:
    /**
     *  
     * @param P - Proportional gain 
     * @param I - Integral gain
     * @param D - Derivative gain 
     * @param ramp - Maximum speed of change of the output value
     * @param limit - Maximum output value
     */
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    float operator() (float error);
    void reset();
    void resetIntegral() { integral_prev = 0; };

    float output_ramp; //!< Maximum speed of change of the output value
    float limit; //!< Maximum output value

protected:
    float error_prev; //!< last tracking error value
    float output_prev;  //!< last pid output value
    float integral_prev; //!< last integral component value
    unsigned long timestamp_prev; //!< Last execution timestamp
};

#endif // PID_H