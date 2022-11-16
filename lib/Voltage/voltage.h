#ifndef MBED_VOLTAGE_H
#define MBED_VOLTAGE_H

#include "mbed.h"

#define VOLTAGE_RC 0.95

class voltage {
     public:
      voltage(PinName voltage_pin_);
      float get();

     private:
      AnalogIn voltage_pin;

      float voltage_value;
      float pre_voltage_value;

      Timer sampling_timer;
};

#endif