#include "voltage.h"

#include "mbed.h"

voltage::voltage(PinName voltage_pin_) : voltage_pin(voltage_pin_) {
      voltage_value = 10;
      sampling_timer.start();
}

float voltage::get() {
      if (sampling_timer > 0.1) {
            pre_voltage_value = voltage_value;
            voltage_value = (voltage_pin.read_u16() * 5.0 / 1023.0 / 23.0) * (1 - VOLTAGE_RC) + pre_voltage_value * VOLTAGE_RC;   // 電圧のRCフィルタリング
            sampling_timer.reset();
      }
      return voltage_value;
}