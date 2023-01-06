#include "voltage.h"

#include "mbed.h"

voltage::voltage(PinName voltage_pin_) : voltage_pin(voltage_pin_) {
      voltage_value = 10;
      sampling_timer.start();
}

void voltage::read() {
      if (sampling_timer > 0.5) {
            pre_voltage_value = voltage_value;
            voltage_value = (voltage_pin.read_u16() * 5.0 / 1023.0 / 23.0) * (1 - VOLTAGE_RC) + pre_voltage_value * VOLTAGE_RC;   // 電圧のRCフィルタリング
            sampling_timer.reset();
      }
}

float voltage::get() {
      return voltage_value;
}