#ifndef MBED_LINE_H
#define MBED_LINE_H

#include "mbed.h"

#define RC 0.5
#define REACTION_AVERAGE_NUMBER_OF_TIMES 10000

class line {
     public:
      line(PinName led_pin_, PinName front_1_, PinName front_2_, PinName right_1_, PinName right_2_, PinName back_1_, PinName back_2_, PinName left_1_, PinName left_2_);

      void led(bool intensity);
      void read();
      void set();
      bool check_all();
      bool check_front();
      bool check_right();
      bool check_back();
      bool check_left();
      bool check(uint8_t line_number);

      uint16_t threshold;

     private:
      DigitalOut led_pin;
      AnalogIn front_1;
      AnalogIn front_2;
      AnalogIn right_1;
      AnalogIn right_2;
      AnalogIn back_1;
      AnalogIn back_2;
      AnalogIn left_1;
      AnalogIn left_2;

      bool check_tf[8];
      uint16_t value[8];
      uint16_t pre_value[8];
      uint32_t reaction[8];
};

#endif