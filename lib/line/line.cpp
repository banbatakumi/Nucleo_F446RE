#include "line.h"

#include "mbed.h"

line::line(PinName led_pin_, PinName front_1_, PinName front_2_, PinName right_1_, PinName right_2_, PinName back_1_, PinName back_2_, PinName left_1_, PinName left_2_)
    : led_pin(led_pin_), front_1(front_1_), front_2(front_2_), right_1(right_1_), right_2(right_2_), back_1(back_1_), back_2(back_2_), left_1(left_1_), left_2(left_2_) {
      threshold = 60;
}

void line::led(bool intensity) {
      led_pin = intensity;
}

void line::read() {
      value[1] = front_2.read() * 1000;
      value[0] = front_1.read() * 1000;
      value[2] = right_1.read() * 1000;
      value[3] = right_2.read() * 1000;
      value[4] = back_1.read() * 1000;
      value[5] = back_2.read() * 1000;
      value[6] = left_1.read() * 1000;
      value[7] = left_2.read() * 1000;

      if (moving_average_count == MOVING_AVERAGE_COUNT_NUMBER) moving_average_count = 0;
      for (uint8_t count = 0; count < 8; count++) {
            tmp_value[count][moving_average_count] = value[count];
            value[count] = 0;
            for (uint8_t count_1 = 0; count_1 < MOVING_AVERAGE_COUNT_NUMBER; count_1++) value[count] += tmp_value[count][count_1];
            value[count] /= MOVING_AVERAGE_COUNT_NUMBER;
            check_tf[count] = value[count] > reaction[count] + threshold ? 1 : 0;   // 反応したかのチェック
      }
      moving_average_count++;
}

void line::set() {
      led_pin = 1;
      wait_us(100000);
      for (uint8_t count = 0; count < 8; count++) reaction[count] = 0;
      for (uint16_t count = 0; count < REACTION_AVERAGE_NUMBER_OF_TIMES; count++) {
            reaction[0] += front_1.read() * 1000;
            reaction[1] += front_2.read() * 1000;
            reaction[2] += right_1.read() * 1000;
            reaction[3] += right_2.read() * 1000;
            reaction[4] += back_1.read() * 1000;
            reaction[5] += back_2.read() * 1000;
            reaction[6] += left_1.read() * 1000;
            reaction[7] += left_2.read() * 1000;
      }
      for (uint8_t count = 0; count < 8; count++) reaction[count] = reaction[count] / REACTION_AVERAGE_NUMBER_OF_TIMES;
      led_pin = 0;
}

bool line::check_all() {
      if (check_tf[0] == 1 || check_tf[1] == 1 || check_tf[2] == 1 || check_tf[3] == 1 || check_tf[4] == 1 || check_tf[5] == 1 || check_tf[6] == 1 || check_tf[7] == 1) {
            return 1;
      } else {
            return 0;
      }
}

bool line::check_front() {
      return check_tf[0] == 1 || check_tf[1] == 1 ? 1 : 0;
}

bool line::check_right() {
      return check_tf[2] == 1 || check_tf[3] == 1 ? 1 : 0;
}

bool line::check_back() {
      return check_tf[4] == 1 || check_tf[5] == 1 ? 1 : 0;
}

bool line::check_left() {
      return check_tf[6] == 1 || check_tf[7] == 1 ? 1 : 0;
}

bool line::check(uint8_t line_number) {
      return check_tf[line_number] == 1 ? 1 : 0;
}

uint16_t line::get_value(uint8_t line_number) {
      return value[line_number];
}