#include "line.h"

#include "mbed.h"

line::line(PinName front_1_, PinName front_2_, PinName right_1_, PinName right_2_, PinName back_1_, PinName back_2_, PinName left_1_, PinName left_2_)
    : front_1(front_1_), front_2(front_2_), right_1(right_1_), right_2(right_2_), back_1(back_1_), back_2(back_2_), left_1(left_1_), left_2(left_2_) {
      threshold = 30;
}

void line::read() {
      value[0] = front_1.read_u16() * 0.01;
      value[1] = front_2.read_u16() * 0.01;
      value[2] = right_1.read_u16() * 0.01;
      value[3] = right_2.read_u16() * 0.01;
      value[4] = back_1.read_u16() * 0.01;
      value[5] = back_2.read_u16() * 0.01;
      value[6] = left_1.read_u16() * 0.01;
      value[7] = left_2.read_u16() * 0.01;

      for (uint8_t count = 0; count < 8; count++) {
            value[count] = value[count] * (1 - RC) + pre_value[count] * RC;
            pre_value[count] = value[count];
            check_tf[count] = value[count] > reaction[count] + threshold ? 1 : 0;   // 反応したかのチェック
      }
}

void line::set() {
      wait_us(100000);
      for (uint8_t count = 0; count < 8; count++) reaction[count] = 0;
      for (uint16_t count = 0; count < REACTION_AVERAGE_NUMBER_OF_TIMES; count++) {
            reaction[0] += front_1.read_u16() * 0.01;
            reaction[1] += front_2.read_u16() * 0.01;
            reaction[2] += right_1.read_u16() * 0.01;
            reaction[3] += right_2.read_u16() * 0.01;
            reaction[4] += back_1.read_u16() * 0.01;
            reaction[5] += back_2.read_u16() * 0.01;
            reaction[6] += left_1.read_u16() * 0.01;
            reaction[7] += left_2.read_u16() * 0.01;
      }
      for (uint8_t count = 0; count < 8; count++) reaction[count] = reaction[count] / REACTION_AVERAGE_NUMBER_OF_TIMES;
}

bool line::check_all() { return check_tf[0] == 1 || check_tf[1] == 1 || check_tf[2] == 1 || check_tf[3] == 1 || check_tf[4] == 1 || check_tf[5] == 1 || check_tf[6] == 1 || check_tf[7] == 1 ? 1 : 0; }

bool line::check_front() { return check_tf[0] == 1 || check_tf[1] == 1 ? 1 : 0; }

bool line::check_right() { return check_tf[2] == 1 || check_tf[3] == 1 ? 1 : 0; }

bool line::check_back() { return check_tf[4] == 1 || check_tf[5] == 1 ? 1 : 0; }

bool line::check_left() { return check_tf[6] == 1 || check_tf[7] == 1 ? 1 : 0; }

bool line::check(uint8_t line_number) { return check_tf[line_number] == 1 ? 1 : 0; }