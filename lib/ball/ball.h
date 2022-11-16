#ifndef MBED_BALL_H
#define MBED_BALL_H

#include "mbed.h"

#define IR_READ_NUMBER_OF_TIME 250   // ボールセンサを読む回数
#define IR_NUM 8   // IRセンサの個数
#define IR_SAMPLE_NUMBER 25   // n回分の過去に読んだ値の使用
#define PI 3.14159

class ball {
     public:
      ball(PinName ir_0_, PinName ir_1_, PinName ir_2_, PinName ir_3_, PinName ir_4_, PinName ir_5_, PinName ir_6_, PinName ir_7_);
      void get(int16_t* ball_angle, int16_t* ball_distance, float ball_rc);

     private:
      DigitalIn ir_0;
      DigitalIn ir_1;
      DigitalIn ir_2;
      DigitalIn ir_3;
      DigitalIn ir_4;
      DigitalIn ir_5;
      DigitalIn ir_6;
      DigitalIn ir_7;

      uint32_t ir_value[IR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0};
      uint16_t pre_ir_value[IR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0};
      uint16_t sample_ir_value[IR_SAMPLE_NUMBER][IR_NUM];
      int32_t result_vector_x = 0, result_vector_y = 0;
      float unit_vector_x[IR_NUM] = {0, 0.707, 1, 0.707, 0, -0.707, -1, -0.707};
      float unit_vector_y[IR_NUM] = {1, 0.707, 0, -0.707, -1, -0.707, 0, 0.707};
};

#endif