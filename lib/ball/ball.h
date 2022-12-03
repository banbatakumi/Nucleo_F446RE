#ifndef MBED_BALL_H
#define MBED_BALL_H

#include "mbed.h"

#define PI 3.1415926535   // 円周率

#define READ_NUMBER_OF_TIME 300   // ボールセンサを読む回数
#define IR_NUM 8   // IRセンサの個数
#define SAMPLE_NUMBER 25   // n回分の過去に読んだ値の使用
class ball {
     public:
      ball(PinName ir_0_, PinName ir_1_, PinName ir_2_, PinName ir_3_, PinName ir_4_, PinName ir_5_, PinName ir_6_, PinName ir_7_);
      void read();
      int16_t angle;
      int16_t distance;
      float rc;

     private:
      DigitalIn ir_0;
      DigitalIn ir_1;
      DigitalIn ir_2;
      DigitalIn ir_3;
      DigitalIn ir_4;
      DigitalIn ir_5;
      DigitalIn ir_6;
      DigitalIn ir_7;

      uint16_t pre_value[IR_NUM];
      uint16_t sample_value[SAMPLE_NUMBER][IR_NUM];
      uint32_t value[IR_NUM];
      int32_t result_vector_x, result_vector_y;
      float unit_vector_x[IR_NUM];
      float unit_vector_y[IR_NUM];
};

#endif