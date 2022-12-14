#include "ball.h"

#include "mbed.h"

ball::ball(PinName ir_0_, PinName ir_1_, PinName ir_2_, PinName ir_3_, PinName ir_4_, PinName ir_5_, PinName ir_6_, PinName ir_7_) : ir_0(ir_0_), ir_1(ir_1_), ir_2(ir_2_), ir_3(ir_3_), ir_4(ir_4_), ir_5(ir_5_), ir_6(ir_6_), ir_7(ir_7_) {
      rc = 0.25;
      for (uint8_t count = 0; count < IR_NUM; count++) {
            unit_vector_x[count] = cos((count * 360.000 / IR_NUM) * PI / 180.000);
            unit_vector_y[count] = sin((count * 360.000 / IR_NUM) * PI / 180.000);
      }
}

void ball::read() {
      for (uint8_t count = 0; count < IR_NUM; count++) value[count] = 0;
      for (uint16_t count = 0; count < READ_NUMBER_OF_TIME; count++) {
            value[0] += ir_0;
            value[1] += ir_1;
            value[2] += ir_2;
            value[3] += ir_3;
            value[4] += ir_4;
            value[5] += ir_5;
            value[6] += ir_6;
            value[7] += ir_7;
      }

      result_vector_x = 0;
      result_vector_y = 0;
      for (uint8_t count = 0; count < IR_NUM; count++) {
            for (uint8_t i = SAMPLE_NUMBER - 1; i > 0; i--) sample_value[i][count] = sample_value[i - 1][count];
            sample_value[0][count] = value[count];
            value[count] = 0;
            for (uint8_t i = 0; i < SAMPLE_NUMBER; i++) value[count] += sample_value[i][count];
            value[count] = (READ_NUMBER_OF_TIME * SAMPLE_NUMBER - value[count]) * (100.000 / (READ_NUMBER_OF_TIME * SAMPLE_NUMBER));
            value[count] = value[count] * (1 - rc) + pre_value[count] * rc;
            pre_value[count] = value[count];
            result_vector_x += value[count] * unit_vector_x[count];
            result_vector_y += value[count] * unit_vector_y[count];
      }

      angle = atan2(result_vector_y, result_vector_x) / PI * 180.000 + 0.5;
      distance = sqrt(pow(result_vector_x, 2) + pow(result_vector_y, 2)) + 0.5;
      if (distance > 100) distance = 100;
}