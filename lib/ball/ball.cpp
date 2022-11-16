#include "ball.h"

#include "mbed.h"

ball::ball(PinName ir_0_, PinName ir_1_, PinName ir_2_, PinName ir_3_, PinName ir_4_, PinName ir_5_, PinName ir_6_, PinName ir_7_) : ir_0(ir_0_), ir_1(ir_1_), ir_2(ir_2_), ir_3(ir_3_), ir_4(ir_4_), ir_5(ir_5_), ir_6(ir_6_), ir_7(ir_7_) {
}

void ball::get(int16_t* ball_angle, int16_t* ball_distance, float ball_rc) {
      for (uint8_t count = 0; count < IR_NUM; count++) ir_value[count] = 0;
      for (uint16_t count = 0; count < IR_READ_NUMBER_OF_TIME; count++) {
            ir_value[0] += ir_0;
            ir_value[1] += ir_1;
            ir_value[2] += ir_2;
            ir_value[3] += ir_3;
            ir_value[4] += ir_4;
            ir_value[5] += ir_5;
            ir_value[6] += ir_6;
            ir_value[7] += ir_7;
      }

      result_vector_x = 0;
      result_vector_y = 0;
      for (uint8_t count = 0; count < IR_NUM; count++) {
            for (uint8_t i = IR_SAMPLE_NUMBER - 1; i > 0; i--) sample_ir_value[i][count] = sample_ir_value[i - 1][count];
            sample_ir_value[0][count] = ir_value[count];
            ir_value[count] = 0;
            for (uint8_t i = 0; i < IR_SAMPLE_NUMBER; i++) ir_value[count] += sample_ir_value[i][count];
            ir_value[count] = (IR_READ_NUMBER_OF_TIME * IR_SAMPLE_NUMBER - ir_value[count]) * (100.000 / (IR_READ_NUMBER_OF_TIME * IR_SAMPLE_NUMBER));
            ir_value[count] = ir_value[count] * (1 - ball_rc) + pre_ir_value[count] * ball_rc;
            pre_ir_value[count] = ir_value[count];
            result_vector_x += ir_value[count] * unit_vector_x[count];
            result_vector_y += ir_value[count] * unit_vector_y[count];
      }

      *ball_angle = atan2(result_vector_x, result_vector_y) / PI * 180.000 + 0.5;
      *ball_distance = sqrt(pow(result_vector_x, 2) + pow(result_vector_y, 2)) + 0.5;
      if (*ball_distance > 100) *ball_distance = 100;
}