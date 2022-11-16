#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "mbed.h"

#define MOTOR_FREQUENCY 30000
#define MIN_BRAKE 10   // モーターの最小値ブレーキ
#define POWER_LIMIT 75   // モーターの最大パワー
#define KP 0.500   // 姿勢制御比例ゲイン
#define KD 100.000   // 姿制御微分ゲイン
#define PD_LIMIT 50   // 姿勢制御の最大パワー
#define POWER_RC 0.1   // モーターのRCフィルタ
#define PI 3.14159

class motor {
     public:
      motor(PinName motor_1_1_, PinName motor_1_2_, PinName motor_2_1_, PinName motor_2_2_, PinName motor_3_1_, PinName motor_3_2_, PinName motor_4_1_, PinName motor_4_2_);
      void run(int16_t move_angle, int16_t move_speed);
      void reset_pwm();
      void brake();

     private:
      PwmOut motor_1_1;
      PwmOut motor_1_2;
      PwmOut motor_2_1;
      PwmOut motor_2_2;
      PwmOut motor_3_1;
      PwmOut motor_3_2;
      PwmOut motor_4_1;
      PwmOut motor_4_2;

      int16_t power[4] = {0, 0, 0, 0};
      int16_t pre_power[4] = {0, 0, 0, 0};
      float maximum_power = 0;
      int16_t pre_p = 0;
      int16_t pd = 0, p = 0, d = 0;
};

#endif