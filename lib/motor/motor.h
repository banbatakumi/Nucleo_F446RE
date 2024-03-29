#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "mbed.h"

#define PI 3.1415926535   // 円周率

#define MOTOR_FREQUENCY 20000   // モーターのPWM周波数
#define MIN_BRAKE 0  // モーターの最小値ブレーキ
#define POWER_LIMIT 75   // モーターの最大パワー
#define KP 1.000   // 姿勢制御比例ゲイン
#define KD 5.000   // 姿制御微分ゲイン
#define PD_LIMIT 55   // 姿勢制御の最大パワー
#define MOVING_AVERAGE_COUNT_NUMBER 5   // 移動平均フィルタの回数
#define D_PERIODO 0.01
class motor {
     public:
      motor(PinName motor_1_1_, PinName motor_1_2_, PinName motor_2_1_, PinName motor_2_2_, PinName motor_3_1_, PinName motor_3_2_, PinName motor_4_1_, PinName motor_4_2_);
      void run(int16_t move_angle, int16_t move_speed, int8_t robot_angle = 0);
      void set_pwm();
      void brake(uint16_t brake_time = 0);
      void free();

      int16_t motor_1();
      int16_t motor_2();
      int16_t motor_3();
      int16_t motor_4();
      int16_t move_angle();
      int16_t move_speed();

      int16_t yaw;

     private:
      PwmOut motor_1_1;
      PwmOut motor_1_2;
      PwmOut motor_2_1;
      PwmOut motor_2_2;
      PwmOut motor_3_1;
      PwmOut motor_3_2;
      PwmOut motor_4_1;
      PwmOut motor_4_2;

      int16_t power[4], pre_power[4], tmp_power[4][MOVING_AVERAGE_COUNT_NUMBER], maximum_power;
      int16_t pre_p, pd, p, d, angle, speed;

      uint8_t moving_average_count;

      Timer d_timer;
};

#endif