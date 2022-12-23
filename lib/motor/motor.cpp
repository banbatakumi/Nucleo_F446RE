#include "motor.h"

#include "mbed.h"

motor::motor(PinName motor_1_1_, PinName motor_1_2_, PinName motor_2_1_, PinName motor_2_2_, PinName motor_3_1_, PinName motor_3_2_, PinName motor_4_1_, PinName motor_4_2_) : motor_1_1(motor_1_1_), motor_1_2(motor_1_2_), motor_2_1(motor_2_1_), motor_2_2(motor_2_2_), motor_3_1(motor_3_1_), motor_3_2(motor_3_2_), motor_4_1(motor_4_1_), motor_4_2(motor_4_2_) {
      motor_1_1 = 0;
      motor_1_2 = 0;
      motor_2_1 = 0;
      motor_2_2 = 0;
      motor_3_1 = 0;
      motor_3_2 = 0;
      motor_4_1 = 0;
      motor_4_2 = 0;

      d_timer.start();
}

void motor::run(int16_t move_angle, int16_t move_speed, int8_t robot_angle) {
      if (move_speed > POWER_LIMIT) move_speed = POWER_LIMIT;   // 速度が上限を超えていないか
      for (uint8_t count = 0; count < 4; count++) power[count] = sin((move_angle - (45 + count * 90)) * PI / 180.000) * move_speed * (count < 2 ? -1 : 1);   // 角度とスピードを各モーターの値に変更

      // モーターの最大パフォーマンス発揮
      for (uint8_t count = 0; count < 4; count++) maximum_power = maximum_power < abs(power[count]) ? abs(power[count]) : maximum_power;
      for (uint8_t count = 0; count < 4 && move_speed > 0; count++) power[count] *= float(move_speed) / maximum_power;

      // PD姿勢制御
      p = robot_angle - yaw;   // 比例
      if (d_timer.read() > D_PERIODO) {
            d = p - pre_p;   // 微分
            pre_p = p;
            d_timer.reset();
      }
      pd = p * KP + d * KD;
      if (abs(pd) > PD_LIMIT) pd = PD_LIMIT * (abs(pd) / pd);
      for (uint8_t count = 0; count < 4; count++) {
            power[count] += count < 2 ? -pd : pd;
            power[count] = abs(power[count]) > POWER_LIMIT ? POWER_LIMIT * (abs(power[count]) / power[count]) : power[count];   // モーターの上限値超えた場合の修正
            power[count] = power[count] * (1 - POWER_RC) + pre_power[count] * POWER_RC;
            pre_power[count] = power[count];
      }

      motor_1_1 = abs(power[0]) < MIN_BRAKE ? 1 : (power[0] > 0 ? power[0] * 0.01 : 0);
      motor_1_2 = abs(power[0]) < MIN_BRAKE ? 1 : (power[0] < 0 ? power[0] * -0.01 : 0);
      motor_2_1 = abs(power[1]) < MIN_BRAKE ? 1 : (power[1] > 0 ? power[1] * 0.01 : 0);
      motor_2_2 = abs(power[1]) < MIN_BRAKE ? 1 : (power[1] < 0 ? power[1] * -0.01 : 0);
      motor_3_1 = abs(power[2]) < MIN_BRAKE ? 1 : (power[2] > 0 ? power[2] * 0.01 : 0);
      motor_3_2 = abs(power[2]) < MIN_BRAKE ? 1 : (power[2] < 0 ? power[2] * -0.01 : 0);
      motor_4_1 = abs(power[3]) < MIN_BRAKE ? 1 : (power[3] > 0 ? power[3] * 0.01 : 0);
      motor_4_2 = abs(power[3]) < MIN_BRAKE ? 1 : (power[3] < 0 ? power[3] * -0.01 : 0);
}

void motor::set_pwm() {
      motor_1_1.period_us(MOTOR_FREQUENCY);
      motor_1_2.period_us(MOTOR_FREQUENCY);
      motor_2_1.period_us(MOTOR_FREQUENCY);
      motor_2_2.period_us(MOTOR_FREQUENCY);
      motor_3_1.period_us(MOTOR_FREQUENCY);
      motor_3_2.period_us(MOTOR_FREQUENCY);
      motor_4_1.period_us(MOTOR_FREQUENCY);
      motor_4_2.period_us(MOTOR_FREQUENCY);
}

void motor::brake(uint16_t brake_time) {
      motor_1_1 = 1;
      motor_1_2 = 1;
      motor_2_1 = 1;
      motor_2_2 = 1;
      motor_3_1 = 1;
      motor_3_2 = 1;
      motor_4_1 = 1;
      motor_4_2 = 1;
      wait_us(brake_time * 1000);
}

void motor::free() {
      motor_1_1 = 0;
      motor_1_2 = 0;
      motor_2_1 = 0;
      motor_2_2 = 0;
      motor_3_1 = 0;
      motor_3_2 = 0;
      motor_4_1 = 0;
      motor_4_2 = 0;
}