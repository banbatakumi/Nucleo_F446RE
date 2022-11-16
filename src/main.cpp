// RCJ 2022 season program for Takumi Banba

#include "Adafruit_SSD1306.h"
#include "Serial.h"
#include "ball.h"
#include "math.h"
#include "mbed.h"
#include "motor.h"
#include "speaker.h"
#include "voltage.h"

// 定義
#define PI 3.14159   // 円周率

#define DISPLAY_UPDATE_RATE 0.05   // OLEDの更新時間

#define MOTOR_FREQUENCY 25000   // モーターのPWM周波数
#define MIN_BRAKE 10   // モーターの最小値ブレーキ
#define MOTOR_LIMIT 75   // モーターの最大パワー
#define KP 0.500   // 姿勢制御比例ゲイン
#define KD 100.000   // 姿制御微分ゲイン
#define PD_LIMIT 50   // 姿勢制御の最大パワー
#define MOTOR_RC 0.1   // モーターのRCフィルタ

#define LINE_REACTION_AVERAGE_NUMBER_OF_TIMES 5000   // ラインの閾値を決める平均回数
#define LINE_RC 0.25

#define IR_READ_NUMBER_OF_TIME 250   // ボールセンサを読む回数
#define IR_NUM 8   // IRセンサの個数
#define IR_SAMPLE_NUMBER 25   // n回分の過去に読んだ値の使用

#define VOLTAGE_RC 0.95   // 電圧監視のRCフィルタ強さ
#define VOLTAGE_STOP_COUNT_NUMBER_OF_TIMES 500   // 停止する電圧低下の回数
#define HIGH_VOLTAGE 8.5
#define MEDIUM_VOLTAGE 8.0

#define BALL_FOLLOW_RANGE 45.000   // 回り込み時にボールを追い始める角度

// I2Cの設定
I2C i2c(PB_9, PB_8);
Adafruit_SSD1306_I2c oled(i2c, D9, SSD_I2C_ADDRESS, 64, 128);

speaker _speaker(PC_9);

ball _ball(PB_12, PB_13, PB_14, PC_4, PA_15, PC_11, PD_2, PC_8);

voltage _voltage(PA_5);

motor _motor(PB_3, PA_10, PA_11, PB_5, PB_10, PA_8, PA_9, PB_6);

// ピン割当
Serial arduino_sub(PA_0, PA_1);   // TX, RX
Serial arduino_imu(PA_2, PA_3);   // TX, RX

DigitalIn button_middle(PC_12);
DigitalIn button_left(PC_10);
DigitalIn button_right(PC_13);

DigitalOut line_led(PB_2);

AnalogIn line_front_1(PB_0);
AnalogIn line_front_2(PC_1);
AnalogIn line_right_1(PC_3);
AnalogIn line_right_2(PC_2);
AnalogIn line_back_1(PA_4);
AnalogIn line_back_2(PC_0);
AnalogIn line_left_1(PA_7);
AnalogIn line_left_2(PA_6);
/*
PwmOut motor_1_2(PA_10);
PwmOut motor_1_1(PB_3);
PwmOut motor_2_2(PB_5);
PwmOut motor_2_1(PA_11);
PwmOut motor_3_2(PA_8);
PwmOut motor_3_1(PB_10);
PwmOut motor_4_2(PB_6);
PwmOut motor_4_1(PA_9);
*/

// 関数定義
// void _motor(int16_t move_angle, int16_t move_speed, uint8_t brake);
void line_read(bool line_set, uint16_t line_threshpre);
void line_move(uint8_t* line_true, int16_t* line_move_angle, uint8_t* line_brake, int16_t ball_angle);
void ui(int8_t* select, int8_t display_mode, int8_t* set_mode, int8_t* set_value, int8_t* mode, float voltage_value, bool* line_set, int16_t ball_angle, int16_t ball_distance, int16_t* move_speed, int16_t* line_move_speed, uint16_t* line_threshpre, float dt, float* ball_rc, uint8_t* ball_follow_depth);

// グローバル変数定義（なるべく使わないように）
bool line_check[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int16_t yellow_angle = 0, blue_angle = 0;

short yaw, set_yaw;

// タイマー定義
Timer display_timer;
Timer line_timer;
Timer dt_timer;
Timer line_brake_timer;

void imu_getc() {   // IMU情報の取得
      if (arduino_imu.getc() == 'a') {
            short yaw_plus = 0, yaw_minus = 0;
            yaw_plus = arduino_imu.getc();
            yaw_minus = arduino_imu.getc();
            yaw = (yaw_plus == 0 ? yaw_minus * -1 : yaw_plus) - set_yaw;
            yaw -= yaw > 180 ? 360 : (yaw < -180 ? -360 : 0);
      }
}

void cam_getc() {   // カメラ情報の入手
      if (arduino_sub.getc() == 'a') {
            yellow_angle = arduino_sub.getc() - 106;
            blue_angle = arduino_sub.getc() - 106;
            if (yellow_angle == -106) yellow_angle = 0;
            if (blue_angle == -106) blue_angle = 0;
            yellow_angle *= -1;
            blue_angle *= -1;
      }
}

int main() {
      oled.begin();
      oled.clearDisplay();
      oled.setTextCursor(0, 0);
      oled.printf("setting now");
      oled.display();
      _speaker.startup_sound(2);

      // メイン関数内の変数定義
      bool line_set = 0;
      uint8_t line_true = 0, line_brake = 0;
      int16_t line_move_angle = 0;
      uint16_t line_threshpre = 250;

      int16_t ball_angle = 0, ball_distance = 0;
      float ball_rc = 0.5;

      bool battery_warning = 0;
      uint16_t voltage_stop_count = 0;
      float voltage_value = 0, voltage_drop_value = 7.00;

      int8_t mode = 0, display_mode = 0, select = 0, set_value = 0, set_mode = 0;
      bool pre_button_middle = 1, pre_button_right = 1, pre_button_left = 1;

      uint8_t ball_follow_depth = 25;
      int16_t move_speed = 30, line_move_speed = 30;

      float dt = 0;

      // serial set up
      arduino_sub.baud(19200);
      arduino_imu.baud(38400);

      // ラインセンサの閾値設定
      line_led = 1;
      wait_us(250000);
      line_read(1, line_threshpre);
      line_led = 0;

      // モーターPWM周波数の設定
      _motor.set_pwm();

      // arduinoのシリアル割り込み設定
      arduino_imu.attach(imu_getc, Serial::RxIrq);
      arduino_sub.attach(cam_getc, Serial::RxIrq);

      // 必要なタイマーのスタート
      display_timer.start();
      dt_timer.start();

      while (true) {
            voltage_value = _voltage.get();
            if (voltage_stop_count > VOLTAGE_STOP_COUNT_NUMBER_OF_TIMES) {   // 電圧低下による停止
                  _motor.free();
                  line_led = 0;
                  if (battery_warning == 0) {
                        oled.clearDisplay();
                        oled.setTextCursor(0, 0);
                        oled.printf("battery is row\n");
                        oled.printf("please change battery");
                        oled.display();
                        _speaker.run(400, 50);
                        wait_us(50000);
                        _speaker.run(400, 50);
                        wait_us(50000);
                        battery_warning = 1;
                  }
                  oled.setTextCursor(0, 30);
                  oled.printf("middle: continue");
                  oled.display();
                  voltage_drop_value = 10.00;
                  if (button_middle == 0) {
                        voltage_drop_value = 0.00;
                        voltage_stop_count = 0;
                        _speaker.run(380, 50);
                  }
            } else {   // 通常時
                  line_read(line_set, line_threshpre);   // ラインセンサの値取得
                  _ball.get(&ball_angle, &ball_distance, ball_rc);   // IRセンサの値の取得
                  _motor.yaw = yaw;

                  if (voltage_value < voltage_drop_value) voltage_stop_count++;
                  if (voltage_value > voltage_drop_value && voltage_stop_count > 0) voltage_stop_count--;

                  if (mode == 0) {   // UI操作時
                        _motor.free();
                  } else if (mode == 1) {   // モード１
                        line_move(&line_true, &line_move_angle, &line_brake, ball_angle);

                        if (line_true != 0) {
                              if (line_brake == 1) {
                                    _motor.brake();
                              } else {
                                    _motor.run(line_move_angle, line_move_speed);
                              }
                        } else {
                              if (abs(blue_angle) > 75) {   // コートの端
                                    _motor.run(blue_angle > 0 ? -135 : 135, move_speed);
                              } else if (ball_distance == 0) {   // ボールがない
                                    _motor.run(0, 0);
                              } else {   // ボールがある時
                                    _motor.run(abs(ball_angle) > BALL_FOLLOW_RANGE ? ball_angle + (ball_angle > 0 ? 90 : -90) : ball_angle * ((90 + BALL_FOLLOW_RANGE) / BALL_FOLLOW_RANGE) * (((ball_distance < ball_follow_depth ? ball_follow_depth : ball_distance) - ball_follow_depth) / float(100.000 - ball_follow_depth)), move_speed);   // 回り込み
                              }
                        }
                  } else if (mode == 2) {   // モード２
                  } else if (mode == 3) {   // モード３
                  } else if (mode == 4) {   // モード４
                  }

                  // ui
                  if (mode == 0) {   // UI操作時
                        display_timer.start();
                        dt_timer.start();
                        dt = 1.000 / dt_timer.read();
                        dt_timer.reset();

                        if (display_timer.read() > DISPLAY_UPDATE_RATE) {
                              ui(&select, display_mode, &set_mode, &set_value, &mode, voltage_value, &line_set, ball_angle, ball_distance, &move_speed, &line_move_speed, &line_threshpre, dt, &ball_rc, &ball_follow_depth);   // DISPLAY_UPDATE_RATEごとにOLEDを更新する
                              display_timer.reset();
                        }

                        _motor.set_pwm();
                  } else {   // 動いている時
                        display_timer.stop();
                        dt_timer.stop();
                        if (set_value == 0) {
                              if (select == 2 && (mode == 1 || mode == 2)) select = 0;
                              mode = 0;
                        }
                  }

                  // ボタンの処理
                  if (button_middle == 0 && pre_button_middle == 1 && button_right == 1 && button_left == 1) {
                        select++;
                        set_value = 0;
                  }

                  if (button_right == 0 && pre_button_right == 1 && select == 0) display_mode++;
                  if (button_left == 0 && pre_button_left == 1 && select == 0) display_mode--;

                  if (button_right == 0 && pre_button_right == 1 && set_mode == 1) set_value = set_value != 2 ? 2 : 0;
                  if (button_left == 0 && pre_button_left == 1 && set_mode == 1) set_value = set_value != 1 ? 1 : 0;
                  if (button_right == 0 && pre_button_right == 1 && set_mode == 2) set_value++;
                  if (button_right == 0 && set_mode == 2 && button_middle == 0) set_value++;
                  if (button_left == 0 && pre_button_left == 1 && set_mode == 2) set_value--;
                  if (button_left == 0 && set_mode == 2 && button_middle == 0) set_value--;

                  if (set_mode == 2 && button_middle == 0 && (button_right == 0 || button_left == 0)) _speaker.run(325, 50);
                  if ((button_right == 0 || button_left == 0) && (select == 0 || set_mode != 0) && (pre_button_right == 1 && pre_button_left == 1)) _speaker.run(350, 25);
                  if (button_middle == 0 && pre_button_middle == 1 && button_right == 1 && button_left == 1) _speaker.run(375, 50);

                  pre_button_middle = button_middle;
                  pre_button_right = button_right;
                  pre_button_left = button_left;
            }
      }
}

void ui(int8_t* select, int8_t display_mode, int8_t* set_mode, int8_t* set_value, int8_t* mode, float voltage_value, bool* line_set, int16_t ball_angle, int16_t ball_distance, int16_t* move_speed, int16_t* line_move_speed, uint16_t* line_threshpre, float dt, float* ball_rc, uint8_t* ball_follow_depth) {   // UI
      if (*mode == 0) {
            oled.clearDisplay();   // ディスプレイのクリア
            oled.setTextCursor(0, 0);   // 描写位置の設定
            if (*select == 0) {
                  oled.printf("%.2fhz", dt);
                  oled.setTextCursor(95, 0);   // 描写位置の設定
                  oled.printf("%.2fv\n", voltage_value);
                  for (int count = 0; count < 128; count++) oled.drawPixel(count, 11, 1);
            }
      }

      if (display_mode == 0) {
            if (*select == 0) {
                  oled.printf("main\n");
            } else if (*select == 1) {
                  if (*mode == 0) {
                        oled.printf("left : offense\n");
                        oled.printf("right: defense");
                        line_led = 1;
                        *set_mode = 1;
                  }
                  *mode = *set_value == 1 ? 1 : (*set_value == 2 ? 2 : 0);
            } else if (*select == 2) {
                  if (*mode == 0) {
                        oled.printf("left : ball follow\n");
                        oled.printf("right: PD test");
                  } else if (*mode == 1 || *mode == 2) {
                        *select = 0;
                  }
                  *mode = *set_value == 1 ? 3 : (*set_value == 2 ? 4 : 0);
            } else {
                  line_led = 0;
                  *select = 0;
                  *set_mode = 0;
                  *set_value = 0;
                  *mode = 0;
            }
      } else if (display_mode == -1) {
            if (*select == 0) {
                  oled.printf("imu");
            } else if (*select == 1) {
                  oled.printf("yaw  : %d\n", yaw);
                  oled.printf("left or right: reset");
                  *set_mode = 1;
                  if (*set_value == 1 || *set_value == 2) {
                        set_yaw += yaw;
                        *set_value = 0;
                  }
            } else {
                  *select = 0;
                  *set_mode = 0;
            }
      } else if (display_mode == 1) {
            if (*select == 0) {
                  oled.printf("speed");
            } else if (*select == 1) {
                  oled.printf("speed : %d\n", *move_speed);
                  *set_mode = 2;
                  *move_speed += *set_value * 5;
                  *set_value = 0;
            } else if (*select == 2) {
                  oled.printf("line speed : %d\n", *line_move_speed);
                  *set_mode = 2;
                  *line_move_speed += *set_value * 5;
                  *set_value = 0;
            } else {
                  *select = 0;
                  *set_mode = 0;
            }
      } else if (display_mode == -2) {
            if (*select == 0) {
                  oled.printf("line sensor");
            } else if (*select == 1) {
                  oled.setTextCursor(20, 0);
                  oled.printf("%d", line_check[0] == 1 ? 1 : 0);
                  oled.setTextCursor(20, 10);
                  oled.printf("%d", line_check[1] == 1 ? 1 : 0);
                  oled.setTextCursor(40, 20);
                  oled.printf("%d", line_check[2] == 1 ? 1 : 0);
                  oled.setTextCursor(30, 20);
                  oled.printf("%d", line_check[3] == 1 ? 1 : 0);
                  oled.setTextCursor(20, 40);
                  oled.printf("%d", line_check[4] == 1 ? 1 : 0);
                  oled.setTextCursor(20, 30);
                  oled.printf("%d", line_check[5] == 1 ? 1 : 0);
                  oled.setTextCursor(0, 20);
                  oled.printf("%d", line_check[6] == 1 ? 1 : 0);
                  oled.setTextCursor(10, 20);
                  oled.printf("%d", line_check[7] == 1 ? 1 : 0);
                  oled.setTextCursor(0, 55);
                  oled.printf("left or right: reset");

                  line_led = 1;
                  *set_mode = 1;
                  if (*set_value == 1) {
                        *line_set = 1;
                        *set_value = 0;
                  } else if (*set_value == 2) {
                        *line_set = 1;
                        *set_value = 0;
                  } else {
                        *line_set = 0;
                  }
            } else if (*select == 2) {
                  oled.printf("threshpre : %d", *line_threshpre);
                  *set_mode = 2;
                  *line_threshpre += *set_value * 10;
                  *set_value = 0;
            } else {
                  line_led = 0;
                  *select = 0;
                  *set_mode = 0;
            }
      } else if (display_mode == -3) {
            if (*select == 0) {
                  oled.printf("ir");
            } else if (*select == 1) {
                  oled.printf("dir: %d\n", ball_angle);
                  oled.printf("dis: %d\n", ball_distance);
                  oled.printf("rcf: %.2f\n", *ball_rc);
                  for (uint16_t count = 0; count < 360; count += 3) oled.drawPixel(95 + (15 * cos(count * PI / 180.000)), 32 + (15 * sin(count * PI / 180.000)), 1);
                  for (uint16_t count = 0; count < 360; count += 2) oled.drawPixel(95 + (30 * cos(count * PI / 180.000)), 32 + (30 * sin(count * PI / 180.000)), 1);
                  for (uint8_t count = 0; count < 64; count++) oled.drawPixel(95, count, 1);
                  for (uint8_t count = 63; count < 127; count++) oled.drawPixel(count, 32, 1);
                  for (uint16_t count = 0; count < 360; count += 10) oled.drawPixel(95 + ((110 - ball_distance) / 2.5 * cos((ball_angle - 90) * PI / 180.000)) + (3 * cos(count * PI / 180.000)), 32 + ((110 - ball_distance) / 2.5 * sin((ball_angle - 90) * PI / 180.000)) + (3 * sin(count * PI / 180.000)), 1);
                  *set_mode = 2;
                  *ball_rc += *set_value * 0.05;
                  *set_value = 0;
            } else {
                  *set_mode = 0;
                  *select = 0;
            }
      } else if (display_mode == -4) {
            if (*select == 0) {
                  oled.printf("camera");
            } else if (*select == 1) {
                  oled.printf("yellow: %d\n", yellow_angle);
                  oled.printf("blue  : %d\n", blue_angle);
            } else {
                  *select = 0;
            }
      } else if (display_mode == 2) {
            if (*select == 0) {
                  oled.printf("ball follow");
            } else if (*select == 1) {
                  oled.printf("depth : %d\n", *ball_follow_depth);
                  *set_mode = 2;
                  *ball_follow_depth += *set_value * 5;
                  *set_value = 0;
            } else {
                  *select = 0;
                  *set_mode = 0;
            }
      }
      oled.display();
}

void line_read(bool line_set, uint16_t line_threshpre) {   // ラインセンサ値の取得
      uint16_t line_value[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      static uint16_t pre_line_value[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      static uint16_t line_reaction[8] = {0, 0, 0, 0, 0, 0, 0, 0};

      if (line_set == 1) {   // 閾値の設定
            for (uint8_t count = 0; count < 8; count++) line_reaction[count] = 0;
            for (uint16_t count = 0; count < LINE_REACTION_AVERAGE_NUMBER_OF_TIMES; count++) {
                  line_reaction[0] += line_front_1.read_u16() * 0.01;
                  line_reaction[1] += line_front_2.read_u16() * 0.01;
                  line_reaction[2] += line_right_1.read_u16() * 0.01;
                  line_reaction[3] += line_right_2.read_u16() * 0.01;
                  line_reaction[4] += line_back_1.read_u16() * 0.01;
                  line_reaction[5] += line_back_2.read_u16() * 0.01;
                  line_reaction[6] += line_left_1.read_u16() * 0.01;
                  line_reaction[7] += line_left_2.read_u16() * 0.01;
            }
            for (uint8_t count = 0; count < 8; count++) line_reaction[count] = line_reaction[count] / LINE_REACTION_AVERAGE_NUMBER_OF_TIMES;
      }

      // 値の取得
      line_value[0] = line_front_1.read_u16() * 0.01;
      line_value[1] = line_front_2.read_u16() * 0.01;
      line_value[2] = line_right_1.read_u16() * 0.01;
      line_value[3] = line_right_2.read_u16() * 0.01;
      line_value[4] = line_back_1.read_u16() * 0.01;
      line_value[5] = line_back_2.read_u16() * 0.01;
      line_value[6] = line_left_1.read_u16() * 0.01;
      line_value[7] = line_left_2.read_u16() * 0.01;

      for (uint8_t count = 0; count < 8; count++) {
            line_value[count] = line_value[count] * (1 - LINE_RC) + pre_line_value[count] * LINE_RC;
            pre_line_value[count] = line_value[count];
            line_check[count] = line_value[count] > line_reaction[count] + line_threshpre ? 1 : 0;   // 反応したかのチェック
      }
}

void line_move(uint8_t* line_true, int16_t* line_move_angle, uint8_t* line_brake, int16_t ball_angle) {
      uint8_t line_check_unit[4] = {0, 0, 0, 0}, line_true_unit[4] = {0, 0, 0, 0};
      static uint8_t pre_line = 0;
      static int16_t line_back_angle = 0;
      float line_unit_vector_x[8] = {0, 0, 0.5, 1, 0, 0, -0.5, -1};
      float line_unit_vector_y[8] = {0.5, 1, 0, 0, -0.5, -1, 0, 0};
      static float line_result_vector_x = 0, line_result_vector_y = 0;
      static float pre_line_result_vector_x = 0, pre_line_result_vector_y = 0;
      *line_brake = 0;

      for (uint8_t count = 0; count < 4; count++) line_check_unit[count] = line_check[count * 2] == 1 || line_check[count * 2 + 1] == 1 ? 1 : 0;
      for (uint8_t count = 0; count < 4; count++) line_true_unit[count] = *line_true == count + 1 || *line_true == count + 5 ? 1 : 0;

      if (line_check_unit[0] == 1 || line_check_unit[1] == 1 || line_check_unit[2] == 1 || line_check_unit[3] == 1) {   // いずれかのラインセンサが反応している時
            line_timer.start();
            line_timer.reset();
            if (*line_true == 0) line_brake_timer.start();
            for (uint8_t count = 0; count < 8; count++) {
                  if (line_check[count] == 1) pre_line = count;   // 最後に反応したラインセンサの記憶
            }

            if (line_check_unit[0] == 1 && ((*line_true == 3 && line_check[0] == 1) || *line_true == 0)) *line_true = 1;   // 前外ライン
            if (line_check[5] == 1 && *line_true == 1) *line_true = 5;   // 前内ライン
            if (line_check_unit[1] == 1 && ((*line_true == 4 && line_check[2] == 1) || *line_true == 0)) *line_true = 2;   // 前外ライン
            if (line_check[7] == 1 && *line_true == 2) *line_true = 6;   // 前内ライン
            if (line_check_unit[2] == 1 && ((*line_true == 1 && line_check[4] == 1) || *line_true == 0)) *line_true = 3;   // 前外ライン
            if (line_check[1] == 1 && *line_true == 3) *line_true = 7;   // 前内ライン
            if (line_check_unit[3] == 1 && ((*line_true == 2 && line_check[6] == 1) || *line_true == 0)) *line_true = 4;   // 前外ライン
            if (line_check[3] == 1 && *line_true == 4) *line_true = 8;   // 前内ライン
      }

      if ((line_true_unit[0] == 1 && pre_line != 4) || (line_true_unit[1] == 1 && pre_line != 6) || (line_true_unit[2] == 1 && pre_line != 0) || (line_true_unit[3] == 1 && pre_line != 2)) {   // ライン処理から通常処理に戻る
            if (line_timer.read() > 0.25 && (line_timer.read() > 2.5 || (line_true_unit[0] == 1 && (ball_angle < -45 || ball_angle > 45)) || (line_true_unit[2] == 1 && (ball_angle > -90 && ball_angle < 90)) || (line_true_unit[1] == 1 && (ball_angle > 90 || ball_angle < 30)) || (line_true_unit[3] == 1 && (ball_angle < -90 || ball_angle > -30)))) {
                  *line_true = 0;
                  line_timer.stop();
                  line_timer.reset();
                  line_brake_timer.stop();
                  line_brake_timer.reset();
            }
            *line_brake = line_check_unit[0] == 0 && line_check_unit[1] == 0 && line_check_unit[2] == 0 && line_check_unit[3] == 0 ? 1 : 0;
      }

      if (*line_true != 0) {
            for (uint8_t count = 0; count < 8; count++) {
                  if (line_check[count] == 1) {
                        line_result_vector_x = 0;
                        line_result_vector_y = 0;
                  }
            }
            for (uint8_t count = 0; count < 8; count++) {
                  line_result_vector_x += line_check[count] * line_unit_vector_x[count];
                  line_result_vector_y += line_check[count] * line_unit_vector_y[count];
            }
            if (pre_line_result_vector_x != line_result_vector_x || pre_line_result_vector_y != line_result_vector_y) line_back_angle = atan2(line_result_vector_x, line_result_vector_y) / PI * 180.000 + 180.5;
            pre_line_result_vector_x = line_result_vector_x;
            pre_line_result_vector_y = line_result_vector_y;
            if (line_back_angle > 180) line_back_angle -= 360;
      }

      if (line_brake_timer.read() <= 0.05) {
            *line_brake = 1;
      } else {
            if (line_true_unit[0] == 1) {   // 前ライン
                  *line_move_angle = line_back_angle <= -90 || line_back_angle >= 90 ? line_back_angle : line_back_angle - 180;
            } else if (line_true_unit[1] == 1) {   // 右ライン
                  *line_move_angle = line_back_angle >= -180 && line_back_angle <= 0 ? line_back_angle : line_back_angle - 180;
            } else if (line_true_unit[2] == 1) {   // 後ライン
                  *line_move_angle = line_back_angle >= -90 && line_back_angle <= 90 ? line_back_angle : line_back_angle - 180;
            } else if (line_true_unit[3] == 1) {   // 左ライン
                  *line_move_angle = line_back_angle >= 0 && line_back_angle <= 180 ? line_back_angle : line_back_angle - 180;
            }
      }
}