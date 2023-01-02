// RCJ 2022 season program for Takumi Banba

#include "Adafruit_SSD1306.h"
#include "Serial.h"
#include "ball.h"
#include "line.h"
#include "math.h"
#include "mbed.h"
#include "motor.h"
#include "speaker.h"
#include "voltage.h"

// 定義
#define PI 3.1415926535   // 円周率

#define DISPLAY_UPDATE_RATE 0.05   // OLEDの更新時間

#define VOLTAGE_STOP_COUNT_NUMBER_OF_TIMES 500   // 停止する電圧低下の回数

#define BALL_FOLLOW_RANGE 45.000   // 回り込み時にボールを追い始める角度

#define CAM_RC 0.6

#define D_PERIODO 0.01
#define KP 2.5
#define KD 5

// I2Cの設定
I2C i2c(PB_9, PB_8);
Adafruit_SSD1306_I2c oled(i2c, D9, SSD_I2C_ADDRESS, 64, 128);

speaker _speaker(PC_9);

ball _ball(PB_12, PB_13, PB_14, PC_4, PA_15, PC_11, PD_2, PC_8);

voltage _voltage(PA_5);

motor _motor(PB_3, PA_10, PA_11, PB_5, PB_10, PA_8, PA_9, PB_6);   // 　45度、135度、225度、315度

line _line(PB_2, PB_0, PC_1, PC_3, PC_2, PA_4, PC_0, PA_7, PA_6);   // ledピン、前ライン、右ライン、後ライン、左ライン

// ピン割当
Serial arduino_cam(PA_0, PA_1);   // TX, RX
Serial arduino_imu(PA_2, PA_3);   // TX, RX

DigitalIn button_middle(PC_12);
DigitalIn button_left(PC_10);
DigitalIn button_right(PC_13);

// 関数定義
void line_move(uint8_t* line_tf, int16_t* line_move_angle, uint8_t* line_brake);
void offence_move(uint8_t* ball_follow_depth, int16_t* move_speed, uint16_t diffence_line);
void diffence_move(uint16_t diffence_line, int16_t* move_speed);
void ui(int8_t* select, int8_t display_mode, int8_t* set_mode, int8_t* set_value, int8_t* mode, int16_t* move_speed, int16_t* line_move_speed, float dt, uint8_t* ball_follow_depth);

// グローバル変数定義（なるべく使わないように）
uint8_t goal_angle_mode = 1;
int16_t goal_angle;
uint16_t back_distance;

int16_t yaw, set_yaw;

// タイマー定義
Timer display_timer;
Timer line_timer;
Timer dt_timer;
Timer line_brake_timer;
Timer diffence_timer;
Timer d_timer;

void imu_getc() {   // IMU情報の取得
      if (arduino_imu.getc() == 'a') {
            int16_t yaw_plus, yaw_minus;
            yaw_plus = arduino_imu.getc();
            yaw_minus = arduino_imu.getc();
            yaw = (yaw_plus == 0 ? yaw_minus * -1 : yaw_plus) - set_yaw;
            yaw -= yaw > 180 ? 360 : (yaw < -180 ? -360 : 0);
      }
}

void cam_getc() {   // カメラ情報の入手
      static int16_t pre_blue_angle, pre_yellow_angle, yellow_angle, blue_angle;
      if (arduino_cam.getc() == 'a') {
            yellow_angle = arduino_cam.getc() - 106;
            blue_angle = arduino_cam.getc() - 106;
            if (yellow_angle == -106) yellow_angle = 0;
            if (blue_angle == -106) blue_angle = 0;
            yellow_angle = yellow_angle * (1 - CAM_RC) + pre_yellow_angle * CAM_RC;
            blue_angle = blue_angle * (1 - CAM_RC) + pre_blue_angle * CAM_RC;
            pre_yellow_angle = yellow_angle;
            pre_blue_angle = blue_angle;
            if (goal_angle_mode == 1) goal_angle = yellow_angle;
            if (goal_angle_mode == 2) goal_angle = blue_angle;

            back_distance = arduino_cam.getc();
      }
}

int main() {
      oled.begin();
      oled.clearDisplay();
      oled.setTextCursor(0, 0);
      oled.printf("setting now");
      oled.display();
      _speaker.startup_sound(1);

      // メイン関数内の変数定義
      uint8_t line_tf = 0, line_brake = 0;
      int16_t line_move_angle = 0;

      bool battery_warning = 0;
      uint16_t voltage_stop_count = 0;
      float voltage_drop_value = 6.50;

      int8_t mode = 0, display_mode = 0, select = 0, set_value = 0, set_mode = 0;
      bool pre_button_middle = 1, pre_button_right = 1, pre_button_left = 1;

      bool pre_moving = 0;
      uint8_t ball_follow_depth = 25;
      int16_t move_speed = 60, line_move_speed = 90;

      uint16_t diffence_line = 60;

      float dt = 0;

      // serial set up
      arduino_cam.baud(19200);
      arduino_imu.baud(38400);

      // ラインセンサの閾値設定
      _line.set();

      // モーターPWM周波数の設定
      _motor.set_pwm();

      // arduinoのシリアル割り込み設定
      arduino_imu.attach(imu_getc, Serial::RxIrq);
      arduino_cam.attach(cam_getc, Serial::RxIrq);

      // 必要なタイマーのスタート
      display_timer.start();
      dt_timer.start();

      while (true) {
            _voltage.read();
            if (voltage_stop_count > VOLTAGE_STOP_COUNT_NUMBER_OF_TIMES) {   // 電圧低下による停止
                  _motor.free();
                  _line.led(0);
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
                  _line.read();
                  _ball.read();   // IRセンサの値の取得
                  _motor.yaw = yaw;

                  if (_voltage.get() < voltage_drop_value) voltage_stop_count++;
                  if (_voltage.get() > voltage_drop_value && voltage_stop_count > 0) voltage_stop_count--;

                  if (mode == 0) {   // UI操作時
                        _motor.free();
                  } else if (mode == 1) {   // モード１
                        line_move(&line_tf, &line_move_angle, &line_brake);

                        if (line_tf != 0) {
                              if (line_brake == 1) {
                                    _motor.brake();
                              } else {
                                    _motor.run(line_move_angle, line_move_speed);
                              }
                        } else {
                              if (_ball.get_distance() == 0) {   // ボールがない
                                    _motor.run(back_distance > 80 ? (goal_angle > 0 ? 135 : -135) : (goal_angle > 0 ? 45 : -45), abs(80 - back_distance) * 2 + abs(goal_angle) * 2);
                              } else {   // ボールがある時
                                    offence_move(&ball_follow_depth, &move_speed, diffence_line);
                              }
                        }
                  } else if (mode == 2) {   // モード２
                        line_move(&line_tf, &line_move_angle, &line_brake);

                        if (line_tf != 0) {
                              if (line_brake == 1) {
                                    _motor.brake();
                              } else {
                                    _motor.run(line_move_angle, line_move_speed);
                              }
                        } else {
                              if (_ball.get_distance() == 0) {   // ボールがない
                                    _motor.run(back_distance > diffence_line ? (goal_angle > 0 ? 135 : -135) : (goal_angle > 0 ? 45 : -45), abs(diffence_line - back_distance) * 2 + abs(goal_angle) * 2);
                              } else {   // ボールがある時
                                    diffence_move(diffence_line, &move_speed);
                              }
                        }
                  } else if (mode == 3) {   // モード３
                        _motor.run(0, 0);
                  } else if (mode == 4) {   // モード４
                        _motor.run(0, 0);
                  }

                  // ui
                  if (mode == 0) {   // UI操作時
                        display_timer.start();
                        dt_timer.start();
                        dt = 1.000 / dt_timer.read();
                        dt_timer.reset();
                        pre_moving = 0;

                        if (display_timer.read() > DISPLAY_UPDATE_RATE) {
                              ui(&select, display_mode, &set_mode, &set_value, &mode, &move_speed, &line_move_speed, dt, &ball_follow_depth);   // DISPLAY_UPDATE_RATEごとにOLEDを更新する
                              display_timer.reset();
                        }
                  } else {   // 動いている時
                        if (pre_moving == 0) {
                              display_timer.stop();
                              dt_timer.stop();
                              _motor.set_pwm();
                              oled.clearDisplay();
                              oled.display();
                              pre_moving = 1;
                        }

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

void offence_move(uint8_t* ball_follow_depth, int16_t* move_speed, uint16_t diffence_line) {
      int16_t tmp_move_speed, tmp_move_angle, ball_angle, ball_distance;
      float move_angle_decrement;
      ball_angle = _ball.get_angle() + yaw;
      ball_distance = _ball.get_distance();

      // 方向
      if (abs(ball_angle) < 15 && ball_distance > 80) {
            tmp_move_angle = 0;
      } else if (abs(ball_angle) <= BALL_FOLLOW_RANGE) {
            tmp_move_angle = ball_angle + (90.000 / BALL_FOLLOW_RANGE * ball_angle);
      } else {
            move_angle_decrement = ((ball_distance < *ball_follow_depth ? *ball_follow_depth : ball_distance) - *ball_follow_depth) / float(95.000 - *ball_follow_depth);
            tmp_move_angle = (ball_angle > 0 ? 90 : -90) + (ball_angle * move_angle_decrement);
      }
      if (tmp_move_angle > 180) tmp_move_angle -= 360;
      if (tmp_move_angle < -180) tmp_move_angle += 360;
      if (goal_angle_mode != 0 && goal_angle != 0 && ball_distance > 90 && abs(ball_angle) < BALL_FOLLOW_RANGE) tmp_move_angle += abs(goal_angle) / 2 > 30 ? (goal_angle > 0 ? 30 : -30) : goal_angle / 2;

      tmp_move_speed = *move_speed;

      _motor.run(tmp_move_angle, tmp_move_speed);   // 回り込み
}

void diffence_move(uint16_t diffence_line, int16_t* move_speed) {
      int16_t tmp_move_speed, tmp_move_angle, robot_angle, ball_angle;
      static int16_t pre_p;
      static int16_t p, d;
      ball_angle = _ball.get_angle() + yaw;

      if (diffence_timer.read() > 2.5) {
            if (diffence_timer.read() < 10 && back_distance < 100) {
                  tmp_move_angle = ball_angle * 2;
                  tmp_move_speed = *move_speed;
                  if (goal_angle_mode != 0 && goal_angle != 0 && back_distance > 80 && (goal_angle > 30 || goal_angle < -30)) robot_angle = goal_angle > 0 ? 60 : -60;
            } else {
                  diffence_timer.reset();
                  diffence_timer.stop();
            }
      } else {
            if (ball_angle > -15 && ball_angle < 15) {
                  diffence_timer.start();
            } else {
                  diffence_timer.reset();
                  diffence_timer.stop();
            }

            d_timer.start();
            p = 0 - ball_angle;   // 比例
            if (d_timer.read() > D_PERIODO) {
                  d = p - pre_p;   // 微分
                  pre_p = p;
                  d_timer.reset();
            }

            if (ball_angle > 0) {
                  if (back_distance > diffence_line) {
                        tmp_move_angle = 90 + abs(diffence_line - back_distance) * 3;
                  } else {
                        tmp_move_angle = 90 - abs(diffence_line - back_distance) * 3;
                  }

                  if (tmp_move_angle < 0) {
                        tmp_move_angle = 0;
                        tmp_move_speed = *move_speed;
                  } else if (tmp_move_angle > 180) {
                        tmp_move_angle = 180;
                        tmp_move_speed = *move_speed;
                  } else {
                        tmp_move_speed = abs(p * KP + d * KD);
                  }
            } else {
                  if (back_distance > diffence_line) {
                        tmp_move_angle = -90 - abs(diffence_line - back_distance) * 3;
                  } else {
                        tmp_move_angle = -90 + abs(diffence_line - back_distance) * 3;
                  }

                  if (tmp_move_angle > 0) {
                        tmp_move_angle = 0;
                        tmp_move_speed = *move_speed;
                  } else if (tmp_move_angle < -180) {
                        tmp_move_angle = 180;
                        tmp_move_speed = *move_speed;
                  } else {
                        tmp_move_speed = abs(p * KP + d * KD);
                  }
            }
      }
      if (tmp_move_speed > *move_speed) tmp_move_speed = *move_speed;
      _motor.run(tmp_move_angle, tmp_move_speed, robot_angle);   // 回り込み
}

void line_move(uint8_t* line_tf, int16_t* line_move_angle, uint8_t* line_brake) {
      uint8_t line_tf_unit[4] = {0, 0, 0, 0};
      int16_t ball_angle;
      static uint8_t pre_line = 0;
      static int16_t line_back_angle = 0;
      float line_unit_vector_x[8] = {0, 0, 0.5, 1, 0, 0, -0.5, -1};
      float line_unit_vector_y[8] = {0.5, 1, 0, 0, -0.5, -1, 0, 0};
      static float line_result_vector_x = 0, line_result_vector_y = 0;
      static float pre_line_result_vector_x = 0, pre_line_result_vector_y = 0;
      *line_brake = 0;
      ball_angle = _ball.get_angle();

      for (uint8_t count = 0; count < 4; count++) line_tf_unit[count] = *line_tf == count + 1 || *line_tf == count + 5 ? 1 : 0;

      if (_line.check_all() == 1) {   // いずれかのラインセンサが反応している時
            line_timer.start();
            line_timer.reset();
            if (*line_tf == 0) line_brake_timer.start();
            for (uint8_t count = 0; count < 8; count++) {
                  if (_line.check(count) == 1) pre_line = count;   // 最後に反応したラインセンサの記憶
            }

            if (_line.check_front() == 1 && *line_tf <= 4) *line_tf = 1;   // 前外ライン
            if (_line.check(1) == 1 && *line_tf == 1) *line_tf = 5;   // 前内ライン
            if (_line.check_right() == 1 && *line_tf <= 4) *line_tf = 2;   // 前外ライン
            if (_line.check(3) == 1 && *line_tf == 2) *line_tf = 6;   // 前内ライン
            if (_line.check_back() == 1 && *line_tf <= 4) *line_tf = 3;   // 前外ライン
            if (_line.check(5) == 1 && *line_tf == 3) *line_tf = 7;   // 前内ライン
            if (_line.check_left() == 1 && *line_tf <= 4) *line_tf = 4;   // 前外ライン
            if (_line.check(7) == 1 && *line_tf == 4) *line_tf = 8;   // 前内ライン
      }

      if (line_brake_timer.read() < 0.05) {
            *line_brake = 1;
      } else if ((line_tf_unit[0] == 1 && pre_line != 4) || (line_tf_unit[1] == 1 && pre_line != 6) || (line_tf_unit[2] == 1 && pre_line != 0) || (line_tf_unit[3] == 1 && pre_line != 2)) {   // ライン処理から通常処理に戻る
            if (line_timer.read() >= 0.05) {
                  if (_line.check_all() == 1 || line_timer.read() > 3 || (line_tf_unit[0] == 1 && (ball_angle < -60 || ball_angle > 60)) || (line_tf_unit[2] == 1 && (ball_angle > -90 && ball_angle < 90)) || (line_tf_unit[1] == 1 && (ball_angle > 90 || ball_angle < 0)) || (line_tf_unit[3] == 1 && (ball_angle < -90 || ball_angle > 0))) {
                        *line_tf = 0;
                        line_timer.stop();
                        line_timer.reset();
                        line_brake_timer.stop();
                        line_brake_timer.reset();
                  }
            }
            *line_brake = _line.check_all() == 0 ? 1 : 0;
      }

      if (*line_tf != 0) {
            if (_line.check_all() == 1) {
                  line_result_vector_x = 0;
                  line_result_vector_y = 0;
            }
            for (uint8_t count = 0; count < 8; count++) {
                  line_result_vector_x += _line.check(count) * line_unit_vector_x[count];
                  line_result_vector_y += _line.check(count) * line_unit_vector_y[count];
            }
            if (pre_line_result_vector_x != line_result_vector_x || pre_line_result_vector_y != line_result_vector_y) line_back_angle = atan2(line_result_vector_x, line_result_vector_y) / PI * 180.000 + 180.5;
            pre_line_result_vector_x = line_result_vector_x;
            pre_line_result_vector_y = line_result_vector_y;
            if (line_back_angle > 180) line_back_angle -= 360;
      }

      if (line_tf_unit[0] == 1) {   // 前ライン
            *line_move_angle = line_back_angle <= -90 || line_back_angle >= 90 ? line_back_angle : line_back_angle - 180;
      } else if (line_tf_unit[1] == 1) {   // 右ライン
            *line_move_angle = line_back_angle >= -180 && line_back_angle <= 0 ? line_back_angle : line_back_angle - 180;
      } else if (line_tf_unit[2] == 1) {   // 後ライン
            *line_move_angle = line_back_angle >= -90 && line_back_angle <= 90 ? line_back_angle : line_back_angle - 180;
      } else if (line_tf_unit[3] == 1) {   // 左ライン
            *line_move_angle = line_back_angle >= 0 && line_back_angle <= 180 ? line_back_angle : line_back_angle - 180;
      }
}

void ui(int8_t* select, int8_t display_mode, int8_t* set_mode, int8_t* set_value, int8_t* mode, int16_t* move_speed, int16_t* line_move_speed, float dt, uint8_t* ball_follow_depth) {   // UI
      if (*mode == 0) {
            oled.clearDisplay();   // ディスプレイのクリア
            oled.setTextCursor(0, 0);   // 描写位置の設定
            if (*select == 0) {
                  oled.printf("%.2fhz", dt);
                  oled.setTextCursor(95, 0);   // 描写位置の設定
                  oled.printf("%.2fv\n", _voltage.get());
                  for (int count = 0; count < 128; count++) oled.drawPixel(count, 11, 1);
            }
      }

      if (display_mode == 0) {
            if (*select == 0) {
                  oled.printf("main\n");
            } else if (*select == 1) {
                  _line.led(1);
                  if (*mode == 0) {
                        oled.printf("left : offense\n");
                        oled.printf("right: defense");
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
                  _line.led(0);
                  *select = 0;
                  *set_mode = 0;
                  *set_value = 0;
                  *mode = 0;
            }
      } else if (display_mode == -1) {
            if (*select == 0) {
                  oled.printf("IMU");
            } else if (*select == 1) {
                  oled.printf("yaw  : %d\n", yaw);
                  oled.printf("left : zero\n");
                  oled.printf("right : offset");
                  *set_mode = 1;
                  if (*set_value == 1) set_yaw += yaw;
                  if (*set_value == 2) {
                        set_yaw = 0;
                        arduino_imu.putc('a');
                  }
                  *set_value = 0;
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
      } else if (display_mode == -3) {
            if (*select == 0) {
                  oled.printf("line sensor");
            } else if (*select == 1) {
                  _line.led(1);
                  oled.setTextCursor(20, 0);
                  oled.printf("%d", _line.check(0) == 1 ? 1 : 0);
                  oled.setTextCursor(20, 10);
                  oled.printf("%d", _line.check(1) == 1 ? 1 : 0);
                  oled.setTextCursor(40, 20);
                  oled.printf("%d", _line.check(2) == 1 ? 1 : 0);
                  oled.setTextCursor(30, 20);
                  oled.printf("%d", _line.check(3) == 1 ? 1 : 0);
                  oled.setTextCursor(20, 40);
                  oled.printf("%d", _line.check(4) == 1 ? 1 : 0);
                  oled.setTextCursor(20, 30);
                  oled.printf("%d", _line.check(5) == 1 ? 1 : 0);
                  oled.setTextCursor(0, 20);
                  oled.printf("%d", _line.check(6) == 1 ? 1 : 0);
                  oled.setTextCursor(10, 20);
                  oled.printf("%d", _line.check(7) == 1 ? 1 : 0);
                  oled.setTextCursor(0, 55);
                  oled.printf("left or right: reset");

                  *set_mode = 1;
                  if (*set_value == 1 || *set_value == 2) {
                        _line.set();
                        *set_value = 0;
                  }
            } else if (*select == 2) {
                  _line.led(0);
                  oled.printf("threshpre : %d", _line.threshold);
                  *set_mode = 2;
                  _line.threshold += *set_value * 10;
                  *set_value = 0;
            } else {
                  *select = 0;
                  *set_mode = 0;
            }
      } else if (display_mode == -4) {
            if (*select == 0) {
                  oled.printf("ir");
            } else if (*select == 1) {
                  oled.printf("ang: %d\n", _ball.get_angle());
                  oled.printf("dis: %d\n", _ball.get_distance());
                  for (uint16_t count = 0; count < 360; count += 3) oled.drawPixel(95 + (15 * cos(count * PI / 180.000)), 32 + (15 * sin(count * PI / 180.000)), 1);
                  for (uint16_t count = 0; count < 360; count += 2) oled.drawPixel(95 + (30 * cos(count * PI / 180.000)), 32 + (30 * sin(count * PI / 180.000)), 1);
                  for (uint8_t count = 0; count <= 64; count++) oled.drawPixel(95, count, 1);
                  for (uint8_t count = 63; count <= 127; count++) oled.drawPixel(count, 32, 1);
                  for (uint16_t count = 0; count < 360; count += 10) oled.drawPixel(95 + ((110 - _ball.get_distance()) / 2.5 * cos((_ball.get_angle() - 90) * PI / 180.000)) + (3 * cos(count * PI / 180.000)), 32 + ((110 - _ball.get_distance()) / 2.5 * sin((_ball.get_angle() - 90) * PI / 180.000)) + (3 * sin(count * PI / 180.000)), 1);
            } else {
                  *select = 0;
            }
      } else if (display_mode == -2) {
            if (*select == 0) {
                  oled.printf("camera");
            } else if (*select == 1) {
                  if (goal_angle_mode == 0) oled.printf("off\n");
                  if (goal_angle_mode == 1) oled.printf("yellow: %d\n", goal_angle);
                  if (goal_angle_mode == 2) oled.printf("blue  : %d\n", goal_angle);
                  oled.printf("left  : yellow\n");
                  oled.printf("right : blue");
                  *set_mode = 1;
                  if (*set_value == 1) goal_angle_mode = 1;
                  if (*set_value == 2) goal_angle_mode = 2;
                  *set_value = 0;
            } else if (*select == 2) {
                  if (goal_angle_mode == 0) oled.printf("left or right : off");
                  if (goal_angle_mode == 1) oled.printf("yellow: %d\n", goal_angle);
                  if (goal_angle_mode == 2) oled.printf("blue  : %d\n", goal_angle);
                  if (*set_value == 1 || *set_value == 2) goal_angle_mode = 0;
                  *set_value = 0;
            } else {
                  *select = 0;
                  *set_mode = 0;
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
      } else if (display_mode == -5) {
            if (*select == 0) {
                  oled.printf("distance");
            } else if (*select == 1) {
                  oled.printf("dis: %d\n", back_distance);
                  for (uint16_t count = 30; count <= 34; count++)
                        for (uint16_t count_1 = 0; count_1 < back_distance / 2; count_1 += 1) oled.drawPixel(count_1, count, 1);
                  for (uint16_t count = 0; count <= 2; count++)
                        for (uint16_t count_1 = 22; count_1 < 42; count_1 += 1) oled.drawPixel(50 * count, count_1, 1);
                  oled.setTextCursor(0, 45);
                  oled.printf("0");
                  oled.setTextCursor(42, 45);
                  oled.printf("100");
                  oled.setTextCursor(92, 45);
                  oled.printf("200");
            } else {
                  *select = 0;
            }
      } else if (display_mode == 3) {
            if (*select == 0) {
                  oled.printf("sub MCU reset");
            } else if (*select == 1) {
                  oled.printf("left : CAM MCU\n");
                  oled.printf("right : IMU MCU");
                  *set_mode = 1;
                  if (*set_value == 1) arduino_cam.putc('a');
                  if (*set_value == 2) arduino_imu.putc('a');
                  *set_value = 0;
            } else {
                  *select = 0;
                  *set_mode = 0;
            }
      }
      oled.display();
}