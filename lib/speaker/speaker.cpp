#include "speaker.h"

#include "mbed.h"

speaker::speaker(PinName speaker_pin_) : speaker_pin(speaker_pin_) {
      speaker_pin = 0;
}

void speaker::run(uint16_t sound_scale, uint16_t sound_time) {
      speaker_pin = SPEAKER_DUTY;
      speaker_pin.period_us(sound_scale);
      if (sound_time != 0) wait_us(sound_time * 1000);
      speaker_pin = 0;
}

void speaker::startup_sound(uint8_t mode) {
      if (mode == 1) {
            speaker_pin = SPEAKER_DUTY;
            speaker_pin.period_us(350);
            wait_us(50000);
            speaker_pin = 0;

            wait_us(50000);

            speaker_pin = SPEAKER_DUTY;
            speaker_pin.period_us(300);
            wait_us(50000);

            speaker_pin = 0;
      } else if (mode == 2) {
            speaker_pin = SPEAKER_DUTY;
            speaker_pin.period_us(350);
            wait_us(50000);
            speaker_pin = 0;

            wait_us(50000);

            speaker_pin = SPEAKER_DUTY;
            speaker_pin.period_us(350);
            wait_us(50000);
            speaker_pin = 0;
      } else if (mode == 3) {
            speaker_pin = SPEAKER_DUTY;
            speaker_pin.period_us(350);
            wait_us(50000);
            speaker_pin = 0;
            wait_us(150000);
            speaker_pin = SPEAKER_DUTY;
            speaker_pin.period_us(400);
            wait_us(50000);
            speaker_pin = 0;
            wait_us(50000);
            speaker_pin = SPEAKER_DUTY;
            speaker_pin.period_us(350);
            wait_us(50000);
            speaker_pin = 0;
            wait_us(50000);
            speaker_pin = SPEAKER_DUTY;
            speaker_pin.period_us(300);
            wait_us(50000);
            speaker_pin = 0;
      }
}