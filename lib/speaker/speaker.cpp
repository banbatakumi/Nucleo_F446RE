#include "speaker.h"

#include "mbed.h"

speaker::speaker(PinName pin) : _pin(pin) {
      _pin = 0;
}

void speaker::run(uint16_t sound_scale, uint16_t sound_time) {
      _pin = SPEAKER_DUTY;
      _pin.period_us(sound_scale);
      if (sound_time != 0) wait_us(sound_time * 1000);
      _pin = 0;
}

void speaker::startup_sound(uint8_t mode) {
      if (mode == 1) {
            _pin = SPEAKER_DUTY;
            _pin.period_us(300);
            wait_us(50000);
            _pin = 0;

            wait_us(150000);

            _pin = SPEAKER_DUTY;
            _pin.period_us(350);
            wait_us(50000);
            _pin = 0;

            wait_us(50000);

            _pin = SPEAKER_DUTY;
            _pin.period_us(300);
            wait_us(50000);
            _pin = 0;

            wait_us(50000);

            _pin = SPEAKER_DUTY;
            _pin.period_us(250);
            wait_us(50000);
            _pin = 0;
      } else if (mode == 2) {
            _pin = SPEAKER_DUTY;
            _pin.period_us(300);
            wait_us(50000);
            _pin = 0;

            wait_us(50000);

            _pin = SPEAKER_DUTY;
            _pin.period_us(300);
            wait_us(50000);
            _pin = 0;

            wait_us(50000);

            _pin = SPEAKER_DUTY;
            _pin.period_us(250);
            wait_us(50000);
            _pin = 0;
      }
}