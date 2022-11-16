#ifndef MBED_SPEAKER_H
#define MBED_SPEAKER_H

#include "mbed.h"

#define SPEAKER_DUTY 0.1

class speaker {
     public:
      speaker(PinName speaker_pin_);
      void run(uint16_t sound_scale, uint16_t sound_time);
      void startup_sound(uint8_t mode);

     private:
      PwmOut speaker_pin;
};

#endif