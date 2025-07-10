#ifndef MOTOR_LIBRARY
#define MOTOR_LIBRARY

#include "Arduino.h"
#include "driver/ledc.h"

// function to set on-board LEDs
void ledarray_set(int leds[]);

// class to contain all of the motor data and commands
class Motors{
	public:
    float cur_lm1_smoothed = 0; // current read variables
    float cur_lm2_smoothed = 0;
    float cur_rm1_smoothed = 0;
    float cur_rm2_smoothed = 0;
    float current_tot = 0;

    const float current_alpha = 0.99; // rate of change on current averaging
    // Variables to store values
    int fault;              // fault in motors indicator
    int current_lm1;        // current values sensed from left motor 1
    int current_lm2;        // current values sensed from left motor 2
    int current_rm1;        // current values sensed from right motor 1
    int current_rm2;        // current values sensed from right motor 2
    int lm_timer = millis();
    int rm_timer = millis();
    int lm_counter = 0;
    int rm_counter = 0;
    int overcurrent_error = 0;
		Motors();
    static void ledc_init(ledc_channel_t chnl, int pin, int res, int freq);
		void setup();
    void enable();
    void sample_values();
		void overcurrent_left();
		void overcurrent_right();
		void left_motors(int r_thr);
		void right_motors(int r_thr);
		void motor_speaker_left(int freq, float duration);
		void motor_speaker_right(int freq, float duration);
		void motor_speaker_both(int freq, float duration);
};
#endif