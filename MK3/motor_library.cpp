#include "motor_library.h"
#include "mk3.h"
#include "Arduino.h"
#include "driver/ledc.h"

Motors::Motors() {
  // Motor driver pins set to OUTPUT
  pinMode(LEFT_MOT_DIR_IO, OUTPUT);
  pinMode(RIGHT_MOT_DIR_IO, OUTPUT);
  
  // function to initialize LED channels, requires a specific structure filled and used
  ledc_init(LEDC_CHANNEL_2, LEFT_MOT_PWM_IO, 11, 30000);
  ledc_init(LEDC_CHANNEL_3, RIGHT_MOT_PWM_IO, 11, 30000);

  pinMode(MOTORS_NSLEEP_IO, OUTPUT);

  // Setup all input pins
  pinMode(CURRENT_LM1_IO, INPUT);
  pinMode(CURRENT_LM2_IO, INPUT);
  pinMode(CURRENT_RM1_IO, INPUT);
  pinMode(CURRENT_RM2_IO, INPUT);
  pinMode(FAULT_IO, INPUT);
}

  void Motors::ledc_init(ledc_channel_t chnl, int pin, int res, int freq){
    // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer;
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.duty_resolution = (ledc_timer_bit_t) res; // bit resolution
  ledc_timer.freq_hz = freq; // Set output frequency
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&ledc_timer);

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel;
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = chnl;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.gpio_num = pin;
  ledc_channel.duty = 0; // Set duty to 0%
  ledc_channel.hpoint = 0;
  ledc_channel_config(&ledc_channel);
}

void Motors::setup(){
  // Enable the motors
  enable();

  // startup jingle
  digitalWrite(INDICATOR_1, HIGH);
  motor_speaker_both(494, 0.08);        // play note on motor
  digitalWrite(INDICATOR_1, LOW);
  digitalWrite(INDICATOR_2, HIGH);
  motor_speaker_both(740, 0.08);
  digitalWrite(INDICATOR_1, HIGH);
  digitalWrite(INDICATOR_2, LOW); 
  delay(80);                            // rest
  digitalWrite(INDICATOR_1, LOW);
  digitalWrite(INDICATOR_2, HIGH); 
  motor_speaker_both(494, 0.08);
  digitalWrite(INDICATOR_1, HIGH);
  digitalWrite(INDICATOR_2, LOW); 
  motor_speaker_both(880, 0.08);
  digitalWrite(INDICATOR_1, LOW);
  digitalWrite(INDICATOR_2, HIGH); 
  motor_speaker_both(880, 0.08);
  digitalWrite(INDICATOR_1, LOW);
  digitalWrite(INDICATOR_2, LOW); 
}

void Motors::enable(){
  digitalWrite(MOTORS_NSLEEP_IO, HIGH);
  delay(2); // wait for next state

  // nsleep acknowledge pulse
  digitalWrite(MOTORS_NSLEEP_IO, LOW);
  delayMicroseconds(40);
  digitalWrite(MOTORS_NSLEEP_IO, HIGH);
}

void Motors::sample_values(){
  // Read sensor values
  current_lm1 = analogRead(CURRENT_LM1_IO);
  current_lm2 = analogRead(CURRENT_LM2_IO);
  current_rm1 = analogRead(CURRENT_RM1_IO);
  current_rm2 = analogRead(CURRENT_RM2_IO);
  fault = digitalRead(FAULT_IO);

  // smooth out current values, but only if nonzero
  if(current_lm1 > 0) cur_lm1_smoothed = (cur_lm1_smoothed * current_alpha) + (current_lm1 * (1 - current_alpha));
  if(current_lm2 > 0) cur_lm2_smoothed = (cur_lm2_smoothed * current_alpha) + (current_lm2 * (1 - current_alpha));
  if(current_rm1 > 0) cur_rm1_smoothed = (cur_rm1_smoothed * current_alpha) + (current_rm1 * (1 - current_alpha));
  if(current_rm2 > 0) cur_rm2_smoothed = (cur_rm2_smoothed * current_alpha) + (current_rm2 * (1 - current_alpha));
  current_tot = cur_lm1_smoothed + cur_lm2_smoothed + cur_rm1_smoothed + cur_rm2_smoothed;
}

// Overcurrent testing, Left Motors
void Motors::overcurrent_left(){
  if(lm_timer < millis()){ // See if enough time has elapsed since last check
    lm_timer = millis() + OVERTIME; // reset timer if yes
    if(cur_lm1_smoothed > CURRENTHIGH || cur_lm2_smoothed > CURRENTHIGH){ // check if the current is too high
      lm_counter++; // if yes increment the counter that dampens throttle
      overcurrent_error++;
      #ifdef DEBUG
      Serial.println("lm1 " + String(lm_counter));
      #endif
    }
    else if(lm_counter > 0){ // if not, decrement the counter
      lm_counter--;
      #ifdef DEBUG
      Serial.println("lm " + String(lm_counter));
      #endif
    }
  }
}

// Right Motors
void Motors::overcurrent_right(){
  if(rm_timer < millis()){ // See if enough time has elapsed since last check
    rm_timer = millis() + OVERTIME; // reset timer if yes
    if(cur_rm1_smoothed > CURRENTHIGH || cur_rm2_smoothed > CURRENTHIGH){ // check if the current is too high
      rm_counter++; // if yes increment the counter that dampens throttle
      overcurrent_error++;
      #ifdef DEBUG
      Serial.println("rm " + String(rm_counter));
      #endif
    }
    else if(rm_counter > 0){ // if not, decrement the counter
      rm_counter--;
      #ifdef DEBUG
      Serial.println("rm " + String(rm_counter));
      #endif
    } 
  }
}

// Function to run the left motor channels
// Takes input from -100 to 100
// -100 = full reverse, 0 = hold, 100 = full forward
void Motors::left_motors(int l_thr){
  l_thr = constrain(l_thr, -100, 100);
  // dampen throttle if overcurrents occurred 
  if(l_thr > 2 * lm_counter) l_thr = l_thr - 2 * lm_counter; 
  if (l_thr == 0){
    digitalWrite(LEFT_MOT_DIR_IO, LOW);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0); // set the duty cycle for the channel
  }
  else if (l_thr > 0){
    int val = map(l_thr, 0, 100, 0, 4096);
    digitalWrite(LEFT_MOT_DIR_IO, HIGH);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, val);
  }
  else if (l_thr < 0){
    int val = map(l_thr, -100, 0, 4096, 0);
    digitalWrite(LEFT_MOT_DIR_IO, LOW);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, val);
  }
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2); // apply the duty cycle
}

// Function to run the right motor channels
// Takes input from -100 to 100
// -100 = full reverse, 0 = hold, 100 = full forward
void Motors::right_motors(int r_thr){
  r_thr = constrain(r_thr, -100, 100);
  // dampen throttle if overcurrent events occurred
  if(r_thr > 2 * rm_counter) r_thr = r_thr - 2 * rm_counter; 
  if (r_thr == 0){
    digitalWrite(RIGHT_MOT_DIR_IO, LOW);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0); // set the duty cycle for the channel
  }
  else if (r_thr > 0){
    int val = map(r_thr, 0, 100, 0, 4096);
    digitalWrite(RIGHT_MOT_DIR_IO, LOW);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, val);
  }
  else if (r_thr < 0){
    int val = map(r_thr, -100, 0, 4096, 0);
    digitalWrite(RIGHT_MOT_DIR_IO, HIGH);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, val);
  }
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3); // apply the duty cycle
}

// Function to make the left motors emit a noise at the given frequency for the given duration. 
void Motors::motor_speaker_left(int freq, float duration){
  if(freq == 0) return;
  float delay = 1.0/(float)freq; 
  int count = duration/delay;
  for(int i = 0; i < count; i++){
    digitalWrite(LEFT_MOT_DIR_IO, (i%2));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 4095); // set the duty cycle for led channel 1
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2); // apply the duty cycle
    delayMicroseconds((delay/2)*1000000);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0); // set the duty cycle for led channel 1
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2); // apply the duty cycle
    delayMicroseconds((delay/2)*1000000);
  }
}

// Function to make the right motors emit a noise at the given frequency for the given duration. 
void Motors::motor_speaker_right(int freq, float duration){
  if(freq == 0) return;
    float delay = 1.0/(float)freq; 
    int count = duration/delay;
    for(int i = 0; i < count; i++){
    digitalWrite(RIGHT_MOT_DIR_IO, (i%2));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 4095); // set the duty cycle for led channel 1
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3); // apply the duty cycle
    delayMicroseconds((delay/2)*1000000);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0); // set the duty cycle for led channel 1
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3); // apply the duty cycle
    delayMicroseconds((delay/2)*1000000);
  }
}

// Function to make both motors emit a noise at the given frequency for the given duration. 
void Motors::motor_speaker_both(int freq, float duration){
  if(freq == 0) return;
  float delay = 1.0/(float)freq; 
  int count = duration/delay;
  for(int i = 0; i < count; i++){
    digitalWrite(LEFT_MOT_DIR_IO, (i%2));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 4095); // set the duty cycle for led channel 1
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2); // apply the duty cycle
    digitalWrite(RIGHT_MOT_DIR_IO, (i%2));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 4095); // set the duty cycle for led channel 1
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3); // apply the duty cycle
    delayMicroseconds((delay/2)*1000000);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0); // set the duty cycle for led channel 1
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2); // apply the duty cycle
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0); // set the duty cycle for led channel 1
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3); // apply the duty cycle
    delayMicroseconds((delay/2)*1000000);
  }
}
