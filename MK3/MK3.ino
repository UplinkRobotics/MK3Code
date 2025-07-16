// Uplinkrobotics MK3 Cor Code
// V1.0
//
// Copyright © 2025 UplinkRobotics LLC

// Include libraries
#include "SBUS.h"
#include <ESP32Servo.h>
#include "driver/ledc.h"
#include "motor_library.h"
#include "mk3.h"
#include "ota_updates.h"

/***************************************************************************************************/

// define the rx and tx pins on the ESP
#define RXD2 16
#define TXD2 17

SBUS rxsr(Serial2); //SBUS object
uint16_t channels[16];
bool failSafe;
bool lostFrame;

Motors mot; // initialize motors
Wifi wifi; // initialize wifi code

// Radio channel integers - raw values from the radio
int ch1 = GIMBAL_DEFAULT; // set to default value so crawler doesnt move on startup
int ch2 = DEFAULT; 
int ch3 = DEFAULT;
int ch4 = DEFAULT;
int ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12;

// exponential factor for driving
float k_thr = 0.5; //throttle
float k_str = 0.2; //steering
float k_str_scalar = 1.0;
float k_str_base = k_str;

float THR_MIN = -100;
float THR_MAX = 100;
float STR_MIN = -75;
float STR_MAX = 75;
float STR_MIN_BASE = STR_MIN;
float STR_MAX_BASE = STR_MAX;

// Smoothed values to reduce current draw and make operation smoother
float thr_smoothed = 0;
float str_smoothed = 0;
float bat_smoothed = 11; // start at a nominal battery voltage

// Smoothing alpha values
float thr_alpha_inc = 0.985;      // Alpha value when increasing throttle power i.e. moving stick away from center
float thr_alpha_dec = 0.95;       // Alpha value when decreasing throttle power i.e. moving stick back towards center
float thr_alpha = thr_alpha_inc;  // Alpha value actually used - changed throughout code

float str_alpha_inc = 0.95;       // Alpha value when increasing steering power i.e. moving stick away from center
float str_alpha_dec = 0.92;       // Alpha value when decreasing steering power i.e. moving stick back towards center
float str_alpha = str_alpha_inc;  // Alpha value actually used - changed throughout code

const float battery_alpha = 0.99;

float thr, str;
 
// Sensor values
int voltage_read = 0;
double battery_voltage = 0.0;

// Headlight values
int brightness = 300;   // current brightness value
int brightness_val = 0; // increasing or decreasing brightness
int headlight_mode = 0; // state of the headlight control

int mode = 0;

// Region around neutral where the sticks don't give an output (no motion)
const int deadzone_thr = 2;
const int deadzone_str = 2;

int loop_timer = 0; // timer for main loop execution

// wifi update variables
bool button_c = 0; // values for wifi buttons
bool button_d = 0;
int update_timer = millis(); // timer to get into wifi mode

// Setup function that runs once as the ESP starts up
// ===================================================================================================
void setup() {
  setCpuFrequencyMhz(240); // Set to 240, 160, or 80 MHz
  // Setup the LED headlight/spotlight control, also in the motors library
  mot.ledc_init(LEDC_CHANNEL_0, LED_CH_1, 12, 15625);
  mot.ledc_init(LEDC_CHANNEL_0, LED_CH_2, 12, 15625);
  mot.ledc_init(LEDC_CHANNEL_0, LED_CH_3, 12, 15625);
  mot.ledc_init(LEDC_CHANNEL_0, LED_CH_4, 12, 15625);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // headlights are off
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); // apply the duty cycle

  Serial.begin(921600);     // debug info
  Serial.flush();

  rxsr.begin(RXD2, TXD2, true, 100000); // optional parameters for ESP32: RX pin, TX pin, inverse mode, baudrate
  
  // Setup on board indicator LEDs
  pinMode(INDICATOR_1, OUTPUT);
  pinMode(INDICATOR_2, OUTPUT);
  digitalWrite(INDICATOR_1, LOW);
  digitalWrite(INDICATOR_2, LOW); //TODO: set pins to starting value

  pinMode(SDA, OUTPUT);
  pinMode(SCL, OUTPUT);
  pinMode(PWR_SW, OUTPUT);
  pinMode(BOARD_PWR, OUTPUT);
  pinMode(IO2, OUTPUT);

  mot.setup(); // Setup the motors
}

// Loop that repeats forever within the ESP after setup runs once
// ===================================================================================================
void loop() {
  // READ VALUES / GATHER INFORMATION 
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Read all the raw values from the reciever and store into a variable
  read_receiver(&ch1, &ch2, &ch3, &ch4, &ch5, &ch6, &ch7, &ch8, &ch9, &ch10, &ch11, &ch12); // read values from the Receiver
  // Read sensor values
  voltage_read = analogRead(BATTERY_VOLT_IO);
  mot.sample_values();
  mot.overcurrent_right(); // perform overcurrent testing
  mot.overcurrent_left();

  // RE-MAP VALUES / MAKE VALUES CLEAN AND USEFUL
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Re-map all the rx into their useful ranges

  mode = constrain(map(ch6, LOW_VAL, HIGH_VAL, -1, 3), 0, 2); // driving mode
  if(mode == 2){ //switch up - unstuck Mode
    thr_alpha_inc = 0.25;     // Alpha value when increasing throttle power i.e. moving stick away from center
    thr_alpha_dec = 0.25;     // Alpha value when decreasing throttle power i.e. moving stick back towards center

    str_alpha_inc = 0.25;     // Alpha value when increasing steering power i.e. moving stick away from center
    str_alpha_dec = 0.25;     // Alpha value when decreasing steering power i.e. moving stick back towards center

    THR_MIN = -100;
    THR_MAX = 100;
    STR_MIN = -100;
    STR_MAX = 100;
    STR_MIN_BASE = STR_MIN;
    STR_MAX_BASE = STR_MAX;

    k_thr = 0.0; //throttle
    k_str = 0.0; //steering
    k_str_scalar = 1.0;
    k_str_base = k_str;
  }
  else if( mode == 1){ // switch middle - Inspection mode
    thr_alpha_inc = 0.99;     // Alpha value when increasing throttle power i.e. moving stick away from center
    thr_alpha_dec = 0.96;     // Alpha value when decreasing throttle power i.e. moving stick back towards center

    str_alpha_inc = 0.97;     // Alpha value when increasing steering power i.e. moving stick away from center
    str_alpha_dec = 0.90;     // Alpha value when decreasing steering power i.e. moving stick back towards center

    THR_MIN = -75;
    THR_MAX = 75;
    STR_MIN = -60;
    STR_MAX = 60;
    STR_MIN_BASE = STR_MIN;
    STR_MAX_BASE = STR_MAX;

    k_thr = 0.3; //throttle
    k_str = 0.1; //steering
    k_str_scalar = 1.0;
    k_str_base = k_str;
  }
  else { // switch down - travel mode
    thr_alpha_inc = 0.985;     // Alpha value when increasing throttle power i.e. moving stick away from center
    thr_alpha_dec = 0.95;     // Alpha value when decreasing throttle power i.e. moving stick back towards center

    str_alpha_inc = 0.95;     // Alpha value when increasing steering power i.e. moving stick away from center
    str_alpha_dec = 0.92;     // Alpha value when decreasing steering power i.e. moving stick back towards center

    THR_MIN = -100;
    THR_MAX = 100;
    STR_MIN = -75;
    STR_MAX = 75;
    STR_MIN_BASE = STR_MIN;
    STR_MAX_BASE = STR_MAX;

    k_thr = 0.5; //throttle
    k_str = 0.15; //steering
    k_str_scalar = 1.0;
    k_str_base = k_str;
  }

  thr = constrain(map(ch2, LOW_VAL, HIGH_VAL, THR_MIN, THR_MAX), THR_MIN, THR_MAX); // throttle
  str = constrain(map(ch1, LOW_VAL, HIGH_VAL, STR_MIN, STR_MAX), STR_MIN, STR_MAX); // steering 

  // adaptively change the str exponential curve based on throttle input - make more linear at higher throttles
  k_str_scalar = 1 - (abs(thr_smoothed)/100);
  k_str = k_str_base * k_str_scalar;

  // math for the exponential throttle curves
  thr /= THR_MAX;
  str /= STR_MAX;
  thr = (thr * (1 + k_thr*((thr * thr) -1))); // make exponential
  str = (str * (1 + k_str*((str * str) -1)));
  thr *= THR_MAX;
  str *= STR_MAX;
  thr = constrain(thr, THR_MIN, THR_MAX); // throttle

  // adapt the max steering based on the throttle - at higher throttles get more steering power
  STR_MIN = constrain(map(abs(thr_smoothed), 0, THR_MAX, STR_MIN_BASE, THR_MIN), THR_MIN, THR_MAX);
  STR_MAX = constrain(map(abs(thr_smoothed), 0, THR_MAX, STR_MAX_BASE, THR_MAX), THR_MIN, THR_MAX);
  
  str = constrain(str, STR_MIN, STR_MAX);   // steering

  // code to change the alpha values based on what the throttle is doing
  if ((abs(thr) - abs(thr_smoothed)) >= 0){ // throttle is increasing
    thr_alpha = thr_alpha_inc;
  }
  else{                                     // throttle is decreasing
    thr_alpha = thr_alpha_dec;
  }

  if ((abs(str) - abs(str_smoothed)) >= 0){ // steering is increasing
    str_alpha = str_alpha_inc;
  }
  else{
    str_alpha = str_alpha_dec;              // steering is decreasing
  }

  // smooth out throttle, steering, and servo
  thr_smoothed = (thr_smoothed * thr_alpha) + (thr * (1 - thr_alpha));
  str_smoothed = (str_smoothed * str_alpha) + (str * (1 - str_alpha));
 
  // Calculate the voltage based on the analog value
  battery_voltage = (voltage_read / 319.5) + 1.6; //TODO: redo this equation
  bat_smoothed = (bat_smoothed * battery_alpha) + (battery_voltage * (1 - battery_alpha));

// WIFI CODE
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

button_c = constrain(map(ch10, LOW_VAL, HIGH_VAL, -1, 2), 0, 1); // wifi button 1
button_d = constrain(map(ch11, LOW_VAL, HIGH_VAL, -1, 2), 0, 1); // wifi button 2

// Safeguards to make wifi mode harder to stumble into
  if(!button_c && !button_d) update_timer = millis();
  if(millis() > update_timer + 3000){
    mot.left_motors(0);  // set channels to default to ensure the crawler stops in place
    mot.right_motors(0); 
    thr_smoothed = 0;   // zero these values so it doesn't start rolling after leaving the loop
    str_smoothed = 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 500); // set LED brightness to lowish
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); // apply the duty cycle
    digitalWrite(INDICATOR_1, HIGH); // turn on blue LED
    while(button_c || button_d){ // hold till the buttons are released
      read_receiver(&ch1, &ch2, &ch3, &ch4, &ch5, &ch6, &ch7, &ch8, &ch9, &ch10, &ch11, &ch12);
      button_c = constrain(map(ch10, LOW_VAL, HIGH_VAL, -1, 2), 0, 1); // wifi button 1
      button_d = constrain(map(ch11, LOW_VAL, HIGH_VAL, -1, 2), 0, 1); // wifi button 2
    };
    wifi.beginwifi();   // Turn on Wifi
    Serial.println("Waiting for connection with Mobile device...");
    while (!button_c || !button_d){
      // if the mode changes, end the loop
      wifi.start_transmission(); // check for connection for transmission
      // slowly turn headlights on and off
      if(millis() % 3000 > 1500) ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 300); // set the duty cycle for led channel 2
      else ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // Else headlights are off
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); // apply the duty cycle
      int temptime = millis() + 200; // slight delay to slow loop down
      while(temptime > millis()){
        read_receiver(&ch1, &ch2, &ch3, &ch4, &ch5, &ch6, &ch7, &ch8, &ch9, &ch10, &ch11, &ch12);
        button_c = constrain(map(ch10, LOW_VAL, HIGH_VAL, -1, 2), 0, 1); // wifi button 1
        button_d = constrain(map(ch11, LOW_VAL, HIGH_VAL, -1, 2), 0, 1); // wifi button 2
      }
    }
    Serial.println("Shutting down Wifi");
    wifi.endwifi(); // turn off wifi
    digitalWrite(INDICATOR_1, LOW); // turn on blue LED 
    mot.enable(); // reenable motors
    update_timer = millis();
  }

  // ACTION TAKEN BASED VALUES / WRITE TO OUTPUTS
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  headlight_mode = constrain(map(ch5, LOW_VAL, HIGH_VAL, 0, 2), 0, 2); 
  brightness_val = constrain(map(ch12, LOW_VAL, HIGH_VAL, 20, -20), -10, 10);
  // control the LEDs
  if(headlight_mode == 2) ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // turn headlights off
  else if(headlight_mode == 1){
    // check if brightness would be out of bounds here
    brightness += brightness_val;
    if(brightness < 0) brightness = 0;
    if(brightness > 4096) brightness = 4096;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness); // set to variable brightness
  }
  else ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4096); // set to max brightness

  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); // apply the duty cycle
  
  // If battery is close to dead turn on red led
  //if(battery_voltage < 9.9) led_array[1] = 1;
  //else led_array[1] = 0;

  // if fault from motor drivers turn on yellow led
  //if(mot.fault == 0) led_array[0] = 1;
  //else led_array[0] = 0;
 
  // handle motor controls // TODO: move this inside motor driver file
  if ((thr_smoothed > -deadzone_thr && thr_smoothed < deadzone_thr) &&
      (str_smoothed > -deadzone_thr && str_smoothed < deadzone_thr)){
    mot.left_motors(0);
    mot.right_motors(0);
  }
  else{
    mot.left_motors(thr_smoothed + str_smoothed);
    mot.right_motors(thr_smoothed - str_smoothed);
  }
 
  // check loop timer, make sure the loop isnt taking way too long
  if(millis() < LOOP_TIME + loop_timer) while(millis() < LOOP_TIME + loop_timer);
  // Serial.println(1000.0/(millis() - loop_timer));  // Print the frequency that the loop is running at
  loop_timer = millis(); // reset loop timer
}

/////////////////////////// END of LOOP ////////////////////////////////////////////////////////////////


// Function to apply logarithmic lighting to the LEDs
int log_lighting(int ch){
  if(ch < 173) return 0; // cut off anything below this point, don't waste current when LEDs aren't on
  float temp = ch * 0.00295; //lot of magic values specifically for SBUS
  temp = (exp(temp) + 25 + 0.154 * ch) * 8;
  return constrain(temp, 0, 4096);
}

// Function to read values from the receiver 
void read_receiver(int *ch1, int *ch2, int *ch3, int *ch4, int *ch5, int *ch6, int *ch7, int *ch8, int *ch9, int *ch10, int *ch11, int *ch12){
  // look for a good SBUS packet from the receiver
  if(rxsr.read(&channels[0], &failSafe, &lostFrame)){
    *ch1 = channels[0];
    *ch2 = channels[1];
    *ch3 = channels[2];
    *ch4 = channels[3];
    *ch5 = channels[4];
    *ch6 = channels[5];
    *ch7 = channels[6];
    *ch8 = channels[7];
    *ch9 = channels[8];
    *ch10 = channels[9];
    *ch11 = channels[10];
    *ch12 = channels[11];

    // sanitize values, usually only matters on controller turn off
    if(lostFrame || failSafe){
      //*ch1 = GIMBAL_DEFAULT;
      *ch2 = DEFAULT;
      *ch3 = DEFAULT;
      *ch4 = DEFAULT;
      //*ch5 = LOW_VAL;
      *ch6 = LOW_VAL; 
      *ch7 = LOW_VAL; 
      //*ch8 = LOW_VAL; 
      *ch9 = LOW_VAL; 
      *ch10 = LOW_VAL;
      *ch11 = LOW_VAL;
    }
  }
}

