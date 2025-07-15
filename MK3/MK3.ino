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
//Wifi wifi; // initialize wifi code

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

float STR_MIN = -75;
float STR_MAX = 75;
float STR_MIN_BASE = STR_MIN;
float STR_MAX_BASE = STR_MAX;

// Smoothed values to reduce current draw and make operation smoother
float thr_smoothed = 0;
float str_smoothed = 0;
float servo_smoothed = 1350; // Start out looking straight forward
float bat_smoothed = 11; // start at a nominal battery voltage

// Smoothing alpha values
float thr_alpha_inc = 0.985;     // Alpha value when increasing throttle power i.e. moving stick away from center
float thr_alpha_dec = 0.95;     // Alpha value when decreasing throttle power i.e. moving stick back towards center
float thr_alpha = thr_alpha_inc;// Alpha value actually used - changed throughout code

float str_alpha_inc = 0.95;     // Alpha value when increasing steering power i.e. moving stick away from center
float str_alpha_dec = 0.92;     // Alpha value when decreasing steering power i.e. moving stick back towards center
float str_alpha = str_alpha_inc;// Alpha value actually used - changed throughout code

//const float servo_alpha = 0.9; 
const float battery_alpha = 0.99;

float thr, str;
int cam_ctrl = 0;
int led2;
 
// Sensor values
int voltage_read = 0;
double battery_voltage = 0.0;

//LED array values
// led_array[8] = {orange, red, yellow, green, blue, red(rgb), green(rgb), blue(rgb)}
int led_array[8] = {0,0,0,0,0,0,0,0};

// Headlight boolean
bool headlights = 0;
int brightness = 0;
int brightness_val = 0;

int mode = 0;
// holders for disconnect continuity of behavior
int ch1holder = 0;
int ch5holder = 0;
int ch8holder = 0;

// Region around neutral where the sticks don't give an output (no motion)
const int deadzone_thr = 2;
const int deadzone_str = 2;

int loop_timer = 0; // timer for main loop execution

// wifi update variables
int update_counter = 0;
int update_timer = millis();
int last_op_mode = 0; // integer for enabling wifi updates
int operating_mode = 0; // used to switch to Wifi mode

// Setup function that runs once as the ESP starts up
// ===================================================================================================
void setup() {
  setCpuFrequencyMhz(240); // Set to 240, 160, or 80 MHz
  // Setup the LED headlight/spotlight control, also in the motors library
  mot.ledc_init(LEDC_CHANNEL_0, LED_CH_1, 12, 15625);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // headlights are off
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); // apply the duty cycle
  mot.ledc_init(LEDC_CHANNEL_1, LED_CH_2, 12, 15625);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0); // headlights are off
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1); // apply the duty cycle
  mot.ledc_init(LEDC_CHANNEL_4, LED_CH_3, 12, 15625);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 0); // headlights are off
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4); // apply the duty cycle
  mot.ledc_init(LEDC_CHANNEL_5, LED_CH_4, 12, 15625);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, 0); // headlights are off
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5); // apply the duty cycle

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

  // setup LED headlight pins
  //pinMode(LED_CH_1, OUTPUT); // LED Channel 1
  //pinMode(LED_CH_2, OUTPUT); // LED Channel 2
  //pinMode(LED_CH_3, OUTPUT); // LED Channel 3
  //pinMode(LED_CH_4, OUTPUT); // LED Channel 4
  mot.setup(); // Setup the motors
}

// Loop that repeats forever within the ESP after setup runs once
// ===================================================================================================
void loop() {
  // READ VALUES / GATHER INFORMATION 
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Read all the raw values from the reciever and store into a variable
  read_receiver(&ch1, &ch2, &ch3, &ch4, &ch5, &ch6, &ch7, &ch8, &ch9, &ch10, &ch11, &ch12); // read values from the Receiver
  ch1holder = ch1;
  ch5holder = ch5;
  ch8holder = ch8;
  // Read sensor values
  voltage_read = analogRead(BATTERY_VOLT_IO);
  mot.sample_values();
  mot.overcurrent_right(); // perform overcurrent testing
  mot.overcurrent_left();

  // RE-MAP VALUES / MAKE VALUES CLEAN AND USEFUL
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Re-map all the rx into their useful ranges

  thr = constrain(map(ch3, LOW_VAL, HIGH_VAL, -100, 100), -100, 100); // throttle
  str = constrain(map(ch4, LOW_VAL, HIGH_VAL, STR_MIN, STR_MAX), STR_MIN, STR_MAX); // steering 

  // mode = constrain(map(ch5, LOW_VAL, HIGH_VAL, -1, 3), 0, 2);
  // if(mode == 0){ //switch up
  //   str_alpha = 0.94;
  // }
  // else if( mode == 1){
  //   str_alpha = 0.965;
  // }
  // else { // switch down
  //   str_alpha = 0.99;
  // }

  // headlight toggle
  //if(failSafe || lostFrame)  headlights = constrain(map(ch5holder, LOW_VAL, HIGH_VAL, -1, 3), 0, 1); else
  brightness_val = constrain(map(ch7, LOW_VAL, HIGH_VAL,-2, 2), -1, 1);
  //cam_ctrl = constrain(map(ch10, LOW_VAL, HIGH_VAL, -1, 3), -1, 1); // camera controls
  //if(cam_ctrl == -1) cam_ctrl = 0;

  //if(cam_ctrl) digitalWrite(CAM_CTRL_IO, LOW); // set the camera control pin or unset it
  //else digitalWrite(CAM_CTRL_IO, HIGH);
  
  // operating mode switch for the WiFi mode
  //operating_mode = constrain(map(ch7, LOW_VAL, HIGH_VAL, -1, 3), 0, 2);   // switch at least three times quickly to go into wifi mode

  // adaptively change the str exponential curve based on throttle input - make more linear at higher throttles
  k_str_scalar = 1 - (abs(thr_smoothed)/100);
  k_str = k_str_base * k_str_scalar;

  // math for the exponential throttle curves
  thr /= 100.0;
  str /= STR_MAX;
  thr = (thr * (1 + k_thr*((thr * thr) -1))); // make exponential
  str = (str * (1 + k_str*((str * str) -1)));
  thr *= 100;
  str *= STR_MAX;
  thr = constrain(thr, -100, 100); // throttle

  // adapt the max steering based on the throttle - at higher throttles get more steering power
  STR_MIN = constrain(map(abs(thr_smoothed), 0, 100, STR_MIN_BASE, -101), -100, 100);
  STR_MAX = constrain(map(abs(thr_smoothed), 0, 100, STR_MAX_BASE, 101), -100, 100);
  
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
  battery_voltage = (voltage_read / 319.5) + 1.6;
  bat_smoothed = (bat_smoothed * battery_alpha) + (battery_voltage * (1 - battery_alpha));

// WIFI CODE
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
// Safeguards to make wifi mode harder to stumble into
  if(update_counter > 0){ // check if the counter needs cleared
    if(update_timer + 1000 < millis() || failSafe || lostFrame){
      update_counter = 0;
    }
  }
  // check for a full switch flip
  if(last_op_mode == 0 && operating_mode == 2 || last_op_mode == 2 && operating_mode == 0){
    update_counter++;
    update_timer = millis(); // timer till the counter resets
    last_op_mode = operating_mode;
  }
  // Over-The-Air Update code
  if(update_counter == 3){   // WiFi and update mode
    update_counter = 0;
    last_op_mode = operating_mode;
    mot.left_motors(0);  // set channels to default to ensure the crawler stops in place
    mot.right_motors(0); 
    digitalWrite(CAM_CTRL_IO, LOW); // set the camera control low to prevent recording on restart
    thr_smoothed = 0;   // zero these values so it doesn't start rolling after leaving the loop
    str_smoothed = 0;

    led_array[4] = 1;   // Turn on blue LED
    ledarray_set(led_array); 

    wifi.beginwifi();   // Turn on Wifi
    Serial.println("Waiting for connection with Mobile device...");
    update_timer = millis(); 
    while (last_op_mode == operating_mode){
      // if the mode changes, end the loop
      read_receiver(&ch1, &ch2, &ch3, &ch4, &ch5, &ch6, &ch7, &ch8, &ch9, &ch10, &ch11, &ch12);
      operating_mode = constrain(map(ch7, 172, 1811, -1, 3), 0, 2);    // 0 = normal operation, 2 = WiFi and update mode
      if(((update_timer + 3000) > millis()) && (operating_mode != 1)){ // 3 second timeout before you can leave wifi mode
        last_op_mode = operating_mode; // stay in this loop
      }
      if(operating_mode == 1) operating_mode = last_op_mode; // ignore middle switch state
      wifi.start_transmission(); // check for connection for transmission
      // slowly turn headlights on and off
      if(millis() % 3000 > 1500) ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1000); // set the duty cycle for led channel 2
      else ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0); // Else headlights are off
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1); // apply the duty cycle

      delay(200); // slow loop down
    }
    setVTX(); // set vtx back to defaults
    Serial.println("Shutting down Wifi");
    wifi.endwifi(); // turn off wifi
    led_array[4] = 0; // turn off blue LED
    ledarray_set(led_array); 
    mot.enable(); // reenable motors
  }
*/
  // ACTION TAKEN BASED VALUES / WRITE TO OUTPUTS
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //logarithmic lighting
  //if(failSafe || lostFrame) led2 = log_lighting(ch8holder);
  //led2 = log_lighting(ch8);

  // control the LEDs. Had to inverse the values for some reason.
  //if(headlights) ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, led2); // set the duty cycle for led channel 2
  //else ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0); // Else headlights are off
  //ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1); // apply the duty cycle

  // check if brightness would be out of bounds here
  brightness += brightness_val*10;
  if(brightness < 0) brightness = 0;
  if(brightness > 4096) brightness = 4096;
  //Serial.println(brightness);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness); // set the duty cycle for led channel 2
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); // apply the duty cycle
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, brightness); // set the duty cycle for led channel 2
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1); // apply the duty cycle
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, brightness); // set the duty cycle for led channel 2
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4); // apply the duty cycle
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, brightness); // set the duty cycle for led channel 2
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5); // apply the duty cycle
  
  // If battery is close to dead turn on red led
  //if(battery_voltage < 9.9) led_array[1] = 1;
  //else led_array[1] = 0;

  // if fault from motor drivers turn on yellow led
  //if(mot.fault == 0) led_array[0] = 1;
  //else led_array[0] = 0;
 
  // handle motor controls
  if ((thr_smoothed > -deadzone_thr && thr_smoothed < deadzone_thr) &&
      (str_smoothed > -deadzone_thr && str_smoothed < deadzone_thr)){
    mot.left_motors(0);
    mot.right_motors(0);
  }
  else{
    mot.left_motors(thr_smoothed + str_smoothed);
    mot.right_motors(thr_smoothed - str_smoothed);
  }

  //ledarray_set(led_array);
 
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

