// marten_pro.h
#ifndef MARTEN_LIBRARY
#define MARTEN_LIBRARY

/***************************************************************************************************/
// Marten specific information for logging purposes
#define CRAWLER_VERSION "MK3"
#define CRAWLER_NUM "001"
#define MANUFACTURE_DATE "7/3/25"

/***************************************************************************************************/
#define CURRENTHIGH 3000 // current reading which going above signifies overcurrent
#define OVERTIME 100 // time between overcurrent samples 

/***************************************************************************************************/

// define the rx and tx pins on the ESP
#define RXD2 16
#define TXD2 17

#define LOOP_TIME 2 // amount of minimum milliseconds in a loop, 1 = 1kHz, 2 = 500Hz

#define CURRENTHIGH 3000 // current reading which going above signifies overcurrent
#define OVERTIME 100 // time between overcurrent samples 

#define LOW_VAL 272
#define HIGH_VAL 1712
#define GIMBAL_DEFAULT 991

#define DEFAULT (HIGH_VAL + LOW_VAL)/2 // get the middle value

#define BOARD_PWR 23

#define INDICATOR_1 13
#define INDICATOR_2 18

#define LED_CH_1 4
#define LED_CH_2 0
#define LED_CH_3 5
#define LED_CH_4 19

#define SDA 21
#define SCL 22
#define PWR_SW 26
#define IO2 2

#define LEFT_MOT_DIR_IO 14  // left motor direction
#define LEFT_MOT_PWM_IO 12  // left motor PWM
#define RIGHT_MOT_DIR_IO 32 // right motor direction
#define RIGHT_MOT_PWM_IO 33 // right motor PWM

#define MOTORS_NSLEEP_IO 15 // motor sleep pin

#define CURRENT_LM1_IO 36   // current sense left motor 1 (Sensor_VP)
#define CURRENT_LM2_IO 39   // current sense left motor 2 (Sensor_VN)
#define CURRENT_RM1_IO 34   // current sense right motor 1
#define CURRENT_RM2_IO 35   // current sense right motor 2

#define FAULT_IO 25         // fault sense - active low - low value indiates fault from one of the motor drivers

#define BATTERY_VOLT_IO 27     // analog pin to the battery level voltage


#endif