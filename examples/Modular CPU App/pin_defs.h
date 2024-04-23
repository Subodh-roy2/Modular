
#ifndef PIN_DEFS_H
#define PIN_DEFS_H
//define pins for board leds
#define BOARD_LED1 0
#define BOARD_LED2 1
#define BOARD_LED3 2
#define BOARD_LED4 4
#define BOARD_LED5 7
#define BOARD_LED6 8

//define pins for ultrasonic at PORT 2
#define SONAR_SENSOR_TRIG A4
#define SONAR_SENSOR_ECHO A5

//define pins for proximity sensor at PORT 2
#define PROXIMITY_SENSOR_1 A6
#define PROXIMITY_SENSOR_2 A7

//define buzzer and rgb led pins at port 3
#define RGB_RED 3
#define RGB_GREEN 5
#define RGB_BLUE 6
#define BUZZER 9

//define move bot motor pins at port 4
#define MOTOR_LEFT_1 10
#define MOTOR_LEFT_2 11
#define MOTOR_RIGHT_1 12
#define MOTOR_RIGHT_2 13

//define free motor pins 
#define MOTOR_1_1 3
#define MOTOR_1_2 5
#define MOTOR_2_1 6
#define MOTOR_2_2 9

//define servo pins
#define SERVO_1 3
#define SERVO_2 5
#define SERVO_3 6
#define SERVO_4 9

//define software serial pins for bluetooth at port 1
#define SOFT_RX A0
#define SOFT_TX A1

#endif
