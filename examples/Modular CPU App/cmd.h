
#ifndef CMD_H
#define CMD_H

#define BOARD_LED_1_OFF 0xA0
#define BOARD_LED_1_ON 0xA1
#define BOARD_LED_2_OFF 0xA2
#define BOARD_LED_2_ON 0xA3
#define BOARD_LED_3_OFF 0xA4
#define BOARD_LED_3_ON 0xA5
#define BOARD_LED_4_OFF 0xA6
#define BOARD_LED_4_ON 0xA7
#define BOARD_LED_5_OFF 0xA8
#define BOARD_LED_5_ON 0xA9
#define BOARD_LED_6_OFF 0xAA
#define BOARD_LED_6_ON 0xAB

#define MOVE_BOT_FWD 0xB0
#define MOVE_BOT_BWD 0xB1
#define MOVE_BOT_LT 0xB2
#define MOVE_BOT_RT 0xB3
#define BOT_STOP 0xB4

#define MOTOR_1_CLKWISE 0xB5
#define MOTOR_1_ACLKWISE 0xB6
#define MOTOR_1_STOP 0xB7
#define MOTOR_2_CLKWISE 0xB8
#define MOTOR_2_ACLKWISE 0xB9
#define MOTOR_2_STOP 0xBA

#define SERVO_1_ANGLE 0xBB
#define SERVO_2_ANGLE 0xBC
#define SERVO_3_ANGLE 0xBD
#define SERVO_4_ANGLE 0xBE

#define BUZZER_ON 0xC0
#define BUZZER_OFF 0xC1
#define RED_LED_COLOR 0xC2
#define GREEN_LED_COLOR 0xC3
#define BLUE_LED_COLOR 0xC4

#define ULTRASONIC_READ 0xD0
#define PROXIMITY_ONE_READ 0xD1
#define PROXIMITY_TWO_READ 0xD2
#define SET_PS_ONE_THRESHOLD 0xD3
#define SET_PS_TWO_THRESHOLD 0xD4
#define PLAY_LINE_FOLLOWER 0xD5
#define STOP_LINE_FOLLOWER 0xD6

#endif