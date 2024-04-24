/*

   designed for Atmega328p/atmega168p based boards
*/

#include "pin_defs.h" //library for pin definitions
#include "cmd.h" //library for communication commands
#include <Ultrasonic.h> //library for ultrasonic sensor
#include <NeoSWSerial.h>
#include <Servo.h>

//#define SERIAL_DEBUG //if define, the pins 0 and 1 will be used for serial communcation

//create ultrasonic object
Ultrasonic ultrasonic;

//create serial communication object
NeoSWSerial bluetooth_Serial;

//variable to hold serial data
volatile byte serial_cmd = 0;

//variable to store ultrasonic distance
int distance = 0;

//variable to store proximity sensor reading
int proximity_value = 0;

//value to store black and white threshold value
int ps1_threshold = 150;
int ps2_threshold = 150;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void move_bot_left();
void move_bot_right();
void move_bot_forward();
void move_bot_backward();
void bot_stop();
void move_motor1_clkwise();
void move_motor1_aclkwise();
void motor1_stop();
void move_motor2_clkwise();
void move_motor2_aclkwise();
void motor2_stop();
void play_line_follower();
void stop_line_follower();
int p[4] = {1, 4, 7, 8};
void setup() {
  //#ifdef SERIAL_DEBUG //use pins 0 and 1 for serial debugging
  //  Serial.begin(115200);
  //#else //else use pins for led
  //  pinMode(BOARD_LED1, OUTPUT);
  //  pinMode(BOARD_LED2, OUTPUT);
  //#endif
  bluetooth_Serial.begin(SOFT_RX, SOFT_TX);
  ultrasonic.begin(SONAR_SENSOR_TRIG, SONAR_SENSOR_ECHO);
  //define digital input, output pins
  //  pinMode(BOARD_LED3, OUTPUT);
  //  pinMode(BOARD_LED4, OUTPUT);
  //  pinMode(BOARD_LED5, OUTPUT);
  //  pinMode(BOARD_LED6, OUTPUT);

  //define bot driving motor pins
  pinMode(MOTOR_LEFT_1, OUTPUT);
  pinMode(MOTOR_LEFT_2, OUTPUT);
  pinMode(MOTOR_RIGHT_1, OUTPUT);
  pinMode(MOTOR_RIGHT_2, OUTPUT);

  //define PORT3 pins as output by default
  //  pinMode(3, OUTPUT);
  //  pinMode(5, OUTPUT);
  //  pinMode(6, OUTPUT);
  //  pinMode(9, OUTPUT);
  //
  //  digitalWrite(3, LOW);
  //  digitalWrite(5, LOW);
  //  digitalWrite(6, LOW);
  //  digitalWrite(9, LOW);


  //  for (int i = 0; i < 5; i++) {
  //    digitalWrite(BOARD_LED3, HIGH);
  //    digitalWrite(BOARD_LED4, HIGH);
  //    digitalWrite(BOARD_LED5, HIGH);
  //    digitalWrite(BOARD_LED6, HIGH);
  //    delay(200);
  //    digitalWrite(BOARD_LED3, LOW);
  //    digitalWrite(BOARD_LED4, LOW);
  //    digitalWrite(BOARD_LED5, LOW);
  //    digitalWrite(BOARD_LED6, LOW);
  //    delay(200);
  //  }
  for (int i = 0; i <= 3; i++) {
    pinMode(p[i], OUTPUT);
  }
  on(0);
  for (int a = 0; a < 3; a++) {
    on3(1);
    on3(0);
  }
  for (int a = 0; a < 3; a++) {
    on2(1);
    on2(0);
  }
  for (int a = 0; a < 20; a++) {
    on(1);
    delay(50);
    on(0);
    delay(50);
  }
//  digitalWrite(p[0], 1);
}

//flags to check if pins has been attached to servo
bool attach_servo_1 = 0;
bool attach_servo_2 = 0;
bool attach_servo_3 = 0;
bool attach_servo_4 = 0;

void loop() {

  //check if a command is received
  if (bluetooth_Serial.available() > 0) {
    serial_cmd = bluetooth_Serial.read();
    //#ifdef SERIAL_DEBUG
    //    Serial.println(serial_cmd, HEX);
    //#endif
    switch (serial_cmd) {
      case BOARD_LED_1_OFF :
        digitalWrite(BOARD_LED1, HIGH);
        break;
      case BOARD_LED_1_ON :
        digitalWrite(BOARD_LED1, LOW);
        break;
      case BOARD_LED_2_OFF :
        digitalWrite(BOARD_LED2, HIGH);
        break;
      case BOARD_LED_2_ON :
        digitalWrite(BOARD_LED2, LOW);
        break;
      case BOARD_LED_3_OFF :
        digitalWrite(BOARD_LED3, LOW);
        break;
      case BOARD_LED_3_ON :
        digitalWrite(BOARD_LED3, HIGH);
        break;
      case BOARD_LED_4_OFF :
        digitalWrite(BOARD_LED4, LOW);
        break;
      case BOARD_LED_4_ON :
        digitalWrite(BOARD_LED4, HIGH);
        break;
      case BOARD_LED_5_OFF :
        digitalWrite(BOARD_LED5, LOW);
        break;
      case BOARD_LED_5_ON :
        digitalWrite(BOARD_LED5, HIGH);
        break;
      case BOARD_LED_6_OFF :
        digitalWrite(BOARD_LED6, LOW);
        break;
      case BOARD_LED_6_ON :
        digitalWrite(BOARD_LED6, HIGH);
        break;
      case MOVE_BOT_FWD :
        move_bot_forward();
        break;
      case MOVE_BOT_BWD :
        move_bot_backward();
        break;
      case MOVE_BOT_LT :
        move_bot_left();
        break;
      case MOVE_BOT_RT :
        move_bot_right();
        break;
      case BOT_STOP :
        bot_stop();
        break;
      case MOTOR_1_CLKWISE :
        //if servo is attached to the pin, then first detach it
        if (attach_servo_1 || attach_servo_2) {
          servo1.detach();
          servo2.detach();
          pinMode(MOTOR_1_1, OUTPUT);
          pinMode(MOTOR_1_2, OUTPUT);
          attach_servo_1 = 0;
          attach_servo_2 = 0;
        }
        else;
        move_motor1_clkwise();
        break;
      case MOTOR_1_ACLKWISE :
        //if servo is attached to the pin, then first detach it
        if (attach_servo_1 || attach_servo_2) {
          servo1.detach();
          servo2.detach();
          pinMode(MOTOR_1_1, OUTPUT);
          pinMode(MOTOR_1_2, OUTPUT);
          attach_servo_1 = 0;
          attach_servo_2 = 0;
        }
        else;
        move_motor1_aclkwise();
        break;
      case MOTOR_1_STOP :
        //if servo is attached to the pin, then first detach it
        if (attach_servo_1 || attach_servo_2) {
          servo1.detach();
          servo2.detach();
          pinMode(MOTOR_1_1, OUTPUT);
          pinMode(MOTOR_1_2, OUTPUT);
          attach_servo_1 = 0;
          attach_servo_2 = 0;
        }
        else;
        motor1_stop();
        break;
      case MOTOR_2_CLKWISE :
        //if servo is attached to the pin, then first detach it
        if (attach_servo_3 || attach_servo_4) {
          servo3.detach();
          servo4.detach();
          pinMode(MOTOR_2_1, OUTPUT);
          pinMode(MOTOR_2_2, OUTPUT);
          attach_servo_3 = 0;
          attach_servo_4 = 0;
        }
        else;
        move_motor2_clkwise();
        break;
      case MOTOR_2_ACLKWISE :
        //if servo is attached to the pin, then first detach it
        if (attach_servo_3 || attach_servo_4) {
          servo3.detach();
          servo4.detach();
          pinMode(MOTOR_2_1, OUTPUT);
          pinMode(MOTOR_2_2, OUTPUT);
          attach_servo_3 = 0;
          attach_servo_4 = 0;
        }
        else;
        move_motor2_aclkwise();
        break;
      case MOTOR_2_STOP :
        //if servo is attached to the pin, then first detach it
        if (attach_servo_3 || attach_servo_4) {
          servo3.detach();
          servo4.detach();
          pinMode(MOTOR_2_1, OUTPUT);
          pinMode(MOTOR_2_2, OUTPUT);
          attach_servo_3 = 0;
          attach_servo_4 = 0;
        }
        else;
        motor2_stop();
        break;
      case SERVO_1_ANGLE :
        if (attach_servo_1 == 0) {
          servo1.attach(SERVO_1);
          attach_servo_1 = 1;
        }
        else;
        while (!bluetooth_Serial.available());
        //read servo angle received from bluetooth
        serial_cmd = bluetooth_Serial.read();
        Serial.print("SERVO1 :"); Serial.println(serial_cmd);
        servo1.write(serial_cmd); //set angle
        break;
      case SERVO_2_ANGLE :
        if (attach_servo_2 == 0) {
          servo2.attach(SERVO_2);
          attach_servo_2 = 1;
        }
        else;
        while (!bluetooth_Serial.available());
        //read servo angle received from bluetooth
        serial_cmd = bluetooth_Serial.read();
        Serial.print("SERVO2 :"); Serial.println(serial_cmd);
        servo2.write(serial_cmd); //set angle
        break;
      case SERVO_3_ANGLE :
        if (attach_servo_3 == 0) {
          servo3.attach(SERVO_3);
          attach_servo_3 = 1;
        }
        else;
        while (!bluetooth_Serial.available());
        //read servo angle received from bluetooth
        serial_cmd = bluetooth_Serial.read();
        Serial.print("SERVO3 :"); Serial.println(serial_cmd);
        servo3.write(serial_cmd); //set angle
        break;
      case SERVO_4_ANGLE :
        if (attach_servo_4 == 0) {
          servo4.attach(SERVO_4);
          attach_servo_4 = 1;
        }
        else;
        while (!bluetooth_Serial.available());
        //read servo angle received from bluetooth
        serial_cmd = bluetooth_Serial.read();
        Serial.print("SERVO4 :"); Serial.println(serial_cmd);
        servo4.write(serial_cmd); //set angle
        break;
      case ULTRASONIC_READ :
        //read distance in centimeter
        distance = ultrasonic.read();
        distance = constrain(distance, 0, 255);
        bluetooth_Serial.write(distance & 0xff);
        //#ifdef SERIAL_DEBUG
        //        Serial.println(distance & 0xff);
        //#endif
        break;
      case PROXIMITY_ONE_READ :
        //read proximity 1 sensor
        proximity_value = analogRead(PROXIMITY_SENSOR_1);
        proximity_value = map(proximity_value, 0, 1023, 0, 100);
        bluetooth_Serial.write(proximity_value & 0xff);
        break;
      case PROXIMITY_TWO_READ :
        //read proximity 1 sensor
        proximity_value = analogRead(PROXIMITY_SENSOR_2);
        proximity_value = map(proximity_value, 0, 1023, 0, 100);
        bluetooth_Serial.write(proximity_value & 0xff);
        break;
      case BUZZER_ON :
        if (attach_servo_4) {
          servo4.detach();
          pinMode(BUZZER, OUTPUT);
          attach_servo_4 = 0;
        }
        else;
        analogWrite(BUZZER, 255);
        break;
      case BUZZER_OFF :
        if (attach_servo_4) {
          servo4.detach();
          pinMode(BUZZER, OUTPUT);
          attach_servo_4 = 0;
        }
        else;
        analogWrite(BUZZER, 0);
        break;
      case RED_LED_COLOR :
        if (attach_servo_1) {
          servo1.detach();
          attach_servo_1 = 0;
        }
        else;
        //wait for red colour hex code
        while (!bluetooth_Serial.available());
        //read red colour value
        serial_cmd = bluetooth_Serial.read();
        serial_cmd = constrain(serial_cmd, 0, 255);
        serial_cmd = map(serial_cmd, 0, 255, 255, 0);
        analogWrite(RGB_RED, serial_cmd);
        break;
      case GREEN_LED_COLOR :
        if (attach_servo_2) {
          servo2.detach();
          attach_servo_2 = 0;
        }
        else;
        while (!bluetooth_Serial.available());
        //read red colour value
        serial_cmd = bluetooth_Serial.read();
        serial_cmd = constrain(serial_cmd, 0, 255);
        serial_cmd = map(serial_cmd, 0, 255, 255, 0);
        analogWrite(RGB_GREEN, serial_cmd);
        break;
      case BLUE_LED_COLOR :
        if (attach_servo_3) {
          servo3.detach();
          attach_servo_3 = 0;
        }
        else;
        while (!bluetooth_Serial.available());
        //read red colour value
        serial_cmd = bluetooth_Serial.read();
        serial_cmd = constrain(serial_cmd, 0, 255);
        serial_cmd = map(serial_cmd, 0, 255, 255, 0);
        analogWrite(RGB_BLUE, serial_cmd);
        break;
      case SET_PS_ONE_THRESHOLD :
        //wait for value from bluetooth
        while (!bluetooth_Serial.available());
        //read ps1 threshold value
        ps1_threshold = bluetooth_Serial.read();
        break;
      case SET_PS_TWO_THRESHOLD :
        //wait for value from bluetooth
        while (!bluetooth_Serial.available());
        //read ps2 threshold value
        ps2_threshold = bluetooth_Serial.read();
        break;
      case PLAY_LINE_FOLLOWER :
        //play line follower until another command is received
        while (!bluetooth_Serial.available()) {
          play_line_follower();
        }
        serial_cmd = bluetooth_Serial.read();
        stop_line_follower();
        break;
      case STOP_LINE_FOLLOWER :
        stop_line_follower();
        break;
    }
  }
}

void move_bot_left() {
  digitalWrite(MOTOR_LEFT_1, HIGH);
  digitalWrite(MOTOR_LEFT_2, LOW);
  digitalWrite(MOTOR_RIGHT_1, HIGH);
  digitalWrite(MOTOR_RIGHT_2, LOW);
}

void move_bot_right() {
  digitalWrite(MOTOR_LEFT_1, LOW);
  digitalWrite(MOTOR_LEFT_2, HIGH);
  digitalWrite(MOTOR_RIGHT_1, LOW);
  digitalWrite(MOTOR_RIGHT_2, HIGH);
}

void move_bot_forward() {
  digitalWrite(MOTOR_LEFT_1, LOW);
  digitalWrite(MOTOR_LEFT_2, HIGH);
  digitalWrite(MOTOR_RIGHT_1, HIGH);
  digitalWrite(MOTOR_RIGHT_2, LOW);
}

void move_bot_backward() {
  digitalWrite(MOTOR_LEFT_1, HIGH);
  digitalWrite(MOTOR_LEFT_2, LOW);
  digitalWrite(MOTOR_RIGHT_1, LOW);
  digitalWrite(MOTOR_RIGHT_2, HIGH);
}

void bot_stop() {
  digitalWrite(MOTOR_LEFT_1, LOW);
  digitalWrite(MOTOR_LEFT_2, LOW);
  digitalWrite(MOTOR_RIGHT_1, LOW);
  digitalWrite(MOTOR_RIGHT_2, LOW);
}

void move_motor1_clkwise() {
  digitalWrite(MOTOR_1_1, HIGH);
  digitalWrite(MOTOR_1_2, LOW);
}

void move_motor1_aclkwise() {
  digitalWrite(MOTOR_1_1, LOW);
  digitalWrite(MOTOR_1_2, HIGH);
}

void motor1_stop() {
  digitalWrite(MOTOR_1_1, LOW);
  digitalWrite(MOTOR_1_2, LOW);
}

void move_motor2_clkwise() {
  digitalWrite(MOTOR_2_1, HIGH);
  digitalWrite(MOTOR_2_2, LOW);
}


void move_motor2_aclkwise() {
  digitalWrite(MOTOR_2_1, LOW);
  digitalWrite(MOTOR_2_2, HIGH);
}

void motor2_stop() {
  digitalWrite(MOTOR_2_1, LOW);
  digitalWrite(MOTOR_2_2, LOW);
}

void play_line_follower() {
  int ps1_value = analogRead(PROXIMITY_SENSOR_1);
  ps1_value = map(ps1_value, 0, 1023, 0, 100);
  int ps2_value = analogRead(PROXIMITY_SENSOR_2);
  ps2_value = map(ps2_value, 0, 1023, 0, 100);

  //if black line is detected by both sensors then stop the bot
  if ((ps1_value < ps1_threshold) && (ps2_value < ps2_threshold)) {
    bot_stop();
  }
  //if white line is detected by both sensors then move the bot
  else if ((ps1_value > ps1_threshold) && (ps2_value > ps2_threshold)) {
    move_bot_forward();
  }
  //if black line is detected by left sensor then turn left
  else if ((ps1_value < ps1_threshold)) {
    move_bot_left();
  }
  //if black line is detected by right sensor then turn right
  else {
    move_bot_right();
  }
}

void stop_line_follower() {
  bot_stop();
}
void on(int x) {
  for (int i = 0; i <= 3; i++) {
    digitalWrite(p[i], x);
  }
}
void on2(int x) {
  for (int i = 0; i <= 3; i++) {
    digitalWrite(p[i], x);
    delay(50);
  }
}
void on3(int x) {
  for (int i = 3; i >= 0; i--) {
    digitalWrite(p[i], x);
    delay(50);
  }
}
