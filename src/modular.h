/*
  @file modular.h - Library for Coding with Modular Kits
  @creator LearningBix - www.learningbix.com
  @help support@learningbix.com
*/

#ifndef MODULAR_H
#define MODULAR_H

#define CA 0
#define CC 1

#include <Arduino.h>
#include "add/DHT.h"
#include "add/I2Cdev.h"
#include "add/MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define DHTTYPE DHT11
#define newLine 1
#define setOn 1
#define setOff 0

class Modular {
  public:
    uint8_t boardLed(uint8_t led_num, bool led_state);
    uint8_t sonarBegin(uint8_t cpu_port);
    uint8_t sonarRead(uint8_t cpu_port);
    uint8_t proximityOneBegin(uint8_t cpu_port);
    uint8_t proximityOneRead(uint8_t cpu_port);
    uint8_t proximityTwoBegin(uint8_t cpu_port);
    uint8_t proximityTwoRead(uint8_t cpu_port);
    uint8_t steerBotBegin(uint8_t cpu_port);
    void steerBotForward(uint8_t cpu_port, int bot_speed = -1);
    void steerBotBackward(uint8_t cpu_port, int bot_speed = -1);
    void steerBotLeftAxial(uint8_t cpu_port, int bot_speed = -1);
    void steerBotRightAxial(uint8_t cpu_port, int bot_speed = -1);
    void steerBotLeftCurve(uint8_t cpu_port, int bot_speed = -1);
    void steerBotRightCurve(uint8_t cpu_port, int bot_speed = -1);
    void haltBot(uint8_t cpu_port);
    uint8_t motorOneBegin(uint8_t cpu_port);
    void motorOneRotateClockwise(uint8_t cpu_port, int motor_speed = -1);
    void motorOneRotateAntiClockwise(uint8_t cpu_port, int motor_speed = -1);
    void motorOneHalt(uint8_t cpu_port);
    uint8_t motorTwoBegin(uint8_t cpu_port);
    void motorTwoRotateClockwise(uint8_t cpu_port, int motor_speed = -1);
    void motorTwoRotateAntiClockwise(uint8_t cpu_port, int motor_speed = -1);
    void motorTwoHalt(uint8_t cpu_port);
    uint8_t servoOneBegin(uint8_t cpu_port);
    void servoOneMove(uint8_t servo_angle);
    uint8_t servoTwoBegin(uint8_t cpu_port);
    void servoTwoMove(uint8_t servo_angle);
    uint8_t servoThreeBegin(uint8_t cpu_port);
    void servoThreeMove(uint8_t servo_angle);
    uint8_t servoFourBegin(uint8_t cpu_port);
    void servoFourMove(uint8_t servo_angle);
    uint8_t buzzerBegin(uint8_t cpu_port);
    void buzzerSet(uint8_t cpu_port, bool buzzer_state, uint16_t freq = 0);
    uint8_t rgbBegin(uint8_t cpu_port, bool _LEDtype);
    void rgbSetRed(uint8_t cpu_port, bool led_state, int led_intensity = -1);
    void rgbSetBlue(uint8_t cpu_port, bool led_state, int led_intensity = -1);
    void rgbSetGreen(uint8_t cpu_port, bool led_state, int led_intensity = -1);
    uint8_t bluetoothBegin(uint8_t cpu_port);
    void bluetoothWrite(uint8_t command);
    byte bluetoothRead();
    int bluetoothCheck();
	void clearPort(uint8_t cpu_port);
    void usbBegin(uint16_t baud_rate = 9600);
    byte usbRead();
    void usbWrite(uint8_t message);
    void usbPrint(String message, int base = 0);
	void usbPrint(int message, int base = 0);
	void usbPrint(char message, int base = 0);
	void usbPrint(float message, int base = 0);
	void usbPrint(byte message, int base = 0);
	void usbPrintln(String message, int base = 0);
	void usbPrintln(int message, int base = 0);
	void usbPrintln(char message, int base = 0);
	void usbPrintln(float message, int base = 0);
	void usbPrintln(byte message, int base = 0);
    int usbCheck();
    void pumpSet(uint8_t cpu_port, bool pump_state);
    uint8_t pumpBegin(uint8_t cpu_port);
	    uint8_t relayBegin(uint8_t cpu_port);
    void relaySet(uint8_t cpu_port, bool relay_state);
		uint8_t switchBegin(uint8_t cpu_port);
	bool switchRead(uint8_t cpu_port);
		uint8_t moistBegin(uint8_t cpu_port);
	uint8_t moistRead(uint8_t cpu_port);
uint8_t lightBegin(uint8_t cpu_port);
	uint8_t lightRead(uint8_t cpu_port);
		uint8_t soundBegin(uint8_t cpu_port);
	uint8_t soundRead(uint8_t cpu_port);
		uint8_t dhtBegin(uint8_t cpu_port);
	float tempRead();
	float humidityRead();
	uint8_t motionBegin(uint8_t cpu_port);
	int accXRead();
	int accYRead();
	int accZRead();
	int gyroXRead();
	int gyroYRead();
	int gyroZRead();


  private:

    const uint8_t _success = 0;
    const uint8_t _pinNotValid = 1;
    const uint8_t _portNotValid = 2;
    const uint8_t _portBusy = 3;
	
	bool _LEDtype = CA;

	int16_t _ax;
	int16_t _ay;
	int16_t _az;
	int16_t _gx;
	int16_t _gy;
	int16_t _gz;

    String _port_use_array[4][4];
    uint8_t _port_use_count[4] = {0};

    uint8_t _led_pins[6] = {0, 1, 2, 4, 7, 8};
    uint8_t _led_state_signal[2][6] = {HIGH, HIGH, LOW, LOW, LOW, LOW,
                                       LOW, LOW, HIGH, HIGH, HIGH, HIGH
                                      };

    uint8_t _port_map[4][4] = {A0, A1, A2, A3,
                               A4, A5, A6, A7,
                               3, 5, 6, 9,
                               10, 11, 12, 13
                              };

    uint8_t _port_use[4][4] = {0, 0, 0, 0,
                               0, 0, 0, 0,
                               0, 0, 0, 0,
                               0, 0, 0, 0
                              };
};
#endif
