/*
@file modular.cpp Library for Modular kit Coding
@creator www.learningbix.com
@help support@learningbix.com
*/


#include "add/modular.h"
#include "add/NeoSWSerial.h"
#include "add/Servo.h"
Servo servo_motor_one;
Servo servo_motor_two;
Servo servo_motor_three;
Servo servo_motor_four;

NeoSWSerial bluetooth_serial;
DHT dht;
MPU6050 accelgyro;

// Test function
uint8_t pumpBegin(uint8_t cpu_port);
	uint8_t pumpSet(uint8_t cpu_port);
	
uint8_t Modular::pumpBegin(uint8_t cpu_port){
		if((cpu_port <=4) && (cpu_port >=3)){		
		//check if port is already used
		if(_port_use[cpu_port-1][2] != _port_map[cpu_port-1][2]) {
			pinMode(_port_map[cpu_port-1][2], OUTPUT);
			
			//update port use array;
			_port_use[cpu_port-1][2] = _port_map[cpu_port-1][2];
	
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Pump";
	
			//update port use counter
			_port_use_count[cpu_port-1]++;

			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
	
}

//function to set pump on/off
void Modular::pumpSet(uint8_t cpu_port, bool pump_state){
	digitalWrite(_port_map[cpu_port-1][2], pump_state);
}

    uint8_t relayBegin(uint8_t cpu_port);
    void relaySet(uint8_t cpu_port, bool relay_state);
	
	uint8_t Modular::relayBegin(uint8_t cpu_port){
		if((cpu_port <=4) && (cpu_port >=3)){	
		//check if port is already used
		if(_port_use[cpu_port-1][1] != _port_map[cpu_port-1][1]) {
			pinMode(_port_map[cpu_port-1][1], OUTPUT);
			
			//update port use array;
			_port_use[cpu_port-1][1] = _port_map[cpu_port-1][1];
	
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Relay Switch";
	
			//update port use counter
			_port_use_count[cpu_port-1]++;

			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
	
}
void Modular::relaySet(uint8_t cpu_port, bool relay_state){
	digitalWrite(_port_map[cpu_port-1][1], relay_state);
}

uint8_t Modular::switchBegin(uint8_t cpu_port){
	//check if valid port number is passed
	if((cpu_port <=4) && (cpu_port >=3)){
		//declare pin as input
		pinMode(_port_map[cpu_port-1][2], INPUT);
		//check if port or pin is already used
		if(_port_use[cpu_port-1][2] != _port_map[cpu_port-1][2]){
			//update port use array
			_port_use[cpu_port-1][2] = _port_map[cpu_port-1][2];
			
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Push Switch";

			//update port use counter
			_port_use_count[cpu_port-1]++;
			
			return _success;
		}
		else{//return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to read state of switch
//returns 0 (unpressed), 1 (pressed)
bool Modular::switchRead(uint8_t cpu_port){
	return !(digitalRead(_port_map[cpu_port-1][2]));
}
uint8_t Modular::moistBegin(uint8_t cpu_port){
	//check if valid port number is passed
	if((cpu_port <=2) && (cpu_port >=1)){
		//pinMode(_port_map[cpu_port-1][2], INPUT);
		//check if port or pin is already used
		if(_port_use[cpu_port-1][1] != _port_map[cpu_port-1][1]){
			//update port use array
			_port_use[cpu_port-1][1] = _port_map[cpu_port-1][1];
			
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Moist Sensor";

			//update port use counter
			_port_use_count[cpu_port-1]++;
			
			return _success;
		}
		else{//return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to read moist sensor
//returns moist level 0-100%
uint8_t Modular::moistRead(uint8_t cpu_port){
	return map(analogRead(_port_map[cpu_port-1][1]), 0, 1023, 0, 100);
}
uint8_t Modular::lightBegin(uint8_t cpu_port){
	//check if valid port number is passed
	if((cpu_port <=2) && (cpu_port >=1)){
		//pinMode(_port_map[cpu_port-1][2], INPUT);
		//check if port or pin is already used
		if(_port_use[cpu_port-1][0] != _port_map[cpu_port-1][0]){
			//update port use array
			_port_use[cpu_port-1][0] = _port_map[cpu_port-1][0];
			
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Light Sensor";

			//update port use counter
			_port_use_count[cpu_port-1]++;
			
			return _success;
		}
		else{//return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to read light sensor
//returns light intesity 0-100%
uint8_t Modular::lightRead(uint8_t cpu_port){
	return map(analogRead(_port_map[cpu_port-1][0]), 0, 1023, 0, 100);
}

uint8_t Modular::soundBegin(uint8_t cpu_port){
	//check if valid port number is passed
	if(cpu_port == 2){
		//pinMode(_port_map[cpu_port-1][2], INPUT);
		//check if port or pin is already used
		if(_port_use[cpu_port-1][1] != _port_map[cpu_port-1][1]){
			//update port use array
			_port_use[cpu_port-1][1] = _port_map[cpu_port-1][1];
			
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Sound Sensor";

			//update port use counter
			_port_use_count[cpu_port-1]++;
			
			return _success;
		}
		else{//return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to read sound sensor
//returns sound level 0-100%
uint8_t Modular::soundRead(uint8_t cpu_port){
	return map(analogRead(_port_map[cpu_port-1][1]), 0, 1023, 0, 100);
}

uint8_t Modular::dhtBegin(uint8_t cpu_port){
	//check if valid port number is passed
	//if(cpu_port ==4){
	if((cpu_port <=4) && (cpu_port >=3)){
		//pinMode(_port_map[cpu_port-1][2], INPUT);
		//check if port or pin is already used
		if(_port_use[cpu_port-1][3] != _port_map[cpu_port-1][3]){
			//update port use array
			_port_use[cpu_port-1][3] = _port_map[cpu_port-1][3];
			
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "DHT Sensor";

			dht.begin(_port_use[cpu_port-1][3],DHTTYPE);
			
			//update port use c ounter
			_port_use_count[cpu_port-1]++;
			
			return _success;
		}
		else{//return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to read dht sensor
//returns temperature in deg C
float Modular::tempRead(){
	return dht.readTemperature();
}

//function to read dht sensor
//returns humidity in %
float Modular::humidityRead(){
	return dht.readHumidity();
}

uint8_t Modular::motionBegin(uint8_t cpu_port){
	//check if valid port value is passed
	if(cpu_port == 2){
		//check if port is already used
		if((_port_use[cpu_port-1][0] != _port_map[cpu_port-1][0]) && (_port_use[cpu_port-1][1] != _port_map[cpu_port-1][1])) {
						
			#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
			Wire.begin();
			#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
			Fastwire::setup(400, true);
			#endif
    
			accelgyro.initialize();
			
			if (!accelgyro.testConnection()){
				return	_portBusy;
			}
			else;

			//update port use array;
			_port_use[cpu_port-1][0] = _port_map[cpu_port-1][0];
			_port_use[cpu_port-1][1] = _port_map[cpu_port-1][1];
	
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Motion Sensor";

			//update port use counter
			_port_use_count[cpu_port-1]++;
	
			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to read acceleration
//returns int
int Modular::accXRead(){
	accelgyro.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);
	return _ax;
}

//function to read acceleration
//returns int
int Modular::accYRead(){
	accelgyro.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);
	return _ay;
}

//function to read acceleration
//returns int
int Modular::accZRead(){
	accelgyro.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);
	return _az;
}

//function to read rotation
//returns int
int Modular::gyroXRead(){
	accelgyro.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);
	return _gx;
}

//function to read rotation
//returns int
int Modular::gyroYRead(){
	accelgyro.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);
	return _gy;
}

//function to read rotation
//returns int
int Modular::gyroZRead(){
	accelgyro.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);
	return _gz;
}











//function to control board led
uint8_t Modular::boardLed(uint8_t led_num, bool led_state){
	//led number must be between 1 and 6
	if((led_num <=6) && (led_num >=1)){
		pinMode(_led_pins[led_num], OUTPUT);
		digitalWrite(_led_pins[led_num], _led_state_signal[led_state][led_num]);
	}
	else{ //return error code
		return _pinNotValid;
	}
}

//function to begin sonar sensor at required port
uint8_t Modular::sonarBegin(uint8_t cpu_port){
	//check if valid port value is passed
	if((cpu_port <=4) && (cpu_port >=1)){
		//check if port is already used
		if((_port_use[cpu_port-1][0] != _port_map[cpu_port-1][0]) && (_port_use[cpu_port-1][1] != _port_map[cpu_port-1][1])) {
			pinMode(_port_map[cpu_port-1][0], OUTPUT);
			pinMode(_port_map[cpu_port-1][1], INPUT);
			
			//update port use array;
			_port_use[cpu_port-1][0] = _port_map[cpu_port-1][0];
			_port_use[cpu_port-1][1] = _port_map[cpu_port-1][1];
	
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Sonar Sensor";
			
			//update port use counter
			_port_use_count[cpu_port-1]++;
	
			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to read distance from sonar sensor
//return distance in range of 0 to 255 cm
uint8_t Modular::sonarRead(uint8_t cpu_port){
	unsigned long previousMicros;
    unsigned long timeout = 20000UL;
	
	//create trigger pulse
	digitalWrite(_port_map[cpu_port-1][0], LOW);
	delayMicroseconds(2);
	digitalWrite(_port_map[cpu_port-1][0], HIGH);
	delayMicroseconds(10);
	digitalWrite(_port_map[cpu_port-1][0], LOW);
	
	previousMicros = micros();
	while((!digitalRead(_port_map[cpu_port-1][1])) && ((micros() - previousMicros) <= timeout)); // wait for the echo pin HIGH or timeout
	previousMicros = micros();
	while((digitalRead(_port_map[cpu_port-1][1]))  && ((micros() - previousMicros) <= timeout)); // wait for the echo pin LOW or timeout

	return (micros() - previousMicros)/28/2; // duration
}

//function to begin proximity sensor one
uint8_t Modular::proximityOneBegin(uint8_t cpu_port){
	//check if valid port number is passed
	if((cpu_port <=2) && (cpu_port >=1)){
		//pinMode(_port_map[cpu_port-1][2], INPUT);
		//check if port or pin is already used
		if(_port_use[cpu_port-1][2] != _port_map[cpu_port-1][2]){
			//update port use array
			_port_use[cpu_port-1][2] = _port_map[cpu_port-1][2];
			
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Proximity One Sensor";

			//update port use counter
			_port_use_count[cpu_port-1]++;
			
			return _success;
		}
		else{//return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to read proximity one sensor
//returns reflected light intensity 0-100%
uint8_t Modular::proximityOneRead(uint8_t cpu_port){
	return map(analogRead(_port_map[cpu_port-1][2]), 0, 1023, 0, 100);
}


//function to begin proximity sensor two
uint8_t Modular::proximityTwoBegin(uint8_t cpu_port){
	if((cpu_port <=2) && (cpu_port >=1)){
		//pinMode(_port_map[cpu_port-1][2], INPUT);
		//check if port or pin is already used
		if(_port_use[cpu_port-1][3] != _port_map[cpu_port-1][3]){
			//update port use array
			_port_use[cpu_port-1][3] = _port_map[cpu_port-1][3];

			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Proximity Two Sensor";

			//update port use counter
			_port_use_count[cpu_port-1]++;

			return _success;
		}
		else{//return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to read proximity sensor two
//returns reflected light intensity 0-100%
uint8_t Modular::proximityTwoRead(uint8_t cpu_port){
	return map(analogRead(_port_map[cpu_port-1][3]), 0, 1023, 0, 100);
}

uint8_t Modular::steerBotBegin(uint8_t cpu_port){
	if((cpu_port <=4) && (cpu_port >=3)){
		//check if port or pin is already used
		if((_port_use[cpu_port-1][0] != _port_map[cpu_port-1][0]) && (_port_use[cpu_port-1][1] != _port_map[cpu_port-1][1]) && (_port_use[cpu_port-1][2] != _port_map[cpu_port-1][2]) && (_port_use[cpu_port-1][3] != _port_map[cpu_port-1][3])){
			pinMode(_port_map[cpu_port-1][0], OUTPUT);
			pinMode(_port_map[cpu_port-1][1], OUTPUT);
			pinMode(_port_map[cpu_port-1][2], OUTPUT);
			pinMode(_port_map[cpu_port-1][3], OUTPUT);

			//update port use array
			_port_use[cpu_port-1][0] = _port_map[cpu_port-1][0];
			_port_use[cpu_port-1][1] = _port_map[cpu_port-1][1];
			_port_use[cpu_port-1][2] = _port_map[cpu_port-1][2];
			_port_use[cpu_port-1][3] = _port_map[cpu_port-1][3];
			
			
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = " Steer Motor";

			//update port use counter
			_port_use_count[cpu_port-1]++;
						
			return _success;
		}		
		else{//return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to steer bot forward
//parameter speed in range of 0 to 100%
void Modular::steerBotForward(uint8_t cpu_port, int bot_speed){
	//check if speed has been passed at port 3
	if((cpu_port == 3) && (bot_speed != -1)){
		bot_speed = constrain(bot_speed, 0, 100);
		bot_speed = map(bot_speed, 0, 100, 0, 255);
		analogWrite(_port_map[cpu_port-1][0], LOW);
		analogWrite(_port_map[cpu_port-1][1], bot_speed);
		analogWrite(_port_map[cpu_port-1][2], bot_speed);
		analogWrite(_port_map[cpu_port-1][3], LOW);
	}
	else{
		digitalWrite(_port_map[cpu_port-1][0], LOW);
		digitalWrite(_port_map[cpu_port-1][1], HIGH);
		digitalWrite(_port_map[cpu_port-1][2], HIGH);
		digitalWrite(_port_map[cpu_port-1][3], LOW);		
	}
}


//function to steer bot backward
void Modular::steerBotBackward(uint8_t cpu_port, int bot_speed){
	//check if speed has been passed at port 3
	if((cpu_port == 3) && (bot_speed != -1)){
		bot_speed = constrain(bot_speed, 0, 100);
		bot_speed = map(bot_speed, 0, 100, 0, 255);		
		analogWrite(_port_map[cpu_port-1][0], bot_speed);
		analogWrite(_port_map[cpu_port-1][1], LOW);
		analogWrite(_port_map[cpu_port-1][2], LOW);
		analogWrite(_port_map[cpu_port-1][3], bot_speed);
	}
	else{
		digitalWrite(_port_map[cpu_port-1][0], HIGH);
		digitalWrite(_port_map[cpu_port-1][1], LOW);
		digitalWrite(_port_map[cpu_port-1][2], LOW);
		digitalWrite(_port_map[cpu_port-1][3], HIGH);		
	}
}

//function to steer bot left around the centre axis
void Modular::steerBotLeftAxial(uint8_t cpu_port, int bot_speed){
	//check if speed has been passed at port 3
	if((cpu_port == 3) && (bot_speed != -1)){
		bot_speed = constrain(bot_speed, 0, 100);
		bot_speed = map(bot_speed, 0, 100, 0, 255);		
		analogWrite(_port_map[cpu_port-1][0], bot_speed);
		analogWrite(_port_map[cpu_port-1][1], LOW);
		analogWrite(_port_map[cpu_port-1][2], bot_speed);
		analogWrite(_port_map[cpu_port-1][3], LOW);
	}
	else{
		digitalWrite(_port_map[cpu_port-1][0], HIGH);
		digitalWrite(_port_map[cpu_port-1][1], LOW);
		digitalWrite(_port_map[cpu_port-1][2], HIGH);
		digitalWrite(_port_map[cpu_port-1][3], LOW);		
	}
}


//function to steer bot right around the centre axis
void Modular::steerBotRightAxial(uint8_t cpu_port, int bot_speed){
	//check if speed has been passed at port 3
	if((cpu_port == 3) && (bot_speed != -1)){
		bot_speed = constrain(bot_speed, 0, 100);
		bot_speed = map(bot_speed, 0, 100, 0, 255);		
		analogWrite(_port_map[cpu_port-1][0], LOW);
		analogWrite(_port_map[cpu_port-1][1], bot_speed);
		analogWrite(_port_map[cpu_port-1][2], LOW);
		analogWrite(_port_map[cpu_port-1][3], bot_speed);
	}
	else{
		digitalWrite(_port_map[cpu_port-1][0], LOW);
		digitalWrite(_port_map[cpu_port-1][1], HIGH);
		digitalWrite(_port_map[cpu_port-1][2], LOW);
		digitalWrite(_port_map[cpu_port-1][3], HIGH);		
	}
}

//function to steer bot left with curve
void Modular::steerBotLeftCurve(uint8_t cpu_port, int bot_speed){
	//check if speed has been passed at port 3
	if((cpu_port == 3) && (bot_speed != -1)){
		bot_speed = constrain(bot_speed, 0, 100);
		bot_speed = map(bot_speed, 0, 100, 0, 255);		
		analogWrite(_port_map[cpu_port-1][0], LOW);
		analogWrite(_port_map[cpu_port-1][1], LOW);
		analogWrite(_port_map[cpu_port-1][2], bot_speed);
		analogWrite(_port_map[cpu_port-1][3], LOW);
	}
	else{
		digitalWrite(_port_map[cpu_port-1][0], LOW);
		digitalWrite(_port_map[cpu_port-1][1], LOW);
		digitalWrite(_port_map[cpu_port-1][2], HIGH);
		digitalWrite(_port_map[cpu_port-1][3], LOW);		
	}
}


//function to steer bot right with curve
void Modular::steerBotRightCurve(uint8_t cpu_port, int bot_speed){
	//check if speed has been passed at port 3
	if((cpu_port == 3) && (bot_speed != -1)){
		bot_speed = constrain(bot_speed, 0, 100);
		bot_speed = map(bot_speed, 0, 100, 0, 255);		
		analogWrite(_port_map[cpu_port-1][0], LOW);
		analogWrite(_port_map[cpu_port-1][1], bot_speed);
		analogWrite(_port_map[cpu_port-1][2], LOW);
		analogWrite(_port_map[cpu_port-1][3], LOW);
	}
	else{
		digitalWrite(_port_map[cpu_port-1][0], LOW);
		digitalWrite(_port_map[cpu_port-1][1], HIGH);
		digitalWrite(_port_map[cpu_port-1][2], LOW);
		digitalWrite(_port_map[cpu_port-1][3], LOW);		
	}
}

//function to halt bot
void Modular::haltBot(uint8_t cpu_port){
		digitalWrite(_port_map[cpu_port-1][0], LOW);
		digitalWrite(_port_map[cpu_port-1][1], LOW);
		digitalWrite(_port_map[cpu_port-1][2], LOW);
		digitalWrite(_port_map[cpu_port-1][3], LOW);
}

//function to begin motor one
uint8_t Modular::motorOneBegin(uint8_t cpu_port){
		if((cpu_port <=4) && (cpu_port >=1)){
		//check if port is already used
		if((_port_use[cpu_port-1][0] != _port_map[cpu_port-1][0]) && (_port_use[cpu_port-1][1] != _port_map[cpu_port-1][1])) {
			pinMode(_port_map[cpu_port-1][0], OUTPUT);
			pinMode(_port_map[cpu_port-1][1], OUTPUT);
			
			//update port use array;
			_port_use[cpu_port-1][0] = _port_map[cpu_port-1][0];
			_port_use[cpu_port-1][1] = _port_map[cpu_port-1][1];
	
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Motor One";

			//update port use counter
			_port_use_count[cpu_port-1]++;

			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to rotate motor one clockwise
void Modular::motorOneRotateClockwise(uint8_t cpu_port, int motor_speed){
		//check if speed has been passed at port 3 and port 4
	if(((cpu_port == 3) || (cpu_port == 4)) && (motor_speed != -1)){
		motor_speed = constrain(motor_speed, 0, 100);
		motor_speed = map(motor_speed, 0, 100, 0, 255);		
		analogWrite(_port_map[cpu_port-1][0], LOW);
		analogWrite(_port_map[cpu_port-1][1], motor_speed);
	}
	else{
		digitalWrite(_port_map[cpu_port-1][0], LOW);
		digitalWrite(_port_map[cpu_port-1][1], HIGH);
	}
}

//function to rotate motor one antiClockwise
void Modular::motorOneRotateAntiClockwise(uint8_t cpu_port, int motor_speed){
		//check if speed has been passed at port 3
	if(((cpu_port == 3) || (cpu_port == 4)) && (motor_speed != -1)){
		motor_speed = constrain(motor_speed, 0, 100);
		motor_speed = map(motor_speed, 0, 100, 0, 255);		
		analogWrite(_port_map[cpu_port-1][0], motor_speed);
		analogWrite(_port_map[cpu_port-1][1], LOW);
	}
	else{
		digitalWrite(_port_map[cpu_port-1][0], HIGH);
		digitalWrite(_port_map[cpu_port-1][1], LOW);
	}
}

//function to halt motor one
void Modular::motorOneHalt(uint8_t cpu_port){
	digitalWrite(_port_map[cpu_port-1][0], LOW);
	digitalWrite(_port_map[cpu_port-1][1], LOW);
}

//function to begin motor two
uint8_t Modular::motorTwoBegin(uint8_t cpu_port){
		if((cpu_port ==4) || (cpu_port ==1) || (cpu_port ==3)){
		//check if port is already used
		if((_port_use[cpu_port-1][2] != _port_map[cpu_port-1][2]) && (_port_use[cpu_port-1][3] != _port_map[cpu_port-1][3])) {
			pinMode(_port_map[cpu_port-1][2], OUTPUT);
			pinMode(_port_map[cpu_port-1][3], OUTPUT);
			
			//update port use array;
			_port_use[cpu_port-1][2] = _port_map[cpu_port-1][2];
			_port_use[cpu_port-1][3] = _port_map[cpu_port-1][3];

			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Motor Two";
	
			//update port use counter
			_port_use_count[cpu_port-1]++;
	
			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to rotate motor two clockwise
void Modular::motorTwoRotateClockwise(uint8_t cpu_port, int motor_speed){
		//check if speed has been passed at port 3
	if((cpu_port == 3) && (motor_speed != -1)){
		motor_speed = constrain(motor_speed, 0, 100);
		motor_speed = map(motor_speed, 0, 100, 0, 255);		
		analogWrite(_port_map[cpu_port-1][2], LOW);
		analogWrite(_port_map[cpu_port-1][3], motor_speed);
	}
	else{
		digitalWrite(_port_map[cpu_port-1][2], LOW);
		digitalWrite(_port_map[cpu_port-1][3], HIGH);
	}
}

//function to rotate motor two antiClockwise
void Modular::motorTwoRotateAntiClockwise(uint8_t cpu_port, int motor_speed){
		//check if speed has been passed at port 3
	if((cpu_port == 3) && (motor_speed != -1)){
		motor_speed = constrain(motor_speed, 0, 100);
		motor_speed = map(motor_speed, 0, 100, 0, 255);				
		analogWrite(_port_map[cpu_port-1][2], constrain(motor_speed, 0, 255));
		analogWrite(_port_map[cpu_port-1][3], LOW);
	}
	else{
		digitalWrite(_port_map[cpu_port-1][2], HIGH);
		digitalWrite(_port_map[cpu_port-1][3], LOW);
	}
}

//function to halt motor one
void Modular::motorTwoHalt(uint8_t cpu_port){
	digitalWrite(_port_map[cpu_port-1][2], LOW);
	digitalWrite(_port_map[cpu_port-1][3], LOW);
}

//function to begin servo one
uint8_t Modular::servoOneBegin(uint8_t cpu_port){
		if((cpu_port <=4) && (cpu_port >=1)){
		//check if port is already used
		if(_port_use[cpu_port-1][0] != _port_map[cpu_port-1][0]) {
			servo_motor_one.attach(_port_map[cpu_port-1][0]);
			
			//update port use array;
			_port_use[cpu_port-1][0] = _port_map[cpu_port-1][0];
	
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Servo One";
	
			//update port use counter
			_port_use_count[cpu_port-1]++;

			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to rotate servo one
//parameter angle with range 0 to 180;
void Modular::servoOneMove(uint8_t servo_angle){
	servo_motor_one.write(servo_angle);
}

//function to begin servo two
uint8_t Modular::servoTwoBegin(uint8_t cpu_port){
		if((cpu_port <=4) && (cpu_port >=1)){
		//check if port is already used
		if(_port_use[cpu_port-1][1] != _port_map[cpu_port-1][1]) {
			servo_motor_two.attach(_port_map[cpu_port-1][1]);
			
			//update port use array;
			_port_use[cpu_port-1][1] = _port_map[cpu_port-1][1];
	
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Servo Two";
	
			//update port use counter
			_port_use_count[cpu_port-1]++;

			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to rotate servo two
//parameter angle with range 0 to 180;
void Modular::servoTwoMove(uint8_t servo_angle){
	servo_motor_two.write(servo_angle);
}

//function to begin servo three
uint8_t Modular::servoThreeBegin(uint8_t cpu_port){
		if((cpu_port ==4) || (cpu_port ==1) || (cpu_port ==3)){		
		//check if port is already used
		if(_port_use[cpu_port-1][2] != _port_map[cpu_port-1][2]) {
			servo_motor_three.attach(_port_map[cpu_port-1][2]);
			
			//update port use array;
			_port_use[cpu_port-1][2] = _port_map[cpu_port-1][2];
	
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Servo Three";
	
			//update port use counter
			_port_use_count[cpu_port-1]++;

			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to rotate servo three
//parameter angle with range 0 to 180;
void Modular::servoThreeMove(uint8_t servo_angle){
	servo_motor_three.write(servo_angle);
}

//function to begin servo four
uint8_t Modular::servoFourBegin(uint8_t cpu_port){
		if(cpu_port ==4 || cpu_port ==1 || cpu_port ==3){		
		//check if port is already used
		if(_port_use[cpu_port-1][3] != _port_map[cpu_port-1][3]) {
			servo_motor_four.attach(_port_map[cpu_port-1][3]);
			
			//update port use array;
			_port_use[cpu_port-1][3] = _port_map[cpu_port-1][3];
	
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Servo Four";
	
			//update port use counter
			_port_use_count[cpu_port-1]++;

			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to rotate servo four
//parameter angle with range 0 to 180;
void Modular::servoFourMove(uint8_t servo_angle){
	servo_motor_four.write(servo_angle);
}

//function to begin buzzer
uint8_t Modular::buzzerBegin(uint8_t cpu_port){
		if((cpu_port == 4) || (cpu_port == 1) || (cpu_port == 3)){		
		//check if port is already used
		if(_port_use[cpu_port-1][3] != _port_map[cpu_port-1][3]) {
			pinMode(_port_map[cpu_port-1][3], OUTPUT);
			
			//update port use array;
			_port_use[cpu_port-1][3] = _port_map[cpu_port-1][3];
	
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Buzzer";
	
			//update port use counter
			_port_use_count[cpu_port-1]++;

			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
	
}

//function to set buzzer on/off
//buzzer can be set to different tone also
void Modular::buzzerSet(uint8_t cpu_port, bool buzzer_state, uint16_t freq){
	if((buzzer_state == setOn) && (freq!=0)){
		tone(_port_map[cpu_port-1][3], freq);
	}
	else{
		noTone(_port_map[cpu_port-1][3]);
		digitalWrite(_port_map[cpu_port-1][3], buzzer_state);
	}
}

//function to begin rgb led
uint8_t Modular::rgbBegin(uint8_t cpu_port, bool LEDtype){
		_LEDtype = LEDtype;
		if((cpu_port == 4) || (cpu_port == 1) || (cpu_port == 3)){
		//check if port is already used
		if((_port_use[cpu_port-1][0] != _port_map[cpu_port-1][0]) && (_port_use[cpu_port-1][1] != _port_map[cpu_port-1][1]) && (_port_use[cpu_port-1][2] != _port_map[cpu_port-1][2])) {
			
			//declare pins as output
			pinMode(_port_map[cpu_port-1][0], OUTPUT);
			pinMode(_port_map[cpu_port-1][1], OUTPUT);
			pinMode(_port_map[cpu_port-1][2], OUTPUT);
			
			if(_LEDtype == CA){
			//declare pins as output
			digitalWrite(_port_map[cpu_port-1][0], HIGH);
			digitalWrite(_port_map[cpu_port-1][1], HIGH);
			digitalWrite(_port_map[cpu_port-1][2], HIGH);
			}

			//update port use array;
			_port_use[cpu_port-1][0] = _port_map[cpu_port-1][0];
			_port_use[cpu_port-1][1] = _port_map[cpu_port-1][1];
			_port_use[cpu_port-1][2] = _port_map[cpu_port-1][2];

			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "RGB LED";
	
			//update port use counter
			_port_use_count[cpu_port-1]++;
	
			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}	
}

//function to set red colour of RGB LED
//parameter intesity value between 0 to 100%
void Modular::rgbSetRed(uint8_t cpu_port, bool led_state, int led_intensity){
	if((cpu_port == 3) && (led_state == setOn) &&  (led_intensity != -1)){
		if(_LEDtype == CA){
		//constraing value between 0 to 100
		led_intensity = constrain(led_intensity, 0, 100);
		//map values for Common Anode Led
		led_intensity = map(led_intensity, 0, 100, 255, 0);
		analogWrite(_port_map[cpu_port-1][0], led_intensity);
		}
		else{
		//constraing value between 0 to 100
		led_intensity = constrain(led_intensity, 0, 100);
		//map values for Common Anode Led
		led_intensity = map(led_intensity, 0, 100, 0, 255);
		analogWrite(_port_map[cpu_port-1][0], led_intensity);			
		}
	}
	else{
		if(_LEDtype == CA)
			digitalWrite(_port_map[cpu_port-1][0], !led_state);
		else
			digitalWrite(_port_map[cpu_port-1][0], led_state);
	}
}

//function to set blue colour of RGB LED
//parameter intesity value between 0 to 100%
void Modular::rgbSetBlue(uint8_t cpu_port, bool led_state, int led_intensity){
	if((cpu_port == 3) && (led_state == setOn) &&  (led_intensity != -1)){
		if(_LEDtype == CA){
		//constraing value between 0 to 100
		led_intensity = constrain(led_intensity, 0, 100);
		//map values for Common Anode Led
		led_intensity = map(led_intensity, 0, 100, 255, 0);
		analogWrite(_port_map[cpu_port-1][2], led_intensity);
		}
		else{
		//constraing value between 0 to 100
		led_intensity = constrain(led_intensity, 0, 100);
		//map values for Common Anode Led
		led_intensity = map(led_intensity, 0, 100, 0, 255);
		analogWrite(_port_map[cpu_port-1][2], led_intensity);			
		}
	}
	else{
		if(_LEDtype == CA)
			digitalWrite(_port_map[cpu_port-1][2], !led_state);
		else
			digitalWrite(_port_map[cpu_port-1][2], led_state);
	}
}

//function to set green colour of RGB LED
//parameter intesity value between 0 to 100%
void Modular::rgbSetGreen(uint8_t cpu_port, bool led_state, int led_intensity){
	if((cpu_port == 3) && (led_state == setOn) &&  (led_intensity != -1)){
		if(_LEDtype == CA){
		//constraing value between 0 to 100
		led_intensity = constrain(led_intensity, 0, 100);
		//map values for Common Anode Led
		led_intensity = map(led_intensity, 0, 100, 255, 0);
		analogWrite(_port_map[cpu_port-1][1], led_intensity);
		}
		else{
		//constraing value between 0 to 100
		led_intensity = constrain(led_intensity, 0, 100);
		//map values for Common Anode Led
		led_intensity = map(led_intensity, 0, 100, 0, 255);
		analogWrite(_port_map[cpu_port-1][1], led_intensity);			
		}
	}
	else{
		if(_LEDtype == CA)
			digitalWrite(_port_map[cpu_port-1][1], !led_state);
		else
			digitalWrite(_port_map[cpu_port-1][1], led_state);
	}
}

//function to begin bluetooth commmunication
//fixed baud rate of 9600bps
uint8_t Modular::bluetoothBegin(uint8_t cpu_port){
		//check if valid port value is passed
	if(cpu_port <=4 && cpu_port >=1){
		//check if port is already used
		if((_port_use[cpu_port-1][0] != _port_map[cpu_port-1][0]) && (_port_use[cpu_port-1][1] != _port_map[cpu_port-1][1])) {
		bluetooth_serial.begin(_port_map[cpu_port-1][0], _port_map[cpu_port-1][1]); 
			
			//update port use array;
			_port_use[cpu_port-1][0] = _port_map[cpu_port-1][0];
			_port_use[cpu_port-1][1] = _port_map[cpu_port-1][1];
	
			//update name in port use list
			_port_use_array[cpu_port-1][_port_use_count[cpu_port-1]] = "Bluetooth";

			//update port use counter
			_port_use_count[cpu_port-1]++;

			return _success;
		}
		else{ //return error code
			return	_portBusy;
		}
	}
	else{ //return error code
		return _portNotValid;
	}
}

//function to send string via bluetooth
void Modular::bluetoothWrite(uint8_t command){
	bluetooth_serial.write(command);
}

//function to read data from bluetooth
byte Modular::bluetoothRead(){
	return (0xff & bluetooth_serial.read());
}

//function to check if data is available to read
int Modular::bluetoothCheck(){
	return bluetooth_serial.available();
}

void Modular::usbBegin(uint16_t baud_rate){
	Serial.begin(baud_rate);
}

byte Modular::usbRead(){
	return (0xff & Serial.read());
}

void Modular::usbWrite(uint8_t message){
	Serial.write(message);
}

void Modular::usbPrint(String message, int base){
		Serial.print(message);
}

void Modular::usbPrint(int message, int base){
	switch(base)
	{
		case 0 : Serial.print(message);
				 break;
		case DEC : Serial.print(message, DEC);
				 break;
		case HEX : Serial.print(message, HEX);
				 break;
		case OCT : Serial.print(message, OCT);
				 break;
		case BIN : Serial.print(message, BIN);
				 break;
	}
}

void Modular::usbPrint(char message, int base){
	switch(base)
	{
		case 0 : Serial.print(message);
				 break;
		case DEC : Serial.print(message, DEC);
				 break;
		case HEX : Serial.print(message, HEX);
				 break;
		case OCT : Serial.print(message, OCT);
				 break;
		case BIN : Serial.print(message, BIN);
				 break;
	}
}

void Modular::usbPrint(float message, int base){
	switch(base)
	{
		case 0 : Serial.print(message);
				 break;
		case DEC : Serial.print(message, DEC);
				 break;
		case HEX : Serial.print(message, HEX);
				 break;
		case OCT : Serial.print(message, OCT);
				 break;
		case BIN : Serial.print(message, BIN);
				 break;
	}
}

void Modular::usbPrint(byte message, int base){
	switch(base)
	{
		case 0 : Serial.print(message);
				 break;
		case DEC : Serial.print(message, DEC);
				 break;
		case HEX : Serial.print(message, HEX);
				 break;
		case OCT : Serial.print(message, OCT);
				 break;
		case BIN : Serial.print(message, BIN);
				 break;
	}
}

void Modular::usbPrintln(String message, int base){
	Serial.println(message);
}

void Modular::usbPrintln(int message, int base){
	switch(base)
	{
		case 0 : Serial.println(message);
				 break;
		case DEC : Serial.println(message, DEC);
				 break;
		case HEX : Serial.println(message, HEX);
				 break;
		case OCT : Serial.println(message, OCT);
				 break;
		case BIN : Serial.println(message, BIN);
				 break;
	}
}

void Modular::usbPrintln(char message, int base){
	switch(base)
	{
		case 0 : Serial.println(message);
				 break;
		case DEC : Serial.println(message, DEC);
				 break;
		case HEX : Serial.println(message, HEX);
				 break;
		case OCT : Serial.println(message, OCT);
				 break;
		case BIN : Serial.println(message, BIN);
				 break;
	}
}

void Modular::usbPrintln(float message, int base){
	switch(base)
	{
		case 0 : Serial.println(message);
				 break;
		case DEC : Serial.println(message, DEC);
				 break;
		case HEX : Serial.println(message, HEX);
				 break;
		case OCT : Serial.println(message, OCT);
				 break;
		case BIN : Serial.println(message, BIN);
				 break;
	}
}

void Modular::usbPrintln(byte message, int base){
	switch(base)
	{
		case 0 : Serial.println(message);
				 break;
		case DEC : Serial.println(message, DEC);
				 break;
		case HEX : Serial.println(message, HEX);
				 break;
		case OCT : Serial.println(message, OCT);
				 break;
		case BIN : Serial.println(message, BIN);
				 break;
	}
}

int Modular::usbCheck(){
	return Serial.available();
}

//function to clear required port
void Modular::clearPort(uint8_t cpu_port){
	switch (cpu_port-1){
		case 0 :	for(int count=0;count<4;count++){
						if(_port_use_array[cpu_port-1][count] == "Servo One"){
						servo_motor_one.detach();	
						}
						else if(_port_use_array[cpu_port-1][count] == "Servo Two"){
						servo_motor_two.detach();	
						}
						else if(_port_use_array[cpu_port-1][count] == "Servo Three"){
						servo_motor_three.detach();	
						}
						else if(_port_use_array[cpu_port-1][count] == "Servo Four"){
						servo_motor_four.detach();	
						}
						else;
						_port_use_array[cpu_port-1][count] = "";
					}
					_port_use_count[cpu_port-1] = 0;
					pinMode(A0, OUTPUT);
					pinMode(A1, OUTPUT);
					pinMode(A2, OUTPUT);
					pinMode(A3, OUTPUT);
					digitalWrite(A0, LOW);
					digitalWrite(A1, LOW);
					digitalWrite(A2, LOW);
					noTone(A3);
					digitalWrite(A3, LOW);
					break;
		case 1 :	for(int count=0;count<4;count++){
						_port_use[cpu_port-1][count] = 0;
						if(_port_use_array[cpu_port-1][count] == "Servo One"){
						servo_motor_one.detach();	
						}
						else if(_port_use_array[cpu_port-1][count] == "Servo Two"){
						servo_motor_two.detach();	
						}
						else;
						_port_use_array[cpu_port-1][count] = "";
					}
					_port_use_count[cpu_port-1] = 0;
					pinMode(A4, OUTPUT);
					pinMode(A5, OUTPUT);
					digitalWrite(A4, LOW);
					digitalWrite(A5, LOW);
					break;

		case 2 :	for(int count=0;count<4;count++){
						_port_use[cpu_port-1][count] = 0;
						if(_port_use_array[cpu_port-1][count] == "Servo One"){
						servo_motor_one.detach();	
						}
						else if(_port_use_array[cpu_port-1][count] == "Servo Two"){
						servo_motor_two.detach();	
						}
						else if(_port_use_array[cpu_port-1][count] == "Servo Three"){
						servo_motor_three.detach();	
						}
						else if(_port_use_array[cpu_port-1][count] == "Servo Four"){
						servo_motor_four.detach();	
						}
						else;
						_port_use_array[cpu_port-1][count] = "";
					}
					_port_use_count[cpu_port-1] = 0;
					pinMode(3, OUTPUT);
					pinMode(5, OUTPUT);
					pinMode(6, OUTPUT);
					pinMode(9, OUTPUT);
					digitalWrite(3, LOW);
					digitalWrite(5, LOW);
					digitalWrite(6, LOW);
					noTone(9);
					digitalWrite(9, LOW);
					break;	
		case 3 :	for(int count=0;count<4;count++){
						_port_use[cpu_port-1][count] = 0;
						if(_port_use_array[cpu_port-1][count] == "Servo One"){
						servo_motor_one.detach();	
						}
						else if(_port_use_array[cpu_port-1][count] == "Servo Two"){
						servo_motor_two.detach();	
						}
						else if(_port_use_array[cpu_port-1][count] == "Servo Three"){
						servo_motor_three.detach();	
						}
						else if(_port_use_array[cpu_port-1][count] == "Servo Four"){
						servo_motor_four.detach();	
						}
						else;
						_port_use_array[cpu_port-1][count] = "";
					}
					_port_use_count[cpu_port-1] = 0;
					pinMode(10, OUTPUT);
					pinMode(11, OUTPUT);
					pinMode(12, OUTPUT);
					pinMode(13, OUTPUT);
					digitalWrite(10, LOW);
					digitalWrite(11, LOW);
					digitalWrite(12, LOW);
					noTone(13);
					digitalWrite(13, LOW);
					break;
	}
}					