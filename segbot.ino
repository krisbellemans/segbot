// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and BMA150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "BMA150.h"
#include "ITG3200.h"
#include "PID.h"
#include <math.h>

// class default I2C address is 0x38

BMA150 accel;
ITG3200 gyro;
PID pid;

#define LED_PIN 13
#define KP_PIN	0
#define KI_PIN	1
#define KD_PIN	2

/*******************************************************************************
 *			    Motor Driver Pins
 ******************************************************************************/
#define STBY	10  // Standby

// Motor A
#define PWMA	3   // Speed
#define AIN1	9   // Direction
#define AIN2	8   // Direction

// Motor B
#define PWMB	5   // Speed
#define BIN1	11  // Direction
#define BIN2	12  // Direction

#define FORWARD	0
#define REVERSE	1

#define MOTOR_A	0
#define MOTOR_B 1
/******************************************************************************/

#define HIGH_PASS_COEFF	(float)(0.92)
//#define LOW_PASS_COEFF	(float)0.08
#define LOW_PASS_COEFF	(float)(1.00 - HIGH_PASS_COEFF)

bool blinkState = false;
float acc_pitch_angle = 0;
float gyro_pitch_angle = 0;
float filtered_angle = 0;
float timestep = 0.02;
float biasGyroX, biasGyroY, biasGyroZ, biasAccX, biasAccY, biasAccZ;
unsigned long timer;

static void calibrate_sensors(void)
{
/*
	int totalGyroXValues = 0;
	int totalGyroYValues = 0;
	int totalGyroZValues = 0;
	int totalAccelXValues = 0;
	int totalAccelYValues = 0;
	int totalAccelZValues = 0;
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int i;

	delay(100);
	for (i = 0; i < 50; i++) {
		accel.getAcceleration(&ax, &ay, &az);
		gyro.getRotation(&gx, &gy, &gz);
		totalGyroXValues += gx;
		totalGyroYValues += gy;
		totalGyroZValues += gz;
		totalAccelXValues += ax;
		totalAccelYValues += ay;
		totalAccelZValues += az;
		delay(50);
	}
	biasGyroX = totalGyroXValues / 50;
	biasGyroY = totalGyroYValues / 50;
	biasGyroZ = totalGyroZValues / 50;
	biasAccX = totalAccelXValues / 50;
	biasAccY = totalAccelYValues / 50;
	biasAccZ = (totalAccelZValues / 50) - 272;
	Serial.println(biasGyroX);
	Serial.println(biasGyroY);
	Serial.println(biasGyroZ);
	Serial.println(biasAccX);
	Serial.println(biasAccY);
	Serial.println(biasAccZ);
	delay(5000);
*/
	biasGyroX = -28.00;
	biasGyroY = -30.00;
	biasGyroZ = 11.00;
	biasAccX = 5.00;
	biasAccY = -26.00;
	biasAccZ = -4.00;
}

void setup()
{
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();

	// initialize serial communication
	// (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
	// it's really up to you depending on your project)
	Serial.begin(38400);

	// initialize device
	Serial.println("Initializing I2C devices...");
	accel.initialize();
	gyro.initialize();

	// verify connection
	Serial.println("Testing device connections...");
	Serial.println(accel.testConnection()? "BMA150 connection successful" :
		       "BMA150 connection failed");
	Serial.println(gyro.testConnection()? "ITG3200 connection succesful" :
		       "ITG3200 connection failed");

	gyro.setRate(9);
	gyro.setDLPFBandwidth(ITG3200_DLPF_BW_98);
	calibrate_sensors();
	// configure Arduino LED for
	pinMode(LED_PIN, OUTPUT);

	//configure Motor Driver pins
	pinMode(STBY, OUTPUT);
	pinMode(PWMA, OUTPUT);
	pinMode(AIN1, OUTPUT);
	pinMode(AIN2, OUTPUT);

	pinMode(PWMB, OUTPUT);
	pinMode(BIN1, OUTPUT);
	pinMode(BIN2, OUTPUT);
}

static float calculate_acc_angle(void)
{
	int16_t ax, ay, az;
	int32_t ly, lz;
	accel.getAcceleration(&ax, &ay, &az);
	ly = ay - biasAccY;
	lz = az - biasAccZ;
	//return asinf(ly / sqrt((ly * ly) + (lz * lz)));
	return atan2(ly, lz) * (360.0 / (2 * PI));
}

static float calculate_gyro_angle(float feedback)
{
	int16_t gx, gy, gz;
	gyro.getRotation(&gx, &gy, &gz);
	return feedback + ((gx - biasGyroX) / 14.375) * timestep;
}

static float filter_angle(float gyro, float acc)
{
	// complementary filter combining gyro and acc data
	return (HIGH_PASS_COEFF * gyro) + (LOW_PASS_COEFF * acc);
}

static void update_pid_params(void)
{
	float kp = 15.0;
	float ki = 0.005;
	float kd = 3.0;
	//int kp = analogRead(KP_PIN);
	//int ki = analogRead(KI_PIN);
	//int kd = analogRead(KD_PIN);
	pid.set_params(kp, ki, kd);
}

static void motor_drive(int motor, int speed)
{
	int direction = (speed < 0) ? FORWARD : REVERSE;
	digitalWrite(STBY, HIGH);   // wakeup
	if (MOTOR_A == motor) {
		switch (direction) {
		case FORWARD:
			digitalWrite(AIN1, HIGH);
			digitalWrite(AIN2, LOW);
			break;
		case REVERSE:
			digitalWrite(AIN1, LOW);
			digitalWrite(AIN2, HIGH);
			break;
		default:
			return;
		}
		analogWrite(PWMA, abs(speed));
	} else {
		switch (direction) {
		case FORWARD:
			digitalWrite(BIN1, HIGH);
			digitalWrite(BIN2, LOW);
			break;
		case REVERSE:
			digitalWrite(BIN1, LOW);
			digitalWrite(BIN2, HIGH);
			break;
		default:
			return;
		}
		analogWrite(PWMB, abs(speed));
	}
}

void loop()
{
	update_pid_params();
	timer = millis();
	acc_pitch_angle = calculate_acc_angle();
	gyro_pitch_angle = calculate_gyro_angle(filtered_angle);
	filtered_angle = filter_angle(gyro_pitch_angle, acc_pitch_angle);
	float output = pid.update(filtered_angle);
	motor_drive(MOTOR_A, output);
	motor_drive(MOTOR_B, -output);

	Serial.print("acc: ");
	Serial.print(acc_pitch_angle);
	Serial.print(" gyro: ");
	Serial.print(gyro_pitch_angle);
	Serial.print(" filtered: ");
	Serial.print(filtered_angle);
	Serial.print(" speed: ");
	Serial.println(output);

	// blink LED to indicate activity
	blinkState = !blinkState;
	digitalWrite(LED_PIN, blinkState);

	timer = millis() - timer;
	timer = (timestep * 1000) - timer;
	delay(timer);
}
