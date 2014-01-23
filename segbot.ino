// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and BMA150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "BMA150.h"
#include "ITG3200.h"
#include <math.h>

// class default I2C address is 0x38

BMA150 accel;
ITG3200 gyro;

#define LED_PIN 13
bool blinkState = false;
float acc_pitch_angle = 0;
float gyro_pitch_angle = 0;
float filtered_angle = 0;
float timestep = 0.02;
float biasGyroX, biasGyroY, biasGyroZ, biasAccX, biasAccY, biasAccZ;
unsigned long timer;

void setup()
{
	int totalGyroXValues = 0;
	int totalGyroYValues = 0;
	int totalGyroZValues = 0;
	int totalAccelXValues = 0;
	int totalAccelYValues = 0;
	int totalAccelZValues = 0;
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int i;
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
	// configure Arduino LED for
	pinMode(LED_PIN, OUTPUT);
}

void loop()
{
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int32_t lx, lz;
	timer = millis();
	// read raw gyro measurements from device
	accel.getAcceleration(&ax, &ay, &az);
	gyro.getRotation(&gx, &gy, &gz);
	lx = ax - biasAccX;
	lz = az - biasAccZ;

	//acc_pitch_angle = asinf(lx / sqrt((lx * lx) + (lz * lz)));
	acc_pitch_angle = atan2(lx, lz) * (360.0 / (2 * PI));
	gyro_pitch_angle = filtered_angle + ((gy - biasGyroY) / 14.375) * timestep;

	// complementary filter combining gyro and acc data
	filtered_angle = (0.98 * gyro_pitch_angle) + (0.02 * acc_pitch_angle);

	Serial.print("acc: ");
	Serial.print(acc_pitch_angle);
	Serial.print("\t");
	Serial.print("gyro: ");
	Serial.print(-gyro_pitch_angle);
	Serial.print("\t");
	Serial.print("filtered: ");
	Serial.println(filtered_angle);

	// blink LED to indicate activity
	blinkState = !blinkState;
	digitalWrite(LED_PIN, blinkState);

	timer = millis() - timer;
	timer = (timestep * 1000) - timer;
	delay(timer);
}
