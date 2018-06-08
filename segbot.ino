// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and BMA150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "BMA150.h"
#include "ITG3200.h"
//#include "PID.h"
#include <PID_v1.h>
#include <math.h>

// class default I2C address is 0x38

BMA150 accel;
ITG3200 gyro;
//PID pid;

#define KP_PIN	1
#define KI_PIN	2
#define KD_PIN	3
#define SP_PIN	0
#define LED_PIN	13

#define SETPOINT    (float)(89.9)

/*******************************************************************************
 *			    Motor Driver Pins
 ******************************************************************************/
#define STBY	10		// Standby

// Motor A
#define PWMA	3		// Speed
#define AIN1	9		// Direction
#define AIN2	8		// Direction

// Motor B
#define PWMB	5		// Speed
#define BIN1	11		// Direction
#define BIN2	12		// Direction

#define FORWARD	0
#define REVERSE	1

#define MOTOR_A	0
#define MOTOR_B 1
/******************************************************************************/

#define HIGH_PASS_COEFF	(float)(0.98)
#define LOW_PASS_COEFF	(float)(1.00 - HIGH_PASS_COEFF)

float acc_pitch_angle = 0;
float gyro_pitch_angle = 0;
double filtered_angle = 90.0;
double setpoint = SETPOINT;
double output = 0.0;
float timestep = 0.02;
float biasGyroX, biasGyroY, biasGyroZ, biasAccX, biasAccY, biasAccZ;
unsigned long timer;

PID pid(&filtered_angle, &output, &setpoint, 1.0, 0.0, 0.0, DIRECT);

static void calibrate_sensors(void)
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

	delay(2000);
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
	//Serial.println(biasGyroX);
	//Serial.println(biasGyroY);
	//Serial.println(biasGyroZ);
	//Serial.println(biasAccX);
	//Serial.println(biasAccY);
	//Serial.println(biasAccZ);
	delay(5000);
	biasAccX = 0.0;
	biasAccY = 0.0;
	biasAccZ = 0.0;
}

void setup()
{
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();

	// initialize serial communication
	// (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
	// it's really up to you depending on your project)
	Serial.begin(115200);

	// initialize device
	//Serial.println("Initializing I2C devices...");
	accel.initialize();
	gyro.initialize();


	// verify connection
	//Serial.println("Testing device connections...");
	//Serial.println(accel.testConnection()? "BMA150 connection successful" :
		       //"BMA150 connection failed");
	//Serial.println(gyro.testConnection()? "ITG3200 connection succesful" :
		       //"ITG3200 connection failed");

	gyro.setRate(9);
	gyro.setDLPFBandwidth(ITG3200_DLPF_BW_98);
	calibrate_sensors();

	//configure Motor Driver pins
	pinMode(STBY, OUTPUT);
	pinMode(PWMA, OUTPUT);
	pinMode(AIN1, OUTPUT);
	pinMode(AIN2, OUTPUT);

	pinMode(PWMB, OUTPUT);
	pinMode(BIN1, OUTPUT);
	pinMode(BIN2, OUTPUT);

	pinMode(LED_PIN, OUTPUT);

	//pid.set_setpoint(SETPOINT);
	pid.SetOutputLimits(-255, 255);
	pid.SetMode(AUTOMATIC);
	pid.SetSampleTime(timestep * 1000);
	digitalWrite(STBY, HIGH);	// wakeup
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
	return feedback + ((gx - biasGyroX) / 14.375) * timestep; // take gyro sensitivity into account
}

static float filter_angle(float gyro, float acc)
{
	// complementary filter combining gyro and acc data
	return (HIGH_PASS_COEFF * gyro) + (LOW_PASS_COEFF * acc);
}

static void update_pid_params(float feedback, double *sp)
{
	float kp = (analogRead(KP_PIN)/1023.0) * 100;
	float ki = (analogRead(KI_PIN)/1023.0) * 50;
	float kd = (analogRead(KD_PIN)/1023.0) * 50;
	*sp = 85 + (analogRead(SP_PIN)/102.3);
	Serial.print("kp: ");Serial.print(kp);Serial.print(" ");
	Serial.print("ki: ");Serial.print(ki);Serial.print(" ");
	Serial.print("kd: ");Serial.print(kd);Serial.print(" ");
	Serial.print("sp: ");Serial.print(*sp);Serial.print(" ");
	//pid.set_params(kp, ki, kd);
	pid.SetTunings(kp, ki, kd);
	if (feedback < (*sp + 0.25) && feedback > (*sp - 0.25))
		digitalWrite(LED_PIN, 1);
	else
		digitalWrite(LED_PIN, 0);
}

static void motor_drive(int speed)
{
	int direction = (speed < 0) ? FORWARD : REVERSE;
	switch (direction) {
	case FORWARD:
		digitalWrite(AIN1, LOW);
		digitalWrite(AIN2, HIGH);
		digitalWrite(BIN1, HIGH);
		digitalWrite(BIN2, LOW);
		break;
	case REVERSE:
		digitalWrite(AIN1, HIGH);
		digitalWrite(AIN2, LOW);
		digitalWrite(BIN1, LOW);
		digitalWrite(BIN2, HIGH);
		break;
	default:
		return;
	}
	analogWrite(PWMA, abs(speed));
	analogWrite(PWMB, abs(speed));
}

void loop()
{
	update_pid_params(filtered_angle, &setpoint);
	timer = millis();
	acc_pitch_angle = calculate_acc_angle();
	gyro_pitch_angle = calculate_gyro_angle(filtered_angle);
	filtered_angle = filter_angle(gyro_pitch_angle, acc_pitch_angle);
	//float output = pid.update(filtered_angle);

	pid.Compute();

	if (filtered_angle > 20  && filtered_angle < 160)
		motor_drive(output);
	else
		motor_drive(0);

	Serial.print("time: ");
	Serial.print(timer);
	Serial.print(" angle: ");
	Serial.print(filtered_angle);
	Serial.print(" speed: ");
	Serial.println(output);


	timer = millis() - timer;
	Serial.print("timediff: ");
	Serial.print(timer);
	timer = (timestep * 1000) - timer;
	Serial.print("timestep: ");
	Serial.print(timer);
	//delay(timer);
}
