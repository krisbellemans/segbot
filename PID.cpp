#include "PID.h"
#include "Arduino.h"

PID::PID(void)
{
	this->set_params(0.0, 0.0, 0.0);
	this->set_setpoint(0.0);
	err_sum = 0.0;
	last_err = 0.0;
	timeref = 0;
}

PID::PID(float kp, float ki, float kd, float setpoint)
{
	this->set_params(kp, ki, kd);
	this->set_setpoint(setpoint);
	this->err_sum = 0.0;
	this->last_err = 0.0;
	this->timeref = 0;
}

void PID::set_kp(float kp)
{
	this->kp = kp;
}

void PID::set_ki(float ki)
{
	this->ki = ki;
}

void PID::set_kd(float kd)
{
	this->kd = kd;
}

void PID::set_params(float kp, float ki, float kd)
{
	this->set_kp(kp);
	this->set_ki(ki);
	this->set_kd(kd);
}

void PID::set_setpoint(float setpoint)
{
	this->setpoint = setpoint;
	this->timeref = millis();
}

float PID::update(float feedback)
{
	float output = 0.0;
	unsigned long now = millis();
	unsigned long time_delta = now - this->timeref;
	float error = setpoint - feedback;
	this->err_sum += (error * (time_delta));
	float err_delta = (error - this->last_err) / time_delta;
	//output = this->kp * error + ki * constrain(this->err_sum, -100, 100) + kd * err_delta;
	output = this->kp * error + this->ki * this->err_sum + this->kd * err_delta;
	this->last_err = error;
	this->timeref = now;

	return constrain(output, -255, 255);
	//return output;
}
