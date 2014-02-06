#include "PID.h"

PID::PID(void)
{
	kp = ki = kd = 0.0;
	setpoint = 0.0;
	err_sum = 0.0;
	last_err = 0.0;
	time_delta = 0;
}

PID::PID(float kp, float ki, float kd)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	setpoint = 0.0;
	err_sum = 0.0;
	last_err = 0.0;
	time_delta = 0;
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

float PID::update(float feedback)
{
	float output = 0.0;
	unsigned long now = millis();
	unsigned long time_delta = now - timeref;
	float error = setpoint - feedback;
	this->err_sum += (error * time_delta);
	float error_delta = (error - this->last_err) / time_delta;

	output = this->kp * error + ki * this->err_sum + kd * error_delta;
	this->last_err = error;
	this->timeref = now;

	return output;
}
