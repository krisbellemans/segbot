#ifndef PID_H
#define PID_H

class PID {
public:
	PID(void);
	PID(float kp, float ki, float kd, float setpoint);
	float update(float feedback);
	void set_kp(float kp);
	void set_ki(float ki);
	void set_kd(float kd);
	void set_params(float kp, float ki, float kd);
	void set_setpoint(float setpoint);

private:
	float kp;
	float ki;
	float kd;

	unsigned long timeref;
	float setpoint;
	float err_sum;
	float last_err;
};

#endif /* PID_H */
