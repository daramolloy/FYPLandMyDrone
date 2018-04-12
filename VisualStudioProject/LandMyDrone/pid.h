#ifndef _PID_H_
#define _PID_H_


class PID{

private:
	double dt;	// Loop interval time
	double max;	// Maximum value by which the current value can change
	double min;	// Minimum value by which the current value can change
	double Kp;	// Proportional gain
	double Kd;	// Derivative gain
	double Ki;	// Integral gain
	double prevError = 0;
	double integral = 0;

public:
	void setPID(double dtVal, double maxVal, double minVal, double KpVal, double KdVal, double KiVal);

	// Returns the manipulated variable given a setpoint and current process value
	double calculatePID(double wantedVal, double currVal);

};

#endif