#include "pid.h"

using namespace std;

//Setting PID values
void PID::setPID(double dtVal, double maxVal, double minVal, double KpVal, double KdVal, double KiVal){
	//Setting PID values
	dt = dtVal;
	max = maxVal;
	min = minVal;
	Kp = KpVal;
	Kd = KdVal;
	Ki = KiVal;
}

//Calculating output value based on wanted and current values
double PID::calculatePID(double wantedVal, double currVal){

	//Calculating error for current loop
	double error = wantedVal - currVal;

	//Calculation for proportional term output
	double Pout = Kp * error;

	//Calculation for integral term output
	integral += error * dt;
	double Iout = Ki * integral;

	//Calculation for derivative term output
	double derivative = (error - prevError) / dt;
	double Dout = Kd * derivative;

	//Calculating total output
	double output = Pout + Iout + Dout;

	// Restrict output to max/min change values
	if (output > max)
		output = max;
	else if (output < min)
		output = min;

	//Saving error as previous error for next loop
	prevError = error;

	//Return 'output' value, used to alter currVal
	return output;
}