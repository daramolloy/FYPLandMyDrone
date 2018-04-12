#include "Control.h"
#include "pid.h"
#include <iostream>

//Set up PID controllers and hover values
Control::Control(double maxChange, double pGain, double dGain, double iGain, int hover, int normDesc){
	//PID Controllers
	pidT.setPID(0.34, maxChange, (maxChange - (2 * maxChange)), 0.1, dGain, iGain);
	pidP.setPID(0.34, maxChange, (maxChange - (2 * maxChange)), pGain, dGain, iGain);
	pidR.setPID(0.34, maxChange, (maxChange - (2 * maxChange)), pGain, dGain, iGain);
	hoverValue = hover;
	normalDescent = normDesc;
	lastFiveThrottle[0] = 0;
	lastFiveThrottle[1] = 0;
	lastFiveThrottle[2] = 0;
	lastFiveThrottle[3] = 0;
	lastFiveThrottle[4] = 0;
}

//Set current wanted and actual values
void Control::setVals(int intT, int intY, int intP, int intR, int currT, int currY, int currP, int currR){
	//Sets the current actual and wanted values for the control algorithm to use
	wantedThrottle = intT;
	wantedYaw = intY;
	wantedPitch = intP;
	wantedRoll = intR;
	currentThrottle = currT;
	currentYaw = currY;
	currentPitch = currP;
	currentRoll = currR;
}

//Calculate output values for landing on moving target
int * Control::getLandingControls(cv::Point cent, double height){

	//Vars for distance to centre target from centre frame
	int xDesired;
	int yDesired;

	//Get x dist from middle screen to target in X-axis
	xDesired = cent.x - 320;

	//if not found, don't move roll
	if (cent.x == 0) {
		wantedRoll = 1500;
	}
	else {
		//Calc Roll value linearly mapped from x dist
		wantedRoll = ((((float)xDesired - (float)(-320)) * ((float)1850 - (float)1150)) / ((float)320 - (float)(-320))) + (float)1150;
		//NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
	}

	//Get y dist from middle screen to target in y-axis
	yDesired = 240 - cent.y;

	//if not found, centre pitch
	if (cent.y == 0) {
		wantedPitch = 1500;
	}
	else {
		//Calculating Pitch Value - Linearly mapping from y dist (pixels) from centre to target
		wantedPitch = ((((float)yDesired - (float)(-240)) * ((float)1850 - (float)1150)) / ((float)240 - (float)(-240))) + (float)1150;
	}

	//Height Control
	//If high up, descend faster
	if (height > 1.5) {
		//If centered, descend at 1400
		if (xDesired > -80 && xDesired < 80 && yDesired > -80 && yDesired < 80) {
			wantedThrottle = normalDescent-40;
		}
		//if not centered, hover
		else {
			wantedThrottle = hoverValue;
		}
	}

	//If lower than 1.5m but higher than 0.4m descend at normal rate
	else if (height > 0.4) {
		if (xDesired > -50 && xDesired < 50 && yDesired > -50 && yDesired < 50) {
			wantedThrottle = normalDescent;
		}
		else {
			wantedThrottle = hoverValue;
		}
	}

	//if we have target but is less than 0.4m, cut motors
	else {
		wantedThrottle = 1000;
	}
	
	bool zeroInArray = false; //Flag for checking if zeros are still in array
	int indexOfZero = 6;
	for (int j = 0; j < 5; j++) {
		if (lastFiveThrottle[j] == 0) {	//If there are 0s in array, set to true
			if (zeroInArray == false) {
				indexOfZero = j;
			}
			zeroInArray = true;
		}
	}

	if (zeroInArray == true) {
		lastFiveThrottle[indexOfZero] = wantedThrottle;
	}
	else {
		for (int k = 0; k < 5; k++) {
			if (k == 4) {
				lastFiveThrottle[k] = wantedThrottle;
			}
			else {
				lastFiveThrottle[k] = lastFiveThrottle[k + 1];
			}
		}
	}

	int cutMotors = 0;
	for (int l = 0; l < 5; l++) {
		if (lastFiveThrottle[l] == 1000) {
			cutMotors++;
		}
	}
	if (cutMotors >= 3) {
		wantedThrottle = 1000;
		lastFiveThrottle[4] = wantedThrottle;
	}

	//std::cout << lastFiveThrottle[0] << " , " << lastFiveThrottle[1] << " , " << lastFiveThrottle[2] << " , " << lastFiveThrottle[3] << " , " << lastFiveThrottle[4] << " \n";

	//cout << "\nTarget X, Y : " << xDesired << " , " << yDesired;
	//cout << "\nHeight : " << height << "\nRadius : " << rad;

	//Calculate throttle using PID and the wanted and current values
	double incT = pidT.calculatePID(wantedThrottle, currentThrottle);
	currentThrottle += incT;

	//Limit value 
	if (currentThrottle < 1030) {
		currentThrottle = 1030;
	}
	if (currentThrottle > 1600) {
		currentThrottle = 1600;
	}
	//cout << "\nDesired Throttle : " << wantedThrottle << ",  Change : " << incT << ",  Output Throttle : " << currentThrottle;

	//Calculate pitch using PID and the wanted and current values
	double incP = pidP.calculatePID(wantedPitch, currentPitch);
	currentPitch += incP;

	//Limit value 
	if (currentPitch < 1030) {
		currentPitch = 1030;
	}
	if (currentPitch > 1970) {
		currentPitch = 1970;
	}
	//cout << "\nDesired Pitch : " << wantedPitch << ",  Change : " << incP << ",  Output Pitch : " << currentPitch;

	//Calculate roll using PID and the wanted and current values
	double incR = pidR.calculatePID(wantedRoll, currentRoll);
	currentRoll += incR;

	//Limit value 
	if (currentRoll < 1030) {
		currentRoll = 1030;
	}
	if (currentRoll > 1970) {
		currentRoll = 1970;
	}
	//cout << "\nDesired Roll : " << wantedRoll << ",  Change : " << incR << ",  Output Roll : " << currentRoll;

	//User wanted yaw as the output yaw as this shouldnt change
	currentYaw = wantedYaw;

	//Limit value 
	if (currentYaw < 1030) {
		currentYaw = 1030;
	}
	if (currentYaw > 1970) {
		currentYaw = 1970;
	}

	//Return an array of calculated values
	int currVals[8] = { currentThrottle , currentPitch, currentRoll, currentYaw, wantedThrottle, wantedPitch,wantedRoll,wantedYaw};
	return currVals;
}

//Calculate output values for hovering
int * Control::getHoverControls(cv::Point cent, double height, cv::Point lastCent, double lastHeight) {

	//if (wantedThrottle == 1000) {
	//	wantedThrottle = hoverValue;
	//}

	//if (height < 0.3 || height > 4 || lastHeight < 0.3 || lastHeight > 4) {
	//	//std::cout << "\nError";
	//}
	//else {
	//	int dif = (lastHeight - height) * 100;
	//	wantedThrottle += dif;
	//	//std::cout << "\nGain:	" << dif << "\nWanted:		" << wantedThrottle;

	////Calculate throttle using PID and the wanted and current values
	//	double incT = pidT.calculatePID(wantedThrottle, currentThrottle);
	//	currentThrottle += incT;
	//}

	////Limit value 
	//if (currentThrottle > 1700) {
	//	currentThrottle = 1700;
	//}
	//else if (currentThrottle < 1500) {
	//	currentThrottle = 1500;
	//}
	currentThrottle = 1600;

	int currVals[8] = { currentThrottle , currentPitch, currentRoll, currentYaw, wantedThrottle, wantedPitch,wantedRoll,wantedYaw };
	return currVals;
}
