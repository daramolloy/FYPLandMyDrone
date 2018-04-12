#pragma once
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/core/core.hpp>    
#include <tuple>
#include"pid.h"

class Control {
private:
	int wantedThrottle;
	int wantedYaw;
	int wantedPitch;
	int wantedRoll;
	int currentThrottle;
	int currentYaw;
	int currentPitch;
	int currentRoll;
	int hoverValue;
	int normalDescent;
	int lastFiveThrottle[5];


public:
	Control(double maxChange, double pGain, double dGain, double iGain, int hover, int normDesc);
	void setVals(int intT,int intY,int intP,int intR,int currT,int currY,int currP,int currR);
	int *getLandingControls(cv::Point cent, double height);
	int * Control::getHoverControls(cv::Point cent, double height, cv::Point lastCent, double lastHeight);

	PID pidT;
	PID pidP;
	PID pidR;
};

