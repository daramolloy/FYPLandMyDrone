#include <ctime> //For timing loops
#include "Target.h"	//Finds target using OpenCV and height of drone
#include "Joysticks.h"	//Visualises application output
#include "SerialClass.h"	//For outputting control signal serially to com port
#include "Control.h"	//Calculates best control signal for current frame

using namespace cv;
using namespace std;

//For storing the details of the target 
int targetRadius = 0;
Point centerTarget;
int lastTargetRadius = 0;
Point lastCenterTarget;

double height = 0;	//Stores value of the current height of the drone
int motorCutOffHeight = 0; // For deciding when to cut the motors and land

//Wanted Values of throttle, pitch, roll and yaw
//Set to default
int intT = 1000;
int intP = 1500;
int intR = 1500;
int intY = 1500;

//Actual Current values of throttle, pitch, roll and yaw
int currT = 1000;
int currP = 1500;
int currR = 1500;
int currY = 1500;

//PID Values
double maxChange = 400;
double pGain = 1.5;
double dGain = 0.01;
double iGain = 0.1;

//PWM values correlating to throttle hover and descent
//1000 - 0% throttle
//2000 - 100% throttle
int hoverValue = 1600;
int normalDescent = 1560;

//Used when target is lost
int invalidTargets = 0;

//For CSV Testing
int loopCount = 0;
double incrSec = 0.0;

void main() {

	//Opening windows for visualising application
	cvNamedWindow("VideoInput", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("VisualiseCurrent", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("VisualiseDesired", CV_WINDOW_AUTOSIZE);

	//Initialising classes
	VideoCapture cap;
	SerialClass arduino;
	Target target;
	Control cntrl(maxChange, pGain, dGain, iGain, hoverValue, normalDescent);
	Joysticks joySticks;

	//Opening path to source - either video or live view from drone
	cap.open("C:/Users/daram/Desktop/Film/FYP/Video/Realtime/main.avi"); //"C:/Users/daram/Desktop/Film/FYP/Video/30fps/main.mp4");

	//Mat to put the image in
	Mat origImg;

	//cout << "PID Max Change, P-Gain, D-Gain, I-Gain\n";
	//cout << maxChange << "," << pGain << "," << dGain << "," << iGain <<"\n" ;
	
	cout << "LoopCount, LoopTime, Time, Height, Throttle Value, Pitch Value, Roll Value \n";

	//Pause application to  move screens to correct orientation
	waitKey(0);

	//Loop indefinitely
	while (true) {
		clock_t begin = clock();	//Start clock

		cap.read(origImg);		//Read in new frame 

		if (origImg.empty()) {		//If theres no more frames, reload video or break
			cap.open("C:/Users/daram/Desktop/Film/FYP/Video/Realtime/main.avi");
			cap.read(origImg);
			//cout << "Finished.";
			//waitKey(0);
			//break;
		}

		double *tarResults = target.findTarget(origImg);	//finding the target in the frame

		lastTargetRadius = targetRadius;	//Keeping previous target location incase of
		lastCenterTarget = centerTarget;	// invalid new target

		targetRadius = tarResults[0];		//Assigning new target radius & location
		centerTarget.x = tarResults[1];
		centerTarget.y = tarResults[2];

		if (invalidTargets == 5) {	//If no valid target for 5 frames
			targetRadius = 0;		//Set target values to 0 
			centerTarget.x = 0;		//which sets the drone to just hover
			centerTarget.y = 0;
			invalidTargets = 0;		//Reset invalidTargets
		}
		else if(targetRadius == 0 && centerTarget.x == 0 && centerTarget.y == 0) {	//If no valid target is found
			targetRadius = lastTargetRadius;	//Use the previous location of the target for this target
			centerTarget = lastCenterTarget; 
			invalidTargets++;		//Iterate invalidTargets to know if 5 consecutive frames are invalid
		}
		else {
			invalidTargets = 0;		//Reset invalidTargets if target is valid
		}

		//Create circle on the original image of where target is thought to be
		circle(origImg, centerTarget, targetRadius, cv::Scalar(255, 255, 255), 5);

		cv::Point rect1((320- 80), (240-80));
		cv::Point rect2((320 +80),(240+80));
		cv::rectangle(origImg, rect1, rect2, cv::Scalar(255, 255, 255), 1, 1, 0);

		cv::Point rect3((320 - 50), (240 - 50));
		cv::Point rect4((320 + 50), (240 + 50));
		cv::rectangle(origImg, rect3, rect4, cv::Scalar(255, 255, 255), 1, 1, 0);


		imshow("VideoInput", origImg);		//Show original frame with target highlighted

		height = 0;		//Reset height to 0
		height = target.getHeight(targetRadius);	//Get height with target class, returns in metres

		//Defaults for each variable
		System::String^ t = "1000";
		System::String^ y = "1500";
		System::String^ p = "1500";
		System::String^ r = "1500";

		//Get controls values for following and landing using the control class
		cntrl.setVals(intT, intY, intP, intR, currT,currY,currP,currR);
		int *currVals = cntrl.getLandingControls(centerTarget, height);
		//int *currVals = cntrl.getHoverControls(centerTarget, height, lastCenterTarget, target.getHeight(lastTargetRadius));
		intT = currVals[4];
		intP = currVals[5];
		intR = currVals[6];
		intY = currVals[7];		//Assign calculated values back to local variables to output
		currT = currVals[0];
		currP = currVals[1];
		currR = currVals[2];
		currY = currVals[3];


		//Convert int to System string
		t = System::Convert::ToString(currT);
		p = System::Convert::ToString(currP);
		r = System::Convert::ToString(currR);
		y = System::Convert::ToString(currY);

		//Generating the output string and outputting to serial port
		System::String^ out = ",T" + t + ",Y" + y + ",P" + p + ",R" + r + ",";
		arduino.Output(out);		//Outputting through serial class

		//Used for visualising the output of the application
		joySticks.setJoysticks(intT, intY, intP, intR, "VisualiseDesired");
		joySticks.setJoysticks(currT, currY, currP, currR,"VisualiseCurrent");

		//End clock and time loop
		clock_t end = clock();
		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

		if (elapsed_secs < 0.2) {		//If loop happens to be less than 0.2s - mostly 0.34s for PID controller
			cout << "\n                          Wait for Seconds: " << (0.2 - (elapsed_secs));
			waitKey((0.2 - (elapsed_secs))*1000);	//delay to 0.2
		}

		//For accurate frames per second and loop time values
		//end = clock();
		//elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		//cout << "\n                          Loop Time - Seconds: " << elapsed_secs;
		//double fps = 1 / elapsed_secs;
		//cout << "\nFrames Per Second: " << fps;

		//For CSV Testing
		incrSec = incrSec + elapsed_secs;
		loopCount = loopCount + 1;
		if (isinf(height) ) {	//If target is invalid, height is inf
			height = 0;			//If so, set to 0 for csv testing purposes
		}
		cout << loopCount << "," << elapsed_secs << "," << incrSec << "," << height << "," << currT << "," << currP << "," << currR << endl;

		//Needed to display imshow windows
		waitKey(1);
	}
}