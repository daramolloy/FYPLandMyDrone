#include "Joysticks.h"   
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//Visualising application output
void Joysticks::setJoysticks(int throttle, int yaw, int pitch, int roll, std::string windowName)
{
	//Creating a black image 
	cv::Mat joystickImage(300, 600, CV_8UC3, cv::Scalar(0, 0, 0));;

	//Creating a circle and rectangle for the left joystick
	cv::Point centre1(150, 150);
	cv::Point rect1(50, 50);
	cv::Point rect2(250, 250);
	cv::circle(joystickImage, centre1, 100, cv::Scalar(255, 255, 255), 5);
	cv::rectangle(joystickImage, rect1, rect2, cv::Scalar(255, 255, 255), 8, 8, 0);

	//Creating a circle and rectangle for the right joystick
	cv::Point centre2(450, 150);
	cv::Point rect3(350, 50);
	cv::Point rect4(550, 250);
	cv::circle(joystickImage, centre2, 100, cv::Scalar(255, 255, 255), 5);
	cv::rectangle(joystickImage, rect3, rect4, cv::Scalar(255, 255, 255), 8, 8, 0);

	//Calculating the throttle and yaw co-ordinates for the left stick
	float centreTPoint = ((((float)throttle - (float)1000) * ((float)50 - (float)250)) / ((float)2000 - (float)1000)) + (float)250;
	float centreYPoint = ((((float)yaw - (float)1000) * ((float)50 - (float)250)) / ((float)2000 - (float)1000)) + (float)250;

	//Calculating the pitch and roll co-ordinates for the right stick
	float centrePPoint = ((((float)pitch - (float)1000) * ((float)50 - (float)250)) / ((float)2000 - (float)1000)) + (float)250;
	float centreRPoint = ((((float)roll - (float)1000) * ((float)250 - (float)50)) / ((float)2000 - (float)1000)) + (float)50;
	//Calculations based on this equation for linear mapping
	//NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin

	//Creating point and adding to joystickImage
	cv::Point centre3(centreYPoint, centreTPoint);
	circle(joystickImage, centre3, 5, cv::Scalar(0, 255, 255), 10);
	cv::Point centre4(centreRPoint + 300, centrePPoint);
	circle(joystickImage, centre4, 5, cv::Scalar(0, 255, 255), 10);

	//Show joystickImage 
	cv::imshow(windowName, joystickImage);
}
