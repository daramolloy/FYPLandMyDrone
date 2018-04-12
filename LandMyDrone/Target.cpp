#include "Target.h"
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/core/core.hpp>     

using namespace std;

//Return details of most likely target
double *Target::findTarget(cv::Mat im){
	//Blur the image to reduce the affects of noise
	medianBlur(im, im, 3);

	//Convert to HSV
	cv::Mat hsvImage;
	cvtColor(im, hsvImage, cv::COLOR_BGR2HSV);

	//Isolate red hue pixels
	cv::Mat lowerRedHueRange;
	cv::Mat upperRedHueRange;
	inRange(hsvImage, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lowerRedHueRange);
	inRange(hsvImage, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upperRedHueRange);
	cv::Mat redHueImage;
	addWeighted(lowerRedHueRange, 1.0, upperRedHueRange, 1.0, 0.0, redHueImage);
	//Blur image to smooth pixel isolation
	GaussianBlur(redHueImage, redHueImage, cv::Size(9, 9), 2, 2);

	//Use hough circles to return all red circles
	std::vector<cv::Vec3f> circles;
	HoughCircles(redHueImage, circles, CV_HOUGH_GRADIENT, 1, redHueImage.rows / 8, 100, 20, 23, 160);

	//Create and initialise target radius and location values to 0
	int newTarRadius = 0;
	cv::Point newCenterCircle;
	newCenterCircle.x = 0;
	newCenterCircle.y = 0;

	//If there are more than 20 circles found, it takes too long to process - target is invalid
	if (circles.size() < 20) {
		//For every circle found
		for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
			//Current circle location set to centre
			cv::Point centre(round(circles[current_circle][0]), round(circles[current_circle][1]));
			//Current circle radius to radius
			int radius = round(circles[current_circle][2]);
			//If the radius is within the correct range and the centre is within the frame
			if (radius > 23 && radius < 160 && centre.x > 0 && centre.x < 720 && centre.y > 0 && centre.y < 480) {
				//If this target is bigger than the biggest found
				//(Biggest set to our target as theres often many smaller circles from noise
				// or subcircles of the actual target)
				if (radius > newTarRadius) {
					newTarRadius = radius;
					newCenterCircle = centre;
				}
			}
		}
	}

	//Returning the most likely target as an array
	double arr[3] = { newTarRadius , newCenterCircle.x, newCenterCircle.y };
	return arr;
}

//Get height based on target details
double Target::getHeight(int maxRadius) {
	//Using equation created using known heights vs target radius
	double height = 86.367 * pow(maxRadius, -1.157);
	if (height < 0) {	//If the height goes negative then set to 0
		height = 0;
	}
	return height;
}