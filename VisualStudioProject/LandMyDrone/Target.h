#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/core/core.hpp>    
#include <tuple>

class Target {

public:
	double *findTarget(cv::Mat im);
	double getHeight(int maxRadius);
};

