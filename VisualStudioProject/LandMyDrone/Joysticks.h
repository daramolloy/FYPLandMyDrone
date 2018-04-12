#pragma once
#include <string>

class Joysticks{
public:
	void setJoysticks(int throttle, int yaw, int pitch, int roll, std::string windowName);
};