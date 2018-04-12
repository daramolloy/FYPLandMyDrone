#include <iostream>

#using <System.dll>

public ref class SerialClass {
private:
	//static bool continue; //Only used if reading needed
	static System::IO::Ports::SerialPort^ serialPort;

public:
	SerialClass();
	void Output(System::String^ out);
};