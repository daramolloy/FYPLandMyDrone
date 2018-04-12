#include "SerialClass.h"
#using <System.dll>

using namespace System;
using namespace System::IO::Ports;
using namespace System::Threading;

//Setting up serial comunication with Arduino 
SerialClass::SerialClass() {
	//Setting appropriate properties for serialPort
	serialPort = gcnew System::IO::Ports::SerialPort();
	serialPort->PortName = "COM6";
	serialPort->BaudRate = 9600;
	serialPort->Parity = (Parity)Enum::Parse(Parity::typeid, "None");
	serialPort->DataBits = 8;
	serialPort->StopBits = (StopBits)Enum::Parse(StopBits::typeid, "One");
	serialPort->Handshake = (Handshake)Enum::Parse(Handshake::typeid, "None");
	serialPort->ReadTimeout = 500;
	serialPort->WriteTimeout = 500;
}

//Output string "out" to Arduino
void SerialClass::Output(System::String^ out){
													//StringComparer^ stringComparer = StringComparer::OrdinalIgnoreCase;
	//Thread^ readThread = gcnew Thread(gcnew ThreadStart(SerialClass::Read)); //Only used if reading needed
	serialPort->Open();
	//continue = true; //Only used if reading needed
	//readThread->Start();
	serialPort->WriteLine(System::String::Format("{0}", out));	//Write message to COM port
	//readThread->Join(); //Only used if reading needed
	serialPort->Close();
}

//If reading is neccessary
//static void Read() //Only used if reading needed
//{
//	while (continue)
//	{
//		try
//		{
//			System::String^ message = serialPort->ReadLine();
//			Console::WriteLine(message);
//		}
//		catch (TimeoutException ^) {}
//	}
//}
