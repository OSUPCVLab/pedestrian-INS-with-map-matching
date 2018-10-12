/*
  File: main.cpp
  Written by: M. Taha Koroglu
  Synopsis: Reading Razor IMU data to C++ application.
*/

#include <iostream>
#include "RazorIMU.h"

using namespace std;

int main()
{
	// User has to enter port number. Baudrate is determined by IMU developer so don't change it. 57600
	RazorIMU* razorIMU = new RazorIMU("COM5", 115200, true, true); // last two arguments: display data in console & write data to file
	while ( razorIMU->wanna_continue_stream() ) // press 'q' to end data stream
	{
		razorIMU->read();
	}
	delete razorIMU;
	return EXIT_SUCCESS;
}