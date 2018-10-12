/*
  File: main.cpp
  Written by: M. Taha Koroglu
  Synopsis: Reading Razor IMU data to C++ application through Xbee (wireless).
*/

#include <iostream>
#include "RazorIMU.h"

using namespace std;

int main()
{
	// User has to enter port number. Baudrate is determined by IMU developer so don't change it.
	RazorIMU* razorIMUXbee = new RazorIMU("COM7", 57600, true, true); // last two arguments: display data in console & write data to file
	while (razorIMUXbee->wanna_continue_stream()) // press 'q' to end data stream
	{
		razorIMUXbee->read();
	}

	delete razorIMUXbee;
	return EXIT_SUCCESS;
}