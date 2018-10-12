#pragma once
/* Note: Written by "wedge" in Arduino forums. I added CheckConnection() member function myself. */
#include <Windows.h>
#include <stdio.h>
#include <stdlib.h>
#define ARDUINO_WAIT_TIME 2000 // this is for Arduino resetting itself when serial communication is first achieved.

class Serial
{
private:
	HANDLE hSerial; // Serial comm handler
	bool connected; // Connection status
	COMSTAT status; // Get various information about the connection
	DWORD errors; // Keep track of last error
public:
	// check ARDUINO IDE - Tools - Port to see the port number that Arduino is connected to.
	Serial(char* portName, int baudRate); // Initialize Serial communication with the given COM port and baud rate
	~Serial(); // Close the connection
	/* Read data in a buffer, if nbChar is greater than the maximum number of bytes available, it will return only the bytes available. 
	The function return -1 when nothing could be read, the number of bytes actually read. */
	int ReadData(char* buffer, unsigned int nbChar);
	bool WriteData(char* buffer, unsigned int nbChar); // Writes data from a buffer through the Serial connection. Returns true on success.
	bool IsConnected(); // Check if we are actually connected
	void CheckConnection(Serial* connection, const char* connectedDevice, char* portName, int baudRate) const; // Check connection and respond to positive and negative cases
};