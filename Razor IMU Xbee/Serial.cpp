#include "Serial.h"

// counstructor for Serial class
Serial::Serial(char* portName, int baudRate)
{
	// We're not yet connected
	this->connected = false;

	// Try to connect to the given port through CreateFile
	this->hSerial = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	// Check if the connection was successfull
	if (this->hSerial == INVALID_HANDLE_VALUE)
	{
		// If not success full display an Error
		if (GetLastError() == ERROR_FILE_NOT_FOUND)
			printf("ERROR: Handle was not attached. Reason: %s not available.\n", portName); // Print Error if neccessary
		else
			printf("ERROR!!!");
	}
	else
	{
		// If connected we try to set the comm parameters
		DCB dcbSerialParams = { 0 }; // DCB is a structure that defines the control setting for a serial communications device.
									 // Try to get the current
		if (!GetCommState(this->hSerial, &dcbSerialParams))
			printf("failed to get current serial parameters!"); // If impossible, show an error
		else
		{
			// Define serial connection parameters for the arduino board (https://msdn.microsoft.com/en-us/library/windows/desktop/aa363214%28v=vs.85%29.aspx)
			if (baudRate == 115200)
				dcbSerialParams.BaudRate = CBR_115200; // CBR_9600 is default. CBR stands for computer baud rate. CBR_115200
			else if (baudRate == 57600)
				dcbSerialParams.BaudRate = CBR_57600;
			else if (baudRate == 38400)
				dcbSerialParams.BaudRate = CBR_38400;
			else if (baudRate == 19200)
				dcbSerialParams.BaudRate = CBR_19200;
			else if (baudRate == 9600)// default case
				dcbSerialParams.BaudRate = CBR_9600;
			else
			{
				fprintf(stderr, "There is no option as baudrate = %d", baudRate);
				system("pause");
				exit(EXIT_FAILURE);
			}
			// Note that Arduino command: Serial.begin(speed,config) has following configuration as default.
			// An optional second argument configures the data, parity, and stop bits. The default is 8 data bits, no parity, one stop bit. 
			dcbSerialParams.ByteSize = 8;
			dcbSerialParams.StopBits = ONESTOPBIT;
			dcbSerialParams.Parity = NOPARITY;
			// Setting the DTR to Control_Enable ensures that the Arduino is properly reset upon establishing a connection.
			dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

			// Set the parameters and check for their proper application.
			if (!SetCommState(hSerial, &dcbSerialParams))
				printf("ALERT: Could not set Serial Port parameters");
			else
			{
				this->connected = true; // If everything went fine we're connected 
				PurgeComm(this->hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR); // Flush any remaining characters in the buffers
				Sleep(ARDUINO_WAIT_TIME); // We wait 2s as the arduino board will be reseting
			}
		}
	}
}

Serial::~Serial()
{
	if (this->connected) // Check if we are connected before trying to disconnect
	{
		this->connected = false; // We're no longer connected
		CloseHandle(this->hSerial); // Close the serial handler
	}
}

int Serial::ReadData(char* buffer, unsigned int nbChar)
{
	DWORD bytesRead; // Number of bytes we'll have read
	unsigned int toRead; // Number of bytes we'll really ask to read

						 // Use the ClearCommError function to get status info on the Serial port
	ClearCommError(this->hSerial, &this->errors, &this->status);

	// Check if there is something to read
	if (this->status.cbInQue>0)
	{
		// If there is we check if there is enough data to read the required number
		// of characters, if not we'll read only the available characters to prevent
		// locking of the application.
		if (this->status.cbInQue>nbChar)
			toRead = nbChar;
		else
			toRead = this->status.cbInQue;

		// Try to read the require number of chars, and return the number of read bytes on success
		if (ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL) && bytesRead != 0)
			return bytesRead;
	}
	return -1; // If nothing has been read, or that an error was detected return -1
}


bool Serial::WriteData(char* buffer, unsigned int nbChar)
{
	DWORD bytesSend;
	// Try to write the buffer on the Serial port
	if (!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
	{
		// In case it don't work get comm error and return false
		ClearCommError(this->hSerial, &this->errors, &this->status);
		return false;
	}
	else
		return true;
}

bool Serial::IsConnected()
{
	return this->connected; // Simply return the connection status
}

void Serial::CheckConnection(Serial* connection, const char* connectedDevice, char* portName, int baudRate) const
{
	if (connection->IsConnected())
		printf("%s and computer are connected through %s and baud rate is %d.\n", connectedDevice, portName, baudRate);
	else
	{
		fprintf(stderr, "There is a problem in initialization of serial port connection between %s and laptop!\n", connectedDevice);
		system("pause");
		exit(EXIT_FAILURE);
	}
}