#include "RazorIMU.h"
#include <iostream>
#include <iomanip> // In order to organize positions of the sensor data that will be printed to console.
#include <conio.h> // for kbhit ("keyboard hit") to exit infinite loop - imported from Gabriel's serial communication C::B project
#define _USE_MATH_DEFINES // for C++
#include <math.h>

using namespace std;

RazorIMU::RazorIMU(char* _portName, const int _baudRate, bool _displayData, bool _writeData)
{
	portName = _portName;
	baudRate = _baudRate;
	device = "Sparkfun Razor IMU";
	user_interaction_to_set_data_format();
	displayData = _displayData;
	writeData = _writeData;
	continueStream = true;
	sensorDataFileName = "sensor_data.txt";
	if (writeData)
	{
		fout.open(sensorDataFileName, ios::out);
		if (!fout.is_open())
		{
			cerr << "Unable to open " << sensorDataFileName << "." << endl;
			system("pause");
			exit(EXIT_FAILURE);
		}
	}
	serialConnection = new Serial(_portName, _baudRate);
	serialConnection->CheckConnection(serialConnection, device, _portName, _baudRate);
	set_razorIMU_data_stream();
	cout << "sizeof(packetTaha) = " << sizeof(packetTaha) << endl;
}

RazorIMU::~RazorIMU()
{
	if (writeData)
		fout.close();
	delete serialConnection;
}

void RazorIMU::user_interaction_to_set_data_format()
{
	char select;
	string intro = "Please enter\n1 (Euler angles)\n2 (calibrated sensor outputs)\n3 (raw and calibrated sensor outputs)\n4 (Packet Taha)\n5 (Packet Fischer)\n";
	cout << intro;
	cin >> select;
	while (select != '1' && select != '2' && select != '3' && select != '4' && select != '5')
	{
		cout << intro;
		cin >> select;
	}
	if (select == '1')
	{
		dataFormatRequestMessage = "#ob";
		dataFormat = EAngles;
	}
	else if (select == '2')
	{
		dataFormatRequestMessage = "#oscb";
		dataFormat = CSOutputs;
	}
	else if (select == '3')
	{
		dataFormatRequestMessage = "#osbb";
		dataFormat = RCSOutputs;
	}
	else if (select == '4')
	{
		dataFormatRequestMessage = "#oA";
		dataFormat = PackageTaha;
	}
	else if (select == '5')
	{
		dataFormatRequestMessage = "#oF";
		dataFormat = PackageFischer;
	}
	else
	{
		cerr << "Invalid data format for Razor IMU!" << endl;
		system("pause");
		exit(EXIT_FAILURE);
	}
}

void RazorIMU::set_razorIMU_data_stream() const
{
	if (serialConnection->WriteData(dataFormatRequestMessage, message_size()))
		cout << "Data format request is sent to Razor IMU successfully" << endl;
}

int RazorIMU::message_size() const
{
	int i = 0;
	while (true)
	{
		if (dataFormatRequestMessage[i] != '\0')
			i++;
		else
			break;
	}
	return i;
}

void RazorIMU::read()
{
	int bytesRead, bytesRead2, bytesRead3;
	if (dataFormat == EAngles)
	{
		bytesRead = serialConnection->ReadData((char*)(&eulerAngles), EULER_PACKET_SIZE_BYTES);
		if (bytesRead == EULER_PACKET_SIZE_BYTES)
		{
			if (displayData)
				display_data_in_console();
			if (writeData)
				write_data_to_file(fout);
		}
	}
	else if (dataFormat == CSOutputs)
	{
		bytesRead = serialConnection->ReadData((char*)(&calibratedSensorOutputs), CALIBRATED_SENSOR_OUTPUTS_PACKET_SIZE_BYTES);
		if (bytesRead == CALIBRATED_SENSOR_OUTPUTS_PACKET_SIZE_BYTES)
		{
			if (displayData)
				display_data_in_console();
			if (writeData)
				write_data_to_file(fout);
		}
	}
	else if (dataFormat == RCSOutputs)
	{
		bytesRead = serialConnection->ReadData((char*)(&rawAndCalibratedSensorOutputs), RAW_AND_CALIBRATED_SENSOR_OUTPUTS_PACKET_SIZE_BYTES);
		if (bytesRead == RAW_AND_CALIBRATED_SENSOR_OUTPUTS_PACKET_SIZE_BYTES)
		{
			if (displayData)
				display_data_in_console();
			if (writeData)
				write_data_to_file(fout);
		}
	}
	else if (dataFormat == PackageTaha)
	{
		char headerTest1 = 'f', headerTest2 = 'f'; // initialize it with any character other than 'h'
		while (headerTest1 != 'h') // whenever 'h' is received, then read next 48 bytes as PacketTaha
		{
			serialConnection->ReadData(&headerTest1, 1);
		}
		while (headerTest2 != 's') // whenever 'h' is received, then read next 48 bytes as PacketTaha
		{
			serialConnection->ReadData(&headerTest2, 1);
		}
		bytesRead = serialConnection->ReadData((char*)(&packetTaha.accMag), PACKET_TAHA-2);
		bytesRead2 = serialConnection->ReadData((char*)(&packetTaha.G_DtFlag), 1);
		bytesRead3 = serialConnection->ReadData((char*)(&packetTaha.packetNumber), 1);

		/* represent accelerometer readings wrt inertial coordinate system. notice that initial reading is wrt body coordinate system. */
		represent_accelerometer_wrt_to_inertial_coordinate_system();

		if (bytesRead + bytesRead2 + bytesRead3 == PACKET_TAHA)
		{
			if (displayData)
				display_data_in_console();
			if (writeData)
				write_data_to_file(fout);
		}
	}
	else if (dataFormat == PackageFischer)
	{
		char headerTest1 = 'f', headerTest2 = 'f'; // initialize it with any character other than 'h'
		while (headerTest1 != 'h') // whenever 'h' is received, then read next 48 bytes as PacketTaha
		{
			serialConnection->ReadData(&headerTest1, 1);
		}
		while (headerTest2 != 's') // whenever 'h' is received, then read next 48 bytes as PacketTaha
		{
			serialConnection->ReadData(&headerTest2, 1);
		}
		bytesRead = serialConnection->ReadData((char*)(&packetFischer.accX), PACKET_FISCHER - 2);
		bytesRead2 = serialConnection->ReadData((char*)(&packetFischer.G_DtFlag), 1);
		bytesRead3 = serialConnection->ReadData((char*)(&packetFischer.packetNumber), 1);

		/* represent accelerometer readings wrt inertial coordinate system. notice that initial reading is wrt body coordinate system. */
		//represent_accelerometer_wrt_to_inertial_coordinate_system();

		if (bytesRead + bytesRead2 + bytesRead3 == PACKET_FISCHER)
		{
			if (displayData)
				display_data_in_console();
			if (writeData)
				write_data_to_file(fout);
		}
	}
	else
	{
		cerr << "There is no data format like this!" << endl;
		system("pause");
		exit(EXIT_FAILURE);
	}
}

void RazorIMU::display_data_in_console() const
{
	if (dataFormat == EAngles)
		cout << fixed << setprecision(2) << setw(6) << right << eulerAngles.roll << setw(8) << eulerAngles.pitch << setw(9) << eulerAngles.yaw << endl;
	else if (dataFormat == CSOutputs)
	{
		cout << fixed << setprecision(2) << setw(7) << right << calibratedSensorOutputs.accX << setw(8) << calibratedSensorOutputs.accY
			 << setw(9) << calibratedSensorOutputs.accZ << endl;
			/*setw(9) << calibratedSensorOutputs.magX
			<< setw(9) << calibratedSensorOutputs.magY << setw(9) << calibratedSensorOutputs.magZ
			<< setw(9) << calibratedSensorOutputs.gyroX << setw(9) << calibratedSensorOutputs.gyroY
			<< setw(9) << calibratedSensorOutputs.gyroZ << endl;*/
	}
	else if (dataFormat == RCSOutputs)
	{
		cout << fixed << setprecision(2) << setw(7) << right << rawAndCalibratedSensorOutputs.caccX << setw(8) << rawAndCalibratedSensorOutputs.caccY
			 << setw(9) << rawAndCalibratedSensorOutputs.caccZ << setw(9) << rawAndCalibratedSensorOutputs.raccX
			 << setw(9) << rawAndCalibratedSensorOutputs.raccY << setw(9) << rawAndCalibratedSensorOutputs.raccZ << endl;
	}
	else if (dataFormat == PackageTaha)
	{
		int packetNumber = packetTaha.packetNumber;
		float G_Dt;
		if (packetTaha.G_DtFlag)
			G_Dt = 0.021;
		else
			G_Dt = 0.020;
		cout << fixed << setprecision(2) << right << setw(9) << packetTaha.accMag << setw(9) << packetTaha.gyroMag
			 << setprecision(3) << setw(8) << right << G_Dt
			 //<< setw(9) << right << packetTaha.accX << setw(8) << packetTaha.accY << setw(9) << packetTaha.accZ
			 //<< setw(9) << right << rawAccWRTinertialFrame[0] << setw(8) << rawAccWRTinertialFrame[1]
			 << setw(5) << packetNumber << setprecision(1) << setw(9) << packetTaha.accX 
			 << setw(9) << packetTaha.accY << endl; //rawAccWRTinertialFrame[2] << endl;
	}
	else if (dataFormat == PackageFischer)
	{
		int packetNumber = packetFischer.packetNumber;
		float G_Dt;
		if (packetFischer.G_DtFlag)
			G_Dt = 0.021;
		else
			G_Dt = 0.020;
		cout << fixed << setprecision(2) << right << setw(9) << packetFischer.accX << setw(9) << packetFischer.accY
			 << setw(9) << packetFischer.gyroX << setw(9) << packetFischer.gyroY << setprecision(3) << setw(8) << right << G_Dt
			//<< setw(9) << right << packetTaha.accX << setw(8) << packetTaha.accY << setw(9) << packetTaha.accZ
			//<< setw(9) << right << rawAccWRTinertialFrame[0] << setw(8) << rawAccWRTinertialFrame[1]
			<< setw(5) << packetNumber << endl; //rawAccWRTinertialFrame[2] << endl;
	}
}

void RazorIMU::write_data_to_file(fstream& fout) const
{
	if (dataFormat == EAngles)
	{
		fout << fixed << setprecision(2) << setw(6) << right << eulerAngles.roll << setw(8) << eulerAngles.pitch << setw(9) << eulerAngles.yaw << endl;
	}
	else if (dataFormat == CSOutputs)
	{
		fout << fixed << setprecision(2) << setw(7) << right << calibratedSensorOutputs.accX << setw(8) << calibratedSensorOutputs.accY
			 << setw(9) << calibratedSensorOutputs.accZ << endl;
		/*setw(9) << calibratedSensorOutputs.magX
		<< setw(9) << calibratedSensorOutputs.magY << setw(9) << calibratedSensorOutputs.magZ
		<< setw(9) << calibratedSensorOutputs.gyroX << setw(9) << calibratedSensorOutputs.gyroY
		<< setw(9) << calibratedSensorOutputs.gyroZ << endl;*/
	}
	else if (dataFormat == RCSOutputs)
	{
		fout << fixed << setprecision(2) << setw(7) << right << rawAndCalibratedSensorOutputs.caccX << setw(8) << rawAndCalibratedSensorOutputs.caccY
			 << setw(9) << rawAndCalibratedSensorOutputs.caccZ << setw(9) << rawAndCalibratedSensorOutputs.raccX
			 << setw(9) << rawAndCalibratedSensorOutputs.raccY << setw(9) << rawAndCalibratedSensorOutputs.raccZ << endl;
	}
	else if (dataFormat == PackageTaha)
	{
		int packetNumber = packetTaha.packetNumber;
		float G_Dt;
		if (packetTaha.G_DtFlag)
			G_Dt = 0.021;
		else
			G_Dt = 0.020;
		fout << fixed << setprecision(2) << right << setw(9) << packetTaha.accMag << setw(9) << packetTaha.gyroMag 
			 << setprecision(3) << setw(8) << right << G_Dt
			 // << setw(9) << right << packetTaha.accX << setw(8) << packetTaha.accY
			 // << setw(9) << packetTaha.accZ << setprecision(3) << setw(10) << packetTaha.G_Dt 
			 // << setw(9) << right << rawAccWRTinertialFrame[0] << setw(9) << rawAccWRTinertialFrame[1]
			 // << setw(10) << rawAccWRTinertialFrame[2] << setw(10) << packetTaha.gyroX << setw(10) << packetTaha.gyroY 
			 // << setw(10) << packetTaha.gyroZ << setw(10) << packetTaha.magX << setw(10) << packetTaha.magY << setw(10) << packetTaha.magZ 
			 << setw(5) << packetNumber << setprecision(1) << setw(9) << packetTaha.accX
			 << setw(9) << packetTaha.accY << endl;
	}
	else if (dataFormat == PackageFischer)
	{
		int packetNumber = packetFischer.packetNumber;
		float G_Dt;
		if (packetFischer.G_DtFlag)
			G_Dt = 0.021;
		else
			G_Dt = 0.020;
		fout << fixed << setprecision(2) << right << setw(9) << packetFischer.accX << setw(9) << packetFischer.accY
			 << setw(9) << packetFischer.accZ << setw(9) << packetFischer.gyroX << setw(9) << packetFischer.gyroY 
			 << setw(9) << packetFischer.gyroZ << setprecision(3) << setw(8) << right << G_Dt
			 // << setw(9) << right << packetTaha.accX << setw(8) << packetTaha.accY
			 // << setw(9) << packetTaha.accZ << setprecision(3) << setw(10) << packetTaha.G_Dt 
			 // << setw(9) << right << rawAccWRTinertialFrame[0] << setw(9) << rawAccWRTinertialFrame[1]
			 // << setw(10) << rawAccWRTinertialFrame[2] << setw(10) << packetTaha.gyroX << setw(10) << packetTaha.gyroY 
			 // << setw(10) << packetTaha.gyroZ << setw(10) << packetTaha.magX << setw(10) << packetTaha.magY << setw(10) << packetTaha.magZ 
			 << setw(5) << packetNumber << endl;
	}
}

bool RazorIMU::wanna_continue_stream()
{
	// Read keyboard for 'q' press to quit
	if (_kbhit())
	{
		char key = _getch();
		if (key == 'q')
			continueStream =  false;
	}
	return continueStream;
}

/* angle2quat() built-in function in MATLAB Aerospace Toolbox */
void RazorIMU::euler_angles_to_quaternion()
{
	/*float t0 = cos(0.5f*packetTaha.yaw);
	float t1 = sin(0.5f*packetTaha.yaw);
	float t2 = cos(0.5f*packetTaha.roll);
	float t3 = sin(0.5f*packetTaha.roll);
	float t4 = cos(0.5f*packetTaha.pitch);
	float t5 = sin(0.5f*packetTaha.pitch);

	quaternion.w = t0 * t2 * t4 + t1 * t3 * t5;
	quaternion.x = t0 * t3 * t4 - t1 * t2 * t5;
	quaternion.y = t0 * t2 * t5 + t1 * t3 * t4;
	quaternion.z = t1 * t2 * t4 - t0 * t3 * t5;*/
}

/* angle2dcm() built-in function in MATLAB Aerospace Toolbox */
void RazorIMU::euler_angles_to_DCM()
{
	/*float cphi = cos(packetTaha.roll), sphi = sin(packetTaha.roll);
	float ctheta = cos(packetTaha.pitch), stheta = sin(packetTaha.pitch);
	float cpsi = cos(packetTaha.yaw), spsi = sin(packetTaha.yaw);
	
	DCM[0][0] = ctheta*cpsi; DCM[0][1] = ctheta*spsi; DCM[0][2] = -stheta;
	DCM[1][0] = -cphi*spsi + sphi*stheta*cpsi; DCM[1][1] = cphi*cpsi + sphi*stheta*spsi; DCM[1][2] = sphi*ctheta;
	DCM[2][0] = sphi*spsi + cphi*stheta*cpsi; DCM[2][1] = -sphi*cpsi + cphi*stheta*spsi; DCM[2][2] = cphi*ctheta;*/
}

void RazorIMU::euler_angles_to_R_body_to_inertial()
{
	/*float cphi = cos(degree_to_radian(packetTaha.roll)), sphi = sin(degree_to_radian(packetTaha.roll));
	float ctheta = cos(degree_to_radian(packetTaha.pitch)), stheta = sin(degree_to_radian(packetTaha.pitch));
	float cpsi = cos(degree_to_radian(packetTaha.yaw)), spsi = sin(degree_to_radian(packetTaha.yaw));

	RbodyToInertial[0][0] = ctheta*cpsi; RbodyToInertial[0][1] = -cphi*spsi + sphi*stheta*cpsi; RbodyToInertial[0][2] = sphi*spsi + cphi*stheta*cpsi;
	RbodyToInertial[1][0] = ctheta*spsi; RbodyToInertial[1][1] = cphi*cpsi + sphi*stheta*spsi; RbodyToInertial[1][2] = -sphi*cpsi + cphi*stheta*spsi;
	RbodyToInertial[2][0] = -stheta; RbodyToInertial[2][1] = sphi*ctheta; RbodyToInertial[2][2] = cphi*ctheta;*/
}

void RazorIMU::represent_accelerometer_wrt_to_inertial_coordinate_system()
{
	euler_angles_to_R_body_to_inertial(); // first compute rotation matrix
	/*rawAccWRTinertialFrame[0] = RbodyToInertial[0][0] * packetTaha.accX + RbodyToInertial[0][1] * packetTaha.accY + RbodyToInertial[0][2] * packetTaha.accZ;
	rawAccWRTinertialFrame[1] = RbodyToInertial[1][0] * packetTaha.accX + RbodyToInertial[1][1] * packetTaha.accY + RbodyToInertial[1][2] * packetTaha.accZ;
	rawAccWRTinertialFrame[2] = RbodyToInertial[2][0] * packetTaha.accX + RbodyToInertial[2][1] * packetTaha.accY + RbodyToInertial[2][2] * packetTaha.accZ;*/
}

float RazorIMU::degree_to_radian(float& deg) const
{
	return M_PI * (deg / 180.f);
}