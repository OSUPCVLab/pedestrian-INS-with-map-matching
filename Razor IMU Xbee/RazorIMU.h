#pragma once
#include <string>
#include "Serial.h" // serial library written by wedge (Arduino forums)
#include <fstream>
#define SIZE_FLOAT_ARDUINO 4 /* visit https://www.arduino.cc/en/Reference/Float to see a float variable allocates 4 bytes in Atmega328P. */
#define EULER_PACKET_SIZE_BYTES 12
#define CALIBRATED_SENSOR_OUTPUTS_PACKET_SIZE_BYTES 36
#define RAW_AND_CALIBRATED_SENSOR_OUTPUTS_PACKET_SIZE_BYTES 72
#define PACKET_TAHA 18 // 4 bytes accMag, 4 bytes gyroMag, 2 byte flag and packetNumber, 8 bytes accX and accY
using namespace std;

struct EulerAngles {
	float yaw, pitch, roll; // look at Razor_AHRS -> Output.ino to see the order
};

struct CalibratedSensorOutputs {
	float accX, accY, accZ, magX, magY, magZ, gyroX, gyroY, gyroZ; // look at Razor_AHRS.ino to see the order
};

struct RawAndCalibratedSensorOutputs { // look at Razor_AHRS.ino to see the order
	float raccX, raccY, raccZ, rmagX, rmagY, rmagZ, rgyroX, rgyroY, rgyroZ;
	float caccX, caccY, caccZ, cmagX, cmagY, cmagZ, cgyroX, cgyroY, cgyroZ;
};

struct PacketTaha {
	//float accX, accY, accZ, magX, magY, magZ, gyroX, gyroY, gyroZ; // look at Razor_AHRS.ino to see the order
	float accMag, gyroMag; // both data are used to determine stance and swing phase of the pedestrian
	float accX, accY; // wrt inertial coordinate system
	//float yaw, pitch, roll; // look at Razor_AHRS -> Output.ino to see the order
	bool G_DtFlag; // integration time for gyros - see Razor_AHRS.ino and DCM.ino for more
	uint8_t packetNumber;
};

struct Quaternion {
	float w, x, y, z;
};

enum format { EAngles = 1, CSOutputs = 2, RCSOutputs = 3, PackageTaha = 4 };

class RazorIMU
{
private:
	int baudRate; // Arduino supports 115200 as maximum baud-rate.
	char* portName; // go to {Control Panel --> Hardware and Sound --> Device Manager (under Devices and Printers) --> Ports} to see port number
	char* device;
	EulerAngles eulerAngles;
	CalibratedSensorOutputs calibratedSensorOutputs;
	RawAndCalibratedSensorOutputs rawAndCalibratedSensorOutputs;
	PacketTaha packetTaha;
	Quaternion quaternion;
	float DCM[3][3]; // also known as rotation matrix from inertial to body frame
	float RbodyToInertial[3][3]; // this is transpose of DCM matrix, hence rotation matrix from body to inertial frame
	float rawAccWRTinertialFrame[3];
	char* dataFormatRequestMessage;
	format dataFormat;
	bool displayData, writeData;
	bool continueStream;
	fstream fout;
	char* sensorDataFileName;
	Serial* serialConnection;
public:
	RazorIMU(char* _portName, const int _baudRate, bool _displayData, bool _writeData);
	~RazorIMU();
	// set_razorIMU_data_stream() function sends a string message to Razor IMU to set data format
	// "#ob" - Output angles in BINARY format(yaw / pitch / roll as binary float, so one output frame is 3x4 = 12 bytes long).
	// "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
	// One frame consist of three 3x3 float values = 36 bytes. Order is : acc x / y / z, mag x / y / z, gyr x / y / z.
	void user_interaction_to_set_data_format();
	void set_razorIMU_data_stream() const; // "#ob" or "oscb" see Razor_AHRS.ino
	int message_size() const;
	void read();
	void display_data_in_console() const;
	void write_data_to_file(fstream& fout) const;
	bool wanna_continue_stream();
	void euler_angles_to_quaternion();
	void euler_angles_to_DCM();
	void euler_angles_to_R_body_to_inertial();
	void represent_accelerometer_wrt_to_inertial_coordinate_system();
	float RazorIMU::degree_to_radian(float& deg) const;
};