#pragma once
#include <stdint.h>
#include <Windows.h>

class GabrielTimer
{
private:
	uint32_t tic, toc;
	uint32_t difference;
public:
	GabrielTimer();
	void capture_time();
	void differentiate();
	void reset_tic();
	double fps() const;

	/* http://stackoverflow.com/questions/953710/inline-function-linker-error */
	inline uint32_t micros()
	{
		LARGE_INTEGER tics, freq;
		QueryPerformanceCounter(&tics); // get ticks on the internal ~2MHz QPC clock
		QueryPerformanceFrequency(&freq); // get the actual freq. of the internal ~2MHz QPC clock 
		uint64_t t_us = tics.QuadPart*1e6 / freq.QuadPart; // us
		return t_us;
	}

	inline uint32_t millis()
	{
		LARGE_INTEGER tics, freq;
		QueryPerformanceCounter(&tics); //get ticks on the internal ~2MHz QPC clock
		QueryPerformanceFrequency(&freq); //get the actual freq. of the internal ~2MHz QPC clock 
		uint64_t t_ms = tics.QuadPart*1e3 / freq.QuadPart; //ms 
		return t_ms;
	}
};