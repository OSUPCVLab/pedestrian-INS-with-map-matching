#include "GabrielTimer.h"
#include "iostream"

GabrielTimer::GabrielTimer()
{
	this->tic = 0;
	this->toc = 0;
}

void GabrielTimer::capture_time()
{
	if (this->tic == 0)
		this->tic = micros();
	else
		this->toc = micros();
}

void GabrielTimer::differentiate()
{
	if (this->tic <= this->toc)
	{
		this->difference = this->toc - this->tic;
		reset_tic();
	}
	else
	{
		std::cerr << "We cannot go back in time: difference cannot be negative!" << std::endl;
		system("pause");
		exit(1);
	}
}

void GabrielTimer::reset_tic()
{
	this->tic = this->toc; // now tic and toc are equal. we made tic our reference.
}

double GabrielTimer::fps() const
{
	return (1000000.0 / this->difference);
}