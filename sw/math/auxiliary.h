#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <stdlib.h>     /* qsort */
#include <cmath>

void debug_msg(string str)
{
	#ifdef DEBUG // To be defined in the makefile so as to be applicable globally.
	cout << "\e[01;31m[DEBUG]: \e[0m" << str << endl;
	#endif
}

void info_msg(string str)
{
	#ifdef INFO // To be defined in the makefile so as to be applicable globally.
	cout << "\e[01;34m[INFO]: \e[0m" << str << endl;
	#endif
}

/* Keeps a value between two bounds */
void keepbounded(float &value, float min, float max)
{
	if (value < min) { value = min; }
	else if (value > max) { value = max; }
}

/* Wraps an angle in radians between -PI and +PI */
void wrapToPi(float &ang)
{
	if (ang > M_PI) {
		while (ang > M_PI) {		
			ang = ang - 2*M_PI;
		}
	}	
	else if (ang < -M_PI) {
		while (ang < -M_PI) {
			ang = ang + 2*M_PI;
		}
	}
}

void wrapTo2Pi(float &ang)
{
	if (ang > 2*M_PI) {
		while (ang > 2*M_PI) {		
			ang = ang - 2*M_PI;
		}
	}

	else if (ang < 0.0) {
		while (ang < 0.0) {
			ang = ang + 2*M_PI;
		}
	}
}

float wrapToPi_f(float ang)
{
	if (ang > M_PI) {
		while (ang > M_PI) {		
			ang = ang - 2*M_PI;
		}
	}	
	else if (ang < -M_PI) {
		while (ang < -M_PI) {
			ang = ang + 2*M_PI;
		}
	}
	return ang;
}

float wrapTo2Pi_f(float ang)
{
	if (ang > 2*M_PI) {
		while (ang > 2*M_PI) {		
			ang = ang - 2*M_PI;
		}
	}

	else if (ang < 0.0) {
		while (ang < 0.0) {
			ang = ang + 2*M_PI;
		}
	}
	return ang;
	
}
/* Function to convert radians to degrees */
float rad2deg(float rad)
{
	return 180.0 / M_PI * rad;
}

/* Function to convert degrees to radians */
float deg2rad(float deg)
{
	return M_PI / 180.0 * deg;
}


#endif /*AUXILIARY_H*/