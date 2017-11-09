#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <stdlib.h> // qsort
#include <cmath>
#include <vector>
#include <string>

using namespace std;

inline static int bool2int(vector<bool> t)
{
	int n = 0; //initialize
	for (int i = 0; i < 8; i++)
	{
		n += (int)t[i]*(int)pow(2,7-i);
	}   	
	return n;
}

/* Keeps a value between two bounds */
inline static void keepbounded(float &value, float min, float max)
{
	if (value < min) { value = min; }
	else if (value > max) { value = max; }
}

/* Wraps an angle in radians between -PI and +PI */
inline static void wrapToPi(float &ang)
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

inline static void wrapTo2Pi(float &ang)
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

inline static float wrapToPi_f(float ang)
{
	if (ang > M_PI) {while (ang > M_PI) {ang = ang - 2*M_PI;}}
	else if (ang < -M_PI) {while (ang < -M_PI) {ang = ang + 2*M_PI;}}
	return ang;
}

inline static float wrapTo2Pi_f(float ang)
{
	if (ang > 2*M_PI) {while (ang > 2*M_PI) {ang = ang - 2*M_PI;}}
	else if (ang < 0.0) { while (ang < 0.0) { ang = ang + 2*M_PI; }}
	return ang;
}
/* Function to convert radians to degrees */
inline static float rad2deg(float rad) { return 180.0 / M_PI * rad; }

/* Function to convert degrees to radians */
inline static float deg2rad(float deg) {return M_PI / 180.0 * deg;}


#endif /*AUXILIARY_H*/