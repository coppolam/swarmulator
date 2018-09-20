#ifndef TRIGONOMETRY_H
#define TRIGONOMETRY_H

#include <stdlib.h> // qsort
#include <cmath>
#include <vector>
#include <string>

using namespace std;

/* Wraps an angle in radians between -PI and +PI */
inline static void wrapToPi(float &ang)
{
  if (ang >  M_PI) { while (ang >  M_PI) { ang = ang - 2 * M_PI;} }
  else if (ang < -M_PI) { while (ang < -M_PI) { ang = ang + 2 * M_PI;} }
}

inline static void wrapTo2Pi(float &ang)
{
  while (ang > 2 * M_PI) {
    ang = ang - 2 * M_PI;
  }
  while (ang < 0.0) {
    ang = ang + 2 * M_PI;
  }
}

inline static float wrapToPi_f(float ang)
{
  if (ang > M_PI) {
    while (ang > M_PI) {
      ang = ang - 2 * M_PI;
    }
  } else if (ang < -M_PI) {
    while (ang < -M_PI) {
      ang = ang + 2 * M_PI;
    }
  }
  return ang;
}

inline static float wrapTo2Pi_f(float ang)
{
  if (ang > 2 * M_PI) {
    while (ang > 2 * M_PI) {ang = ang - 2 * M_PI;}
  } else if (ang < 0.0) {
    while (ang < 0.0) {
      ang = ang + 2 * M_PI;
    }
  }
  return ang;
}

inline static float rad2deg(float rad) { return 180.0 / M_PI * rad; }

inline static float deg2rad(float deg) { return M_PI / 180.0 * deg; }

// *Compliments of http : //stackoverflow.com/questions/29089710/pointers-in-c-programming-coordinate-conversion */
inline static void polar2cart(const float &radius, float &radians, float &x, float &y)
{
  wrapToPi(radians);
  x = radius * cos(radians);
  y = radius * sin(radians);
}

/* Function to convert cartesian coordinates to polar */
inline static void cart2polar(const float &x, const float &y, float &radius, float &radians)
{
  radius = sqrt(pow(x, 2) + pow(y, 2));
  radians = atan2(y, x);
}

#endif /*TRIGONOMETRY_H*/
