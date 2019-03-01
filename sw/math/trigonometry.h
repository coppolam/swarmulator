#ifndef TRIGONOMETRY_H
#define TRIGONOMETRY_H

#include <stdlib.h> // qsort
#include <cmath>
#include <vector>
#include <string>

using namespace std;

/**
 * Wraps an angle in radians between -PI and +PI, overwrites onto the variable
 */
inline static void wrapToPi(float &ang)
{
  if (ang >  M_PI) { while (ang >  M_PI) { ang = ang - 2 * M_PI;} }
  else if (ang < -M_PI) { while (ang < -M_PI) { ang = ang + 2 * M_PI;} }
}

/**
 * Wraps an angle in radians between 0 and +2PI, overwrites onto the variable
 */
inline static void wrapTo2Pi(float &ang)
{
  while (ang > 2 * M_PI) {
    ang = ang - 2 * M_PI;
  }
  while (ang < 0.0) {
    ang = ang + 2 * M_PI;
  }
}

/**
 * Wraps an angle in radians between -PI and +PI, returns the wrapped value
 */
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

/**
 * Wraps an angle in radians between 0 and +2PI, returns the wrapped value
 */
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

/**
 * Converts an angle from radians to degrees
 */
inline static float rad2deg(float rad) { return 180.0 / M_PI * rad; }

/**
 * Converts an angle from degrees to radians
 */
inline static float deg2rad(float deg) { return M_PI / 180.0 * deg; }

/**
 * Convert polar (r,theta) coordinates to cartesian (x,y) coordinates
 */
inline static void polar2cart(const float &radius, float &radians, float &x, float &y)
{
  wrapToPi(radians);
  x = radius * cos(radians);
  y = radius * sin(radians);
}

/**
 * Convert cartesian (x,y) coordinates to polar (r,theta) coordinates
 */
inline static void cart2polar(const float &x, const float &y, float &radius, float &radians)
{
  radius = sqrt(pow(x, 2) + pow(y, 2));
  radians = atan2(y, x);
}

/**
 * Convert cartesian (x,y) coordinates to polar (r,theta) coordinates
 */
inline static void rotate_xy(const float &x, const float &y, const float &theta, float &xr, float &yr)
{
  xr = x * cos(theta) - y * sin(theta);
  yr = x * sin(theta) + y * cos(theta);
}

#endif /*TRIGONOMETRY_H*/
