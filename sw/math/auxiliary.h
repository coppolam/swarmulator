#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <stdlib.h> // qsort
#include <cmath>
#include <vector>
#include <string>
#include <random>
#include <algorithm> // std::transform
#include <map>
#include <fstream>
#include <sstream>
#include <random>
#include <iterator>
#include <stdio.h>
#include "terminalinfo.h"
#include "fmat.h"

using namespace std;

/**
 * @brief Returns whether a number is positive or negative as a float +1, 0, -1
 *
 * @param x
 * @return float
 */
inline static float sign(float x)
{
  if (x > 0) { return 1; }
  if (x < 0) { return -1; }
  return 0;
}

/**
 * Increase a counter by 1, or reset to 1 if above a given limit
 *
 * @param counter The counter value (uint). This will be increased by one.
 * @param limit The limit of the counter. If this is passed. Then counter = 1.
 */
inline static void increase_counter(uint &counter, const uint &limit)
{
  if (counter > limit) {
    counter = 1;
  } else {
    counter++;
  }
}

/**
 * Increase a counter by 1, or reset to a value if above a given limit
 *
 * @param counter The counter value (uint). This will be increased by one.
 * @param limit The limit of the counter. If this is passed. Then counter = 1.
 */
inline static void increase_counter_to_value(uint &counter, const uint &limit, const uint &reset_value)
{
  if (counter > limit) {
    counter = reset_value;
  } else {
    counter++;
  }
}


/**
 * Convert an 8bit boolean vector to an unsigned integer
 * TODO: Add check for vector length
 *
 * @param t An 8 bit boolean vector
 * @return Integer value of the boolean vector
 */
inline static uint bool2int(vector<bool> t)
{
  uint n = 0; //Initialize
  for (uint i = 0; i < t.size(); i++) {
    n += (uint)t[i] * (uint)pow(2, t.size() - 1 - i);
  }
  return n;
}

/**
 * @brief Bind (saturate) a value between a minimum and a maximum.
 * This is basically a saturation filter.
 *
 * @param value The value of interest
 * @param min Minimum bound
 * @param max Maximum bound
 */
inline static void keepbounded(float &value, float min, float max)
{
  if (value < min) { value = min; }
  else if (value > max) { value = max; }
}

/**
 * Wrap an integer value to a sequence.
 * For instance if min = 1 and max = 8, and x = 10, then the function returns 2. Because it loops 8+2=10
 * TODO: this works only if min = 1 so adjust it
 */
inline static int wraptosequence(int x, int min, int max)
{
  if (x > max) {
    while (x > max) {
      x -= max;
    }
  } else {
    while (x < min) {
      x += max;
    }
  }
  return x;
}

/**
 * Select a random value
 * Compliments of Christopher Smith
 * https://stackoverflow.com/questions/6942273/how-to-get-a-random-element-from-a-c-container
 */
template <typename Iter, typename RandomGenerator>
inline static Iter select_randomly(Iter start, Iter end, RandomGenerator &g)
{
  uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
  advance(start, dis(g));
  return start;
}

/**
 * Select a random value
 * Compliments of Christopher Smith
 * https://stackoverflow.com/questions/6942273/how-to-get-a-random-element-from-a-c-container
 */
template <typename Iter>
inline static Iter select_randomly(Iter start, Iter end)
{
  static random_device rd;
  static mt19937 gen(rd());
  return select_randomly(start, end, gen);
}

/**
 * Calculate the mean of all elements in a vector
 *
 * @param v std::vector holding the values
 */
inline static float vector_mean(const vector<float> &v)
{
  float sum = std::accumulate(v.begin(), v.end(), 0.0);
  return sum / v.size();
}

/**
 * Calculate the standard deviation of all elements in a vector
 *
 * @param v std::vector holding the values
 */
inline static float get_vector_std(const vector<float> &v)
{
  vector<double> diff(v.size());
  transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), vector_mean(v)));
  double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return sqrt(sq_sum / v.size());
}

/**
 * Read a matrix from a txt file
 *
 * @param filename = name of file
 */
inline static std::vector<std::vector<float>> read_matrix(const string filename)
{
  ifstream in(filename);
  std::string line;
  std::vector<std::vector<float>> matrix;
  uint rows = 0;
  if (in.is_open()) {
    while (!in.eof()) {
      std::getline(in, line);
      std::stringstream ss(line);
      matrix.push_back(std::vector<float>());
      float value;
      while (ss >> value) {
        matrix[rows].push_back(value);
      }
      rows++;
    }
  } else {
    std::string msg = "Matrix file not loaded " + filename;
    terminalinfo::error_msg(msg);
  }
  return matrix;
}

/**
 * Read an array from a txt file
 *
 * @param filename = name of file
 */
inline static std::vector<float> read_array(const string filename)
{
  ifstream in(filename);
  std::string line;
  std::vector<float> array;
  if (in.is_open()) {
    std::getline(in, line);
    std::stringstream ss(line);
    float value;
    while (ss >> value) {
      array.push_back(value);
    }
  } else {
    terminalinfo::error_msg("Array file not loaded: " + filename);
  }
  return array;
}

/**
 * 2D Point class use for the functions below
 *
 */
struct Point {
  float x;
  float y;
};

/**
 * Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr'
 *
 * @param p
 * @param q
 * @param r
 * @return true
 * @return false
 */
inline static bool onSegment(Point p, Point q, Point r)
{
  if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
      q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) {
    return true;
  }

  return false;
}

/**
 * Function to find orientation of ordered triplet (p, q, r).
 * The function returns following values
 * 0 --> p, q and r are colinear
 * 1 --> Clockwise
 * 2 --> Counterclockwise
 *
 * @param p
 * @param q
 * @param r
 * @return int
 */
inline static int orientation(Point p, Point q, Point r)
{
  // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
  // for details of below formula.
  int val = (q.y - p.y) * (r.x - q.x) -
            (q.x - p.x) * (r.y - q.y);

  if (val == 0) { return 0; }  // colinear

  return (val > 0) ? 1 : 2; // clock or counterclock wise
}

/**
 * Returns true if line segment 'p1q1' and 'p2q2' intersect, false otherwise.
 *
 * @param p1 Start segment 1
 * @param q1 End segment 1
 * @param p2 Start segment 2
 * @param q2 End segment 2
 * @return true Lines intersect
 * @return false Lines do not intersect
 */
inline static bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
  // Find the four orientations needed for general and
  // special cases
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);

  // General case
  if (o1 != o2 && o3 != o4) {
    return true;
  }

  // Special Cases
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1
  if (o1 == 0 && onSegment(p1, p2, q1)) { return true; }

  // p1, q1 and q2 are colinear and q2 lies on segment p1q1
  if (o2 == 0 && onSegment(p1, q2, q1)) { return true; }

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2
  if (o3 == 0 && onSegment(p2, p1, q2)) { return true; }

  // p2, q2 and q1 are colinear and q1 lies on segment p2q2
  if (o4 == 0 && onSegment(p2, q1, q2)) { return true; }

  return false; // Doesn't fall in any of the above cases
}

/**
 * Get current date/time, format is YYYY-MM-DD-hh:mm:ss
 * Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
 * for more information about date/time format
 *
 * @return A character string with the current date and time.
 */
inline static const std::string currentDateTime()
{
  time_t now = time(0); // Read in the time
  struct tm tstruct;
  char buf[80]; // Buffer
  tstruct = *localtime(&now);

  // Put the time on a string using the buffer
  strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);
  return buf;
}

#endif /*AUXILIARY_H*/
