#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <stdlib.h> // qsort
#include <cmath>
#include <vector>
#include <string>
#include <random>
#include <algorithm> // std::transform

using namespace std;

/**
 * Increase a counter by 1, or reset to 1 if above a given limit
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
 * Convert an 8bit boolean vector to an unsigned integer
 */
inline static int bool2int(vector<bool> t)
{
  int n = 0; //Initialize
  for (int i = 0; i < 8; i++) {
    n += (int)t[i] * (int)pow(2, 7 - i);
  }
  return n;
}

/**
 * Bind a value between a minimum and a maximum
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

/*
 * Calculate the mean of all elements in a vector
 */
inline static float vector_mean(const vector<float> &v)
{
  float sum = std::accumulate(v.begin(), v.end(), 0.0);
  return sum / v.size();
}

/*
 * Calculate the standard deviation of all elements in a vector
 */
inline static float get_vector_std(const vector<float> &v)
{
  vector<double> diff(v.size());
  transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), vector_mean(v)));
  double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return sqrt(sq_sum / v.size());
}

#endif /*AUXILIARY_H*/
