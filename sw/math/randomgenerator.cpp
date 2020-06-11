#include "randomgenerator.h"
#include <iostream>
#include "stdint.h"
#include "auxiliary.h"

random_generator::random_generator()
{
  std::random_device rd;
  generator.seed(rd());
};

float random_generator::uniform_float(float min, float max)
{
  std::uniform_real_distribution<> dist(min, max);
  return dist(generator);
};

int random_generator::uniform_int(int min, int max)
{
  std::uniform_int_distribution<> dist(min, max);
  return dist(generator);
};

float random_generator::gaussian_float(float mean, float stddev)
{
  std::normal_distribution<> dist(mean, stddev);
  return dist(generator);
};

bool random_generator::bernoulli(float p)
{
  std::bernoulli_distribution dist(p);
  return dist(generator);
}


int random_generator::discrete_int(std::vector<float> &d)
{
  std::discrete_distribution<int> dist(d.begin(), d.end());
  return dist(generator);
}

std::vector<float> random_generator::gaussian_float_vector(const int &length, const float &mean, const float &std)
{
  // Generate the random vector
  std::vector<float> v(length, 0);
  for (uint16_t i = 0; i < length; i++) {
    v[i] = gaussian_float(mean, std);
  }

  return v;
}

/**
 * @brief
 *
 * @param length
 * @param min
 * @param max
 * @return vector<float>
 */
std::vector<float> random_generator::uniform_float_vector(const int &length, const float &min, const float &max)
{
  // Generate the random vector
  std::vector<float> v(length, 0);
  for (uint16_t i = 0; i < length; i++) {
    v[i] = uniform_float(min, max);
  }

  return v;
}
