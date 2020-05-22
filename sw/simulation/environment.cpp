#include "environment.h"
#include "main.h"
#include "settings.h"
#include "auxiliary.h"
#include "draw.h"

#include <vector>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "randomgenerator.h"

using namespace std;

Environment::Environment(void)
{
  define_walls();
  define_food(100);
  define_beacon(0., 0.);
  nest = 10;
}

void Environment::define_walls(void)
{
  string s = param->environment();
  if (!strcmp(s.c_str(), "random")) {
    stringstream ss("python3 scripts/python/dungeon_generator.py && mv rooms.txt conf/environments/random.txt");
    system(ss.str().c_str());
    terminalinfo::info_msg("Generating random environment");
  }
  string filename = "conf/environments/" + param->environment() + ".txt";
  walls = read_matrix(filename);
}

void Environment::define_food(uint64_t n)
{
  random_generator rg;
  float lim = limits();
  for (size_t i = 0; i < n; i++) {
    food.push_back(vector<float>());
    food[i].push_back(rg.uniform_float(-lim, lim));
    food[i].push_back(rg.uniform_float(-lim, lim));
  }
}

void Environment::define_beacon(float x, float y)
{
  beacon.push_back(x);
  beacon.push_back(y);
}


// TODO: Temporary function for initialization, but the initalization should change eventually
vector<float> Environment::start(void)
{
  vector<float> s(2);
  s[0] = walls[0][0] + 1.0;
  s[1] = walls[0][1] - 1.0;
  return s;
}

// TODO: Temporary function for initialization, but the initalization should change eventually
float Environment::limits(void)
{
  float max = 0;
  for (size_t i = 0; i < walls.size(); i++) {
    float v = *max_element(walls[i].begin(), walls[i].end()); // c++11
    if (max < v) {
      max = v;
    }
  }
  return max * 0.95; // 0.95 for margin
}

void Environment::add_wall(float x0, float y0, float x1, float y1)
{
  mtx.lock();
  walls.push_back(vector<float>());
  walls[walls.size() - 1].push_back(x0);
  walls[walls.size() - 1].push_back(y0);
  walls[walls.size() - 1].push_back(x1);
  walls[walls.size() - 1].push_back(y1);
  mtx.unlock();
}

bool Environment::sensor(const uint16_t ID, vector<float> s_n, vector<float> s, float &angle)
{
  Point p1, q1, p2, q2;
  p1.y = s[0]; // Flip axis
  p1.x = s[1];
  q1.y = s_n[0];
  q1.x = s_n[1];
  for (size_t i = 0; i < walls.size(); i++) {
    p2.x = walls[i][0];
    p2.y = walls[i][1];
    q2.x = walls[i][2];
    q2.y = walls[i][3];
    if (doIntersect(p1, q1, p2, q2)) {
      angle = atan2(p2.y - q2.y, p2.x - q2.x);
      return true;
    }
  }
  return false;
}

void Environment::animate(void)
{
  draw d;
  for (size_t i = 0; i < walls.size(); i++) {
    d.segment(walls[i][0], walls[i][1], walls[i][2], walls[i][3]);
  }

  for (size_t i = 0; i < food.size(); i++) {
    d.food(food[i][0], food[i][1]);
  }
}


void Environment::grab_food(uint64_t food_ID)
{
  mtx_env.lock();
  food.erase(food.begin() + food_ID);
  mtx_env.unlock();
}

void Environment::drop_food()
{
  mtx_env.lock();
  nest += 1.;
  mtx_env.unlock();
}

void Environment::eat_food(float amount)
{
  if (nest > amount) {
    mtx_env.lock();
    nest -= amount;
    // cout << nest << endl;;
    mtx_env.unlock();
    // terminalinfo::info_msg("Oh no! The swarm ran out of food! Quitting.");
    // program_running = false;
  }
}
