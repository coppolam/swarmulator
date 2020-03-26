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

using namespace std;

Environment::Environment(void)
{
  define();
  animate();
}

void Environment::define(void)
{
  string filename = "conf/environments/" + param->environment() + ".txt";
  walls = read_matrix(filename);
  draw();
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

void Environment::add(float x0, float y0, float x1, float y1)
{
  mtx.lock();
  walls.push_back(vector<float>());
  walls[walls.size() - 1].push_back(x0);
  walls[walls.size() - 1].push_back(y0);
  walls[walls.size() - 1].push_back(x1);
  walls[walls.size() - 1].push_back(y1);
  mtx.unlock();
}

bool Environment::sensor(uint8_t ID, vector<float> s_n, vector<float> s)
{
  Point p1, q1, p2, q2;
  p1.x = s[0];
  p1.y = s[1];
  q1.x = s_n[0]; // +s_n[2] and +s_n[3] as error margins to avoid rounding
  q1.y = s_n[1];
  for (size_t i = 0; i < walls.size(); i++) {
    p2.y = walls[i][0];
    p2.x = walls[i][1];
    q2.y = walls[i][2];
    q2.x = walls[i][3];
    if (doIntersect(p1, q1, p2, q2)) {
      return true;
    }
  }
  return false;
}

void Environment::animate(void)
{
  draw d;
  for (size_t i = 0; i < walls.size(); i++) {
    d.draw_line(walls[i][0], walls[i][1], walls[i][2], walls[i][3]);
  }
}