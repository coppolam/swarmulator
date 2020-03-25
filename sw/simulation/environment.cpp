#include "environment.h"
#include "settings.h"
#include "main.h"
#include "auxiliary.h"
#include "draw.h"

using namespace std;

Environment::Environment(void)
{
  define();
  animate();
}

void Environment::define(void)
{
  walls = read_matrix("conf/arenas/funnel.txt");
  draw();
}

void Environment::add(float x0, float y0, float x1, float y1)
{  
  mtx.lock();
  walls.push_back(std::vector<float>());
  walls[walls.size()-1].push_back(x0);
  walls[walls.size()-1].push_back(y0);
  walls[walls.size()-1].push_back(x1);
  walls[walls.size()-1].push_back(y1);
  mtx.unlock();
}

bool Environment::sensor(uint8_t ID, vector<float> s_n, vector<float> s)
{
  Point p1, q1, p2, q2;
  p1.x = s[0];
  p1.y = s[1];
  q1.x = s_n[0];
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