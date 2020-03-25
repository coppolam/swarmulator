#include "arena.h"
#include "settings.h"
#include "main.h"
#include "auxiliary.h"
#include "draw.h"

using namespace std;

Arena::Arena(void)
{
  std::cout << "Hello" << std::endl;
  define();
  animate();
}

void Arena::define(void)
{
  walls = read_matrix("conf/arenas/square.txt");
  draw();
}

void Arena::add(void)
{
  // TODO: User interaction
}

bool Arena::sensor(uint8_t ID, vector<float> s_n, vector<float> s)
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

void Arena::animate(void)
{
  draw d;
  for (size_t i = 0; i < walls.size(); i++) {
    d.draw_line(walls[i][0], walls[i][1], walls[i][2], walls[i][3]);
  }
}