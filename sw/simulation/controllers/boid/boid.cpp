#include "boid.h"
#include "draw.h"
#include "main.h"
#include "auxiliary.h"

#define K_NEAREST 3
template <typename T> int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

using namespace std;
boid::boid() : Controller()
{
}

void boid::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0.1;
  v_y = 0;

  if (!wall_avoidance_turn(ID, v_x, v_y)) {
    // Get the k nearest neighbors, here k = 5
    get_lattice_motion_all(ID, v_x, v_y, K_NEAREST);

    vector<uint> closest = o.request_closest(ID);
    float avg_psi;

    for (size_t i = 0; i < closest.size(); i++) {
      avg_psi += s[closest[i]]->state[6] / closest.size();
    }

    v_y = avg_psi - s[ID]->state[6];
    cout << minimum_angular_distance(s[ID]->state[6], avg_psi) << endl;
  }
}

void boid::animation(const uint16_t ID)
{
  vector<float> r, b;
  o.relative_location(ID, r, b);
  draw d;
  for (int i = 0; i < min(int(r.size()), int(K_NEAREST)); i++) {
    float x, y;
    polar2cart(r[i], b[i], x, y);
    d.line(x, y, 0.5);
  }
}