#include "boid.h"
#include "draw.h"
#include "main.h"
#include "auxiliary.h"

#define K_NEAREST_BOID 3
#define WALL_SENSOR 2.5

using namespace std;
boid::boid() : Controller()
{
}

void boid::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0.1;
  v_y = 0;

  if (!wall_avoidance_turn(ID, v_x, v_y, WALL_SENSOR)) {// can see walls at 2.5 m away
    // Get the k nearest neighbors, here k = 5
    get_lattice_motion_k_nearest(ID, v_x, v_y, K_NEAREST_BOID);

    vector<uint> closest = o.request_closest(ID);
    float avg_psi = 0.;
    for (int i = 0; i < min(int(closest.size()), int(K_NEAREST_BOID)); i++) {
      avg_psi += s[closest[i]]->state[6] / float(min(int(closest.size()), int(K_NEAREST_BOID)));
    }
    v_y = avg_psi - s[ID]->state[6];
  }
}

void boid::animation(const uint16_t ID)
{
  vector<float> r, b;
  o.relative_location(ID, r, b);
  draw d;
  for (int i = 0; i < min(int(r.size()), int(K_NEAREST_BOID)); i++) {
    float x, y;
    polar2cart(r[i], b[i], x, y);
    d.line(x, y, 0.5);
  }
}