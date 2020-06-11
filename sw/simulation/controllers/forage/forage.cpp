#include "forage.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"
#include "math.h"
#include "draw.h"

using namespace std;

forage::forage() : Controller()
{
  // Initial values
  timer = rg.uniform_int(0, timelim);
  choose = false;
  holds_food = false;
  v_x_ref = 0.0;
  v_y_ref = 0.0;

  // Control values
  timelim = 10.0 * param->simulation_updatefreq();
  vmean = 0.5;

  // Load policy
  if (!strcmp(param->policy().c_str(), "")) { motion_p.assign(16, 0.5); }
  else { motion_p = read_array(param->policy()); }
}

void forage::get_velocity_command(const uint16_t ID, float &v_x, float &psi_rate)
{
  v_x = 0;
  psi_rate = 0;
  float temp, br;
  get_lattice_motion_all(ID, v_x, temp); // Repulsion from neighbors

  // Make the choice. To explore or not to explore?
  if (!choose) {
    choose = true;
    state = environment.nest; // +8 to center around 0
    keepbounded(state, 0, 15);
    st = int(state);
    if (rg.bernoulli(1.0 - motion_p[st])) { explore = false; }
    else { explore = true; } //environment.eat_food(0.1); }
  }

  // Behavior
  if (explore) {
    if (timer == 1) { // Go explore, change direction every new timer
      v_x_ref = vmean;
      v_y_ref = wrapToPi_f(rg.gaussian_float(0., 0.2));
    }

    uint16_t ID_food; // for sim purposes, used to delete the correct food item once grabbed
    if (holds_food) {
      o.beacon(ID, br, v_y_ref); // get distance + angle to beacon
      v_x_ref = br;
      v_y_ref = 0.5 * wrapToPi_f(v_y_ref); // gain on control
      if (br < rangesensor) { // Drop the food if you are in the vicinity of the nest
        environment.drop_food();
        holds_food = false;
        choose = false;
        timer = 1; // reset timer
      }
    } else if (o.sense_food(ID, ID_food)) {
      environment.grab_food(ID_food); // Grab the food item ID_food
      holds_food = true;
    }
  } else { // don't explore
    v_x_ref = 0.0;
    v_y_ref = 0.0;
    if (timer == 1) {choose = false;}
  }
  increase_counter_to_value(timer, timelim, 1);
  v_x += v_x_ref;
  psi_rate += v_y_ref;
  wall_avoidance_turn(ID, v_x, psi_rate);
}

void forage::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}