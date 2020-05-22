#include "forage.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"
#include "math.h"

#define INITIAL_STATE 100
forage::forage() : Controller()
{
  // Initial values
  st = INITIAL_STATE; // initial value
  moving = false;
  v_x_ref = 0.0;
  v_y_ref = 0.0;
  moving_timer = rg.uniform_int(0, timelim);
  moving_timer_1 = rg.uniform_int(0, timelim * 5);
  holds_food = false;
  choose = false;

  // Load policy
  if (!strcmp(param->policy().c_str(), "")) {
    motion_p = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    // motion_p = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
  } else {
    motion_p = read_array(param->policy());
  }

  // Control value
  timelim = 10.0 * param->simulation_updatefreq();
  vmean = 0.5;
}

void forage::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  // Get lattice motion for collision avoidance by controlling speed
  float temp;
  get_lattice_motion_all(ID, v_x, temp); // Repulsion from neighbors

  // Decision making.
  // If at the beacon, and you haven't chosen what to do. Make a choice!
  float br, bt;
  o.beacon(ID, br, bt);
  if (br < rangesensor && !choose) {
    choose = true;
    // state, action
    st = min(int(environment.nest), int(motion_p.size() - 1));
#ifdef ESTIMATOR
    int a;
    if (moving) {a = 1;} else {a = 0;}
    pr.update(ID, st, a);
#endif
    if (rg.bernoulli(1.0 - motion_p[st])) { // Move
      v_x_ref = 0.0;
      v_y_ref = 0.0;
      moving = false;
    } else { // Else explore randomly, change heading
      moving = true;
    }
  }

  // Eat some food every time you are moving and take another maneuver.
  if (moving && moving_timer == 1) {
    v_x_ref = vmean;
    v_y_ref = rg.gaussian_float(0., 0.2);
    wrapToPi(v_y_ref);
    environment.eat_food(0.1);
  }

  /** Routine to sense food and grab it in case **/
  uint16_t ID_food; // for sim purposes, used to delete the correct food item once grabbed
  // Sense the food, return true if sensed and assign ID_food
  if (o.sense_food(ID, ID_food) && !holds_food && st != INITIAL_STATE) {
    environment.grab_food(ID_food); // Grab the food item ID_food
    holds_food = true;
  }

  // Go to beacon and drop food in case.
  if (holds_food || moving_timer_1 == 1 || st == INITIAL_STATE) {
    choose = false;
    o.beacon(ID, br, v_y_ref); // get angle to beacon
    v_y_ref = 0.1 * wrapToPi_f(v_y_ref); // gain on control
    v_x_ref = vmean;
    // Drop the food if you are in the vicinity of the nest
    if (holds_food && br < rangesensor * 2) {
      environment.drop_food();
      holds_food = false;
    }
  }

  // Final output
  v_x += v_x_ref;
  v_y += v_y_ref; // Psi_ref

  // Wall avoidance
  wall_avoidance_t(ID, v_x, v_y);


  // Counters
  increase_counter_to_value(moving_timer, timelim, 1);
  increase_counter_to_value(moving_timer_1, timelim * 5, 1);
}
