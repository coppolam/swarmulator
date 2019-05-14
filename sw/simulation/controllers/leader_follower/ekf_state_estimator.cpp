#include "ndi_follower.h"
#include "main.h"
#include <iostream>
#include <string>
#include <sstream>
#include "trigonometry.h"

// Initilizer
ekf_state_estimator::ekf_state_estimator() {
  initialized = false;
};

void ekf_state_estimator::init_ekf_filter() {
  float pxf, pyf;
  polar2cart(o.request_distance(ID, ID_tracked), o.request_bearing(ID, ID_tracked), pxf, pyf);
  discrete_ekf_no_north_new(&ekf_rl);
  ekf_rl.X[0] = pxf;
  ekf_rl.X[1] = pyf;
  ekf_rl.X[8] = wrapToPi_f(s[ID_tracked]->get_state(6) - s[ID]->get_state(6));
  initialized = true;
  simtime_seconds_store = simtime_seconds;
}

void ekf_state_estimator::run_ekf_filter() {
  // All in local frame of follower!!!! values for position, velocity, acceleration
  float vxf, vyf, vx0f, vy0f, axf, ayf, ax0, ay0;
  // Global to local, rotate the opposite of local to global, hence the negative
  rotate_xy(s[ID]->get_state(2), s[ID]->get_state(3), -s[ID]->get_state(6), vxf,  vyf);
  rotate_xy(s[ID]->get_state(4), s[ID]->get_state(5), -s[ID]->get_state(6), axf, ayf);
  rotate_xy(s[ID_tracked]->get_state(2), s[ID_tracked]->get_state(3), -s[ID_tracked]->get_state(6), vx0f, vy0f);
  rotate_xy(s[ID_tracked]->get_state(4), s[ID_tracked]->get_state(5), -s[ID_tracked]->get_state(6), ax0, ay0);
  ekf_rl.dt = simtime_seconds - simtime_seconds_store;
  simtime_seconds_store = simtime_seconds;
  float U[EKF_L] = {axf, ayf, ax0, ay0, s[ID]->get_state(7), s[ID_tracked]->get_state(7)};
  float Z[EKF_M] = {o.request_distance(ID, ID_tracked), 0.0, 0.0, vxf, vyf, vx0f, vy0f};
  discrete_ekf_no_north_predict(&ekf_rl, U);
  discrete_ekf_no_north_update(&ekf_rl, Z);
  ekf_rl.X[8] = wrapToPi_f(ekf_rl.X[8]);
}

void ekf_state_estimator::run(uint8_t ID_in, uint8_t ID_tracked_in)
{
  if (!initialized) {
    ID = ID_in;
    ID_tracked = ID_tracked_in;
    init_ekf_filter();
    printf("Launched EKF instance for %d to %d\n", ID, ID_tracked);
  } else {
    run_ekf_filter();
  }
}
