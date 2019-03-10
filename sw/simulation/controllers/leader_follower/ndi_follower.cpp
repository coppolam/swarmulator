#include "ndi_follower.h"
#include "main.h"
#include <iostream>
#include <string>
#include <sstream>

#define NDI_METHOD 1
// method 0 is first order approximation, no acceleration or yaw rate used
// method 1 is first order approximation, acceleration and yaw rate used, but yaw rate not taken into account in integral

#define NDI_MOST_RECENT ndihandle.data_entries-1

// Delay of trajectory with respect to the leader, if not specified in airframe file
#ifndef UWB_NDI_DELAY
#define UWB_NDI_DELAY 4
#endif

#define TRAJ_EPS 0.5
#define TRAJ_LENGTH 4


extern "C" {
#define MAKE_MATRIX_PTR(_ptr, _mat, _rows) \
  float * _ptr[_rows]; \
  { \
    int __i; \
    for (__i = 0; __i < _rows; __i++) { _ptr[__i] = &_mat[__i][0]; } \
  }

/** a = 0 */
inline void float_mat_zero(float **a, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { a[i][j] = 0.; }
  }
}

/** o = a * b
 *
 * a: [m x n]
 * b: [n x 1]
 * o: [m x 1]
 */
inline void float_mat_vect_mul(float *o, float **a, float *b, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    o[i] = 0;
    for (j = 0; j < n; j++) {
      o[i] += a[i][j] * b[j];
    }
  }
}
}
float ndi_follower::accessCircularFloatArrElement(float arr[], int index)
{
  float value;
  int realindex = (ndihandle.data_start + index) % NDI_PAST_VALS;
  value = arr[realindex];
  return value;
}

// TODO: Trapezoidal integration, and not simply discarding values before tcur-delay, but linearly interpolating to tcur-delay
float ndi_follower::computeNdiFloatIntegral(float ndiarr[], float curtime)
{
  float integral = 0;
  float dt;
  for (int i = 0; i < ndihandle.data_entries - 1; i++) {
    dt = accessCircularFloatArrElement(ndihandle.tarr, i + 1) - accessCircularFloatArrElement(ndihandle.tarr, i);
    integral += dt * accessCircularFloatArrElement(ndiarr, i);
  }
  dt = curtime - accessCircularFloatArrElement(ndihandle.tarr, NDI_MOST_RECENT);
  integral += dt * accessCircularFloatArrElement(ndiarr, NDI_MOST_RECENT);
  return integral;
}


bool ndi_following_leader = false;
bool ndi_run_computation = true;
int traj_targetindex = 0;

#define UWB_LOWPASS_CUTOFF_FREQUENCY_YAWR 8
// Butterworth2LowPass uwb_butter_yawr;


// static abi_event relative_localization_event;

// extern void uwb_follower_control_init(void)
// {
  // init_butterworth_2_low_pass(&uwb_butter_yawr, UWB_LOWPASS_CUTOFF_FREQUENCY_YAWR, 1. / PERIODIC_FREQUENCY, 0.0);
  // AbiBindMsgRELATIVE_LOCALIZATION(ABI_BROADCAST, &relative_localization_event, relative_localization_callback);
// }

void ndi_follower::cleanNdiValues(float tcur)
{
  int curentries = ndihandle.data_entries;
  for (int i = 0; i < curentries; i++) {
    if ((tcur - ndihandle.tarr[ndihandle.data_start]) > ndihandle.delay) {
      ndihandle.data_start = (ndihandle.data_start + 1) % NDI_PAST_VALS;
      ndihandle.data_entries--;
    }
  }
  return;
}

/**
  * Update the NDI controller periodically
  */
void ndi_follower::uwb_follower_control_periodic(void)
{
  // Re-initialize commands
  ndihandle.commands[0] = 0;
  ndihandle.commands[1] = 0;

  // Get current values
  float curtime = simtime_seconds;
  cleanNdiValues(curtime);
  if (ndihandle.data_entries > 0)
  {
    float oldx = accessCircularFloatArrElement(ndihandle.xarr, 0);
    float oldy = accessCircularFloatArrElement(ndihandle.yarr, 0);
    float newu1 = accessCircularFloatArrElement(ndihandle.u1arr, NDI_MOST_RECENT);
    float newv1 = accessCircularFloatArrElement(ndihandle.v1arr, NDI_MOST_RECENT);
    float oldu2 = accessCircularFloatArrElement(ndihandle.u2arr, 0);
    float oldv2 = accessCircularFloatArrElement(ndihandle.v2arr, 0);
    oldx = oldx - computeNdiFloatIntegral(ndihandle.u1arr, curtime);
    oldy = oldy - computeNdiFloatIntegral(ndihandle.v1arr, curtime);

    float Minv[2][2];
    MAKE_MATRIX_PTR(_MINV, Minv, 2);
    float_mat_zero(_MINV, 2, 2); //fmat_make_zeros(Minv, 2, 2);
    Minv[0][0] = -ndihandle.tau_x;
    Minv[1][1] = -ndihandle.tau_y;
    float l[2], oldxed, oldyed;

#if(NDI_METHOD==0)
    l[0] = newu1 / ndihandle.tau_x;
    l[1] = newv1 / ndihandle.tau_y;
    oldxed = oldu2 - newu1;
    oldyed = oldv2 - newv1;
#elif(NDI_METHOD==1)
    float newr1 = accessCircularFloatArrElement(ndihandle.r1arr, NDI_MOST_RECENT);
    float oldax2 = accessCircularFloatArrElement(ndihandle.ax2arr, 0);
    float olday2 = accessCircularFloatArrElement(ndihandle.ay2arr, 0);
    l[0] = (newu1 - newr1 * newr1 * oldx - newr1 * ndihandle.tau_x * newv1 + oldax2 * ndihandle.tau_x + 2 * newr1 * ndihandle.tau_x * oldv2) / ndihandle.tau_x;
    l[1] = (newv1 - newr1 * newr1 * oldy + newr1 * ndihandle.tau_y * newu1 + olday2 * ndihandle.tau_y - 2 * newr1 * ndihandle.tau_y * oldu2) / ndihandle.tau_y;
    oldxed = oldu2 - newu1 + newr1 * oldy;
    oldyed = oldv2 - newv1 - newr1 * oldx;
#endif

    float v[2];
    v[0] = ndihandle.Kp * oldx + ndihandle.Kd * oldxed;
    v[1] = ndihandle.Kp * oldy + ndihandle.Kd * oldyed;

    float sig[2];
    sig[0] = v[0] - l[0];
    sig[1] = v[1] - l[1];

    float_mat_vect_mul(ndihandle.commands, _MINV, sig, 2, 2);
  }
}


/**
  * Set velocity in X Y Z for guided mode based on the NDI outputs.
  * This can be called from the flight plan when desired.
  */
// bool ndi_follow_leader(void)
// {
//   bool temp = true;
  
//   // Set horizontal speed X and Y
//   if  (stateGetPositionEnu_f()->z > 1.0) {
//     temp &= guidance_h_set_guided_vel(ndihandle.commands[0], ndihandle.commands[1]);
//   }

//   return !temp; // Exit false (for call in flight plan)
// }

OmniscientObserver *omn;
void ndi_follower::get_velocity_command(const uint8_t ID, float &vx_des, float &vy_des)
{
  // int32_t ac_id, float time, float range, float xin, float yin, float zin __attribute__((unused)), 
  // float u1in, float v1in, float u2in, float v2in, float gammain, float trackedAx, float trackedAy, float trackedYawr){
  
  // Store data from leader's position estimate
  if (ndihandle.data_entries == NDI_PAST_VALS) {
    ndihandle.data_entries--;
    ndihandle.data_start = (ndihandle.data_start + 1) % NDI_PAST_VALS;
  }
  ndihandle.xarr[ndihandle.data_end] = s[ID]->get_position(0);
  ndihandle.yarr[ndihandle.data_end] = s[ID]->get_position(1);
  ndihandle.u1arr[ndihandle.data_end] = s[ID]->get_state(2);
  ndihandle.v1arr[ndihandle.data_end] = s[ID]->get_state(3);
  ndihandle.u2arr[ndihandle.data_end] = s[0]->get_state(2);
  ndihandle.v2arr[ndihandle.data_end] = s[0]->get_state(3);
  ndihandle.r1arr[ndihandle.data_end] = s[ID]->get_state(7); //update_butterworth_2_low_pass(&uwb_butter_yawr,stateGetBodyRates_f()->r);
  ndihandle.r2arr[ndihandle.data_end] = s[0]->get_state(7);
  ndihandle.ax1arr[ndihandle.data_end] = s[ID]->get_state(4);
  ndihandle.ay1arr[ndihandle.data_end] = s[ID]->get_state(5);
  ndihandle.ax2arr[ndihandle.data_end] = s[0]->get_state(4);
  ndihandle.ay2arr[ndihandle.data_end] = s[0]->get_state(5);
  ndihandle.gamarr[ndihandle.data_end] = s[0]->get_state(6);
  ndihandle.tarr[ndihandle.data_end] = simtime_seconds;
  ndihandle.data_end = (ndihandle.data_end + 1) % NDI_PAST_VALS;
  ndihandle.data_entries++;
  
  uwb_follower_control_periodic();

  if (ID > 0){
  vx_des = ndihandle.commands[0];
  vy_des = ndihandle.commands[1];
  } 

  cout << vx_des << " " << vy_des << endl;
}
