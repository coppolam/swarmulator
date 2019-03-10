#ifndef NDI_FOLLOWER_H
#define NDI_FOLLOWER_H
#include "controller.h"

using namespace std;

#define NDI_PAST_VALS 200 // Store the last 200 values in order to compute the control

typedef struct ndihandler{
  float delay = 4;
  float tau_x=3;
  float tau_y=3;
  float wn_x=0.9;
  float wn_y=0.9;
  float eps_x=0.28;
  float eps_y=0.28;
  float Kp=-1.5;
  float Ki=0;
  float Kd=-3;
  float xarr[NDI_PAST_VALS];
  float yarr[NDI_PAST_VALS];
  float u1arr[NDI_PAST_VALS];
  float v1arr[NDI_PAST_VALS];
  float u2arr[NDI_PAST_VALS];
  float v2arr[NDI_PAST_VALS];
  float r1arr[NDI_PAST_VALS];
  float r2arr[NDI_PAST_VALS];
  float ax1arr[NDI_PAST_VALS];
  float ay1arr[NDI_PAST_VALS];
  float ax2arr[NDI_PAST_VALS];
  float ay2arr[NDI_PAST_VALS];
  float tarr[NDI_PAST_VALS];
  float gamarr[NDI_PAST_VALS];
  int data_start=0;
  int data_end=0;
  int data_entries=0;
  float commands[2];
  float commandscap[2];
  float maxcommand;
} ndihandler;

class ndi_follower: public Controller
{
  // The omniscient observer is used to simulate sensing the other agents.
  OmniscientObserver *o;


public:
  ndi_follower(): Controller() {};
  ~ndi_follower() {};

ndihandler ndihandle;
// Tuned gains, as used in experiments from paper described in pre-amble comments
 // ndihandler ndihandle = {delay: 4,
 //                        tau_x: 3,
 //                        tau_y: 3,
 //                        wn_x: 0.9,
 //                        wn_y: 0.9,
 //                        eps_x: 0.28,
 //                        eps_y: 0.28,
 //                        Kp: -1.5,
 //                        Ki: 0,
 //                        Kd: -3
                        // data_start = 0,
                        // data_end = 0,
                        // data_entries = 0,
                        // maxcommand = 1.5
                       // };
float accessCircularFloatArrElement(float arr[], int index);
float computeNdiFloatIntegral(float ndiarr[], float curtime);
void cleanNdiValues(float tcur);
bool hover_guided(float h);
bool ndi_follow_leader(void);
void uwb_follower_control_periodic(void);
virtual void get_velocity_command(const uint8_t ID, float &x_des, float &vy_des);
};

#endif /*NDI_FOLLOWER_H*/