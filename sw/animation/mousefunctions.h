#ifndef MOUSEFUNCTIONS_H
#define MOUSEFUNCTIONS_H

#include "main.h"

#include <thread>
#include <mutex>
#include <iostream>
#include <assert.h>     /* assert */
#include "terminalinfo.h"
#include "agentthread.h"

// A bit of a hack for compatibility with old GLUT
// http://iihm.imag.fr/blanch/software/glut-macosx/
#if !defined(GLUT_WHEEL_UP)
#  define GLUT_WHEEL_UP    3
#  define GLUT_WHEEL_DOWN  4
#  define GLUT_WHEEL_LEFT  5
#  define GLUT_WHEEL_RIGHT 6
#endif

float mx = 0;
float my = 0;
float sx = 0;
float sy = 0;
float zms = 0;
float zscale = 0;
float px, py;
bool paused = false;
float xrat = 0;
float yrat = 0;
float rtfactor = param->simulation_realtimefactor();

void keyboard_callback(unsigned char key, int x, int y)
{
  terminalinfo ti;

  stringstream ss;
  switch (key) {
    case 'c':
      mx = 0;
      my = 0;
      ti.info_msg("Recentering Animation.");
      break;
    case 'n':
      mtx.try_lock();
      ti.info_msg("Restarting.");
      ss << "pkill swarmulator && ./swarmulator " << nagents << " " << knearest;
      system(ss.str().c_str());
      break;
    case 'z':
      zms = 0;
      ti.info_msg("Resetting zoom.");
      break;
    case 'q':
      ti.info_msg("Quitting.");
      mtx.try_lock();
      program_running = false;
      break;
    case 'p':
      if (!paused) {
        ti.info_msg("Paused. Press `r' to resume or `s' to step forward.");
        mtx.try_lock();
        paused = true;
      }
      break;
    case 'r':
      if (paused) {
        ti.info_msg("Resume.");
        mtx.unlock();
        paused = false;
        break;
      }
    case 's':
      ti.info_msg("Stepping through.");
      mtx.try_lock();
      mtx.unlock();
      this_thread::sleep_for(chrono::microseconds(1000));
      mtx.lock();
      paused = true;
      break;
    case 'a':
      if (!paused) {
        vector<float> ns = { py, px, 0.0, 0.0, 0.0, 0.0 }; // Initial positions/states
        mtx.lock();
        s.push_back(Particle(nagents, ns, 1.0 / param->simulation_updatefreq()));
        thread agent(start_agent_simulation, nagents);
        agent.detach();
        nagents++;
        if (knearest == nagents - 2) {
          knearest++;
        }
        mtx.unlock();
        ti.info_msg("Drawing new agent.");
        break;
      }
    case 'm':
    if (param->simulation_realtimefactor() == 1)
      param->simulation_realtimefactor() = rtfactor;
    else
      param->simulation_realtimefactor() = 1;
    break;
  }
}

void mouse_motion_callback(int x, int y)
{
  mx += param->mouse_drag_speed() / zscale * ((float)x / ((float)param->window_width() / xrat) - sx);
  my += param->mouse_drag_speed() / zscale * (-(float)y / ((float)param->window_height() / yrat) - sy);
}

void mouse_motion_callback_passive(int x, int y)
{
  px = ((((float)x / ((float)param->window_width() / xrat)) * 8 / (zscale * xrat)) - 4 / (zscale * xrat)) - mx;
  py = (-((((float)y / ((float)param->window_height() / yrat)) * 8 / (zscale * yrat)) - 4 / (zscale * yrat))) - my;
}

void mouse_click_callback(int button, int state, int x, int y)
{
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    sx = (float)x / ((float)param->window_width() / xrat);
    sy = -(float)y / ((float)param->window_height() / yrat);
  }

  if (button == GLUT_WHEEL_UP) {
    zms += param->mouse_zoom_speed();
  } else if (button == GLUT_WHEEL_DOWN) {
    zms += -param->mouse_zoom_speed();
  }

  // Guard on too much / too little zoom
  if (zms > 9) {
    zms = 9;
  } else if (zms < -90) {
    zms = -90;
  }
}

void mouse_draganddrop()
{
  glutMotionFunc(mouse_motion_callback);
  glutPassiveMotionFunc(mouse_motion_callback_passive);
  glutMouseFunc(mouse_click_callback);
  glutKeyboardFunc(keyboard_callback);
  zscale = -10 / (-10 + zms);
  glTranslatef(mx, my, -10 + zms);
}

#endif /* MOUSEFUNCTIONS_H */