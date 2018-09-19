#ifndef MOUSEFUNCTIONS_H
#define MOUSEFUNCTIONS_H

#include "main.h"

#include <thread>
#include <mutex>
#include "terminalinfo.h"
#include "agent_thread.h"

// A bit of a hack for compatibility with old GLUT
// http://iihm.imag.fr/blanch/software/glut-macosx/
#if !defined(GLUT_WHEEL_UP)
#  define GLUT_WHEEL_UP    3
#  define GLUT_WHEEL_DOWN  4
#  define GLUT_WHEEL_LEFT  5
#  define GLUT_WHEEL_RIGHT 6
#endif

float center_x = 0;
float center_y = 0;
float sx = 0;
float sy = 0;
float zoom = 0;
float zoom_scale = 0;
float pointer_x, pointer_y;
bool paused = false;
float xrat = 0;
float yrat = 0;

void keyboard_callback(unsigned char key, int x, int y)
{
  terminalinfo ti;

  switch (key) {
    case 'c':
      ti.info_msg("Recentering Animation.");
      center_x = 0;
      center_y = 0;
      break;
    case 'z':
      ti.info_msg("Resetting zoom.");
      zoom = 0;
      break;
    case 'q':
      ti.info_msg("Quitting Swarmulator.");
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
        ti.info_msg("Resuming.");
        mtx.unlock();
        paused = false;
        break;
      }
    case 's':
      ti.info_msg("Stepping through. Press `s' to keep stepping forwrad to `r' to resume. ");
      mtx.try_lock();
      mtx.unlock();
      this_thread::sleep_for(chrono::microseconds(1000));
      mtx.lock();
      paused = true;
      break;
    case 'a':
      if (!paused) {
        ti.info_msg("Drawing new agent.");
        mtx.lock(); // TODO: Change so that this creates a new agent regardless of agent type
        create_new_agent(nagents, pointer_x, pointer_y);
        mtx.unlock();
        break;
      }
    case 'm':
      ti.info_msg("Toggle realtime factor between 1 and the specified value.");
      if (param->simulation_realtimefactor() != 1) {
        realtimefactor = param->simulation_realtimefactor();
        param->simulation_realtimefactor() = 1;
      } else {
        param->simulation_realtimefactor() = realtimefactor;
      }
      break;
  }
}

void mouse_motion_callback(int x, int y)
{
  center_x += param->mouse_drag_speed() / zoom_scale * ((float)x / ((float)param->window_width() / xrat) - sx);
  center_y += param->mouse_drag_speed() / zoom_scale * (-(float)y / ((float)param->window_height() / yrat) - sy);
}

void mouse_motion_callback_passive(int x, int y)
{
  pointer_x = ((((float)x / ((float)param->window_width() / xrat)) * 8 / (zoom_scale * xrat)) - 4 / (zoom_scale * xrat)) - center_x;
  pointer_y = (-((((float)y / ((float)param->window_height() / yrat)) * 8 / (zoom_scale * yrat)) - 4 / (zoom_scale * yrat))) - center_y;
}

void mouse_click_callback(int button, int state, int x, int y)
{
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    sx =  (float)x / ((float)param->window_width () / xrat);
    sy = -(float)y / ((float)param->window_height() / yrat);
  }

  if (button == GLUT_WHEEL_UP) {
    zoom +=  param->mouse_zoom_speed();
  } else if (button == GLUT_WHEEL_DOWN) {
    zoom += -param->mouse_zoom_speed();
  }

  // Guard on too much / too little zoom
  if (zoom > 9)
  {
    zoom = 9;
  }
  else if (zoom < -90)
  {
    zoom = -90;
  }
}

void mouse_draganddrop()
{
  glutMotionFunc(mouse_motion_callback);
  glutPassiveMotionFunc(mouse_motion_callback_passive);
  glutMouseFunc(mouse_click_callback);
  glutKeyboardFunc(keyboard_callback);
  zoom_scale = -10 / (-10 + zoom);
  glTranslatef(center_x, center_y, -10 + zoom);
}

#endif /* MOUSEFUNCTIONS_H */