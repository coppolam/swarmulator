#ifndef MOUSEFUNCTIONS_H
#define MOUSEFUNCTIONS_H

#include "main.h"

#include <thread>
#include <mutex>
#include "terminalinfo.h"
#include "agent_thread.h"

/** Compatibility with old GLUT for mapping the mouse wheel
 * http://iihm.imag.fr/blanch/software/glut-macosx/
 */
#if !defined(GLUT_WHEEL_UP)
#  define GLUT_WHEEL_UP    3 // Mouse wheel scrolled up
#  define GLUT_WHEEL_DOWN  4 // Mouse wheel scrolled down
#  define GLUT_WHEEL_LEFT  5 // Mouse wheel to the left
#  define GLUT_WHEEL_RIGHT 6 // Mouse wheel to the right
#endif

// Initialization of values declared in drawingparams.h
float center_x = 0;
float center_y = 0;
float sx = 0;
float sy = 0;
float zoom = 0;
float zoom_scale = 0;
float pointer_x, pointer_y;
bool paused = false;
float xrat = 1.0;
float yrat = 1.0;
bool mouse_motion = false;

/**
 * @brief keyboard_callback reads keyboard commands from the animation window of Swarmulator.
 *
 * This function reads keyboard commands from the animation window of Swarmulator
 * and handles the commands.
 * In this way, it becomes easies and intuitive to interact with the simulation of the swarm.
 * Functions include pausing, quitting, adding agents, zoom, etc.
 *
 * @param key The keyboard key that has been pressed
 * @param a (unused) Required by callback structure
 * @param b (unused) Required by callback structure
 */
void keyboard_callback(unsigned char key, __attribute__((unused)) int a, __attribute__((unused)) int b)
{
  terminalinfo ti; // Object to format runtime information on the terminal

  switch (key) {
    case 'c':
      // Center the animation
      ti.info_msg("Recentering Animation.");
      center_x = 0;
      center_y = 0;
      break;
    case 'z':
      // Reset the zoom to the default value
      ti.info_msg("Resetting zoom.");
      zoom = 0;
      break;
    case 'q':
      // End the simulation and quit
      ti.info_msg("Quitting Swarmulator.");
      mtx.try_lock();
      program_running = false;
      break;
    case 'p':
      // Pause the simulation
      if (!paused) {
        ti.info_msg("Paused. Press `r' to resume or `s' to step forward.");
        mtx.try_lock();
        paused = true;
      }
      break;
    case 'r':
      // Resume the simulation (if paused)
      if (paused) {
        ti.info_msg("Resuming.");
        mtx.unlock();
        paused = false;
      }
      s[0]->manual = false;
      break;
    case 's':
      // Step through the simulation. Very useful for debugging or analyzing what's going on.
      ti.info_msg("Stepping through. Press `s' to keep stepping forwrad to `r' to resume. ");
      int t_wait;
      mtx.try_lock();
      mtx.unlock();
      t_wait = (int)1e9 * (1.0 / (param->simulation_updatefreq() * param->simulation_realtimefactor()));
      this_thread::sleep_for(chrono::nanoseconds(t_wait));
      mtx.lock();
      paused = true;
      break;
    case 'a':
      // Draw and simulate a new agent, initialized at the current pointer position
      if (!paused) {
        ti.info_msg("Drawing new agent.");
        mtx.lock();
        create_new_agent(nagents, pointer_y, pointer_x);
        mtx.unlock();
        break;
      }
    case 'm':
      // Toggle the real time parameter between 1 and default, so as to better understand what's going on
      ti.info_msg("Toggle realtime factor between 1 and the specified value.");
      if (param->simulation_realtimefactor() != 1) {
        realtimefactor = param->simulation_realtimefactor();
        param->simulation_realtimefactor() = 1;
      } else {
        param->simulation_realtimefactor() = realtimefactor;
      }
      break;
    case '1':
      s[0]->manualpsi_delta = 0.1;
      break;
    case '2':
      s[0]->manualpsi_delta = -0.1;
      break;
    case 'n':
      // Quit and restart swarmulator
      mtx.try_lock();
      ti.info_msg("Restarting.");
      stringstream ss;
      ss << "pkill swarmulator && ./swarmulator " << nagents;
      system(ss.str().c_str());
      break;
  }

}
/**
 * Detects the mouse motion and adjusts the center of the animation
 *
 * @param x Pointer location in the animation window along x
 * @param y Pointer location in the animation window along y
 */
void mouse_motion_callback(int x, int y)
{
  if (mouse_motion) {
    center_x += param->mouse_drag_speed() / zoom_scale * ((float)x / (float)glutGet(GLUT_WINDOW_WIDTH) - sx);
    center_y += param->mouse_drag_speed() / zoom_scale * (-(float)y / (float)glutGet(GLUT_WINDOW_HEIGHT) - sy);
  }
}

/**
 * Keeps track of the location of the pointer.
 * This is used for launching new agents at specified locations intuitively.
 *
 * @param x Pointer location in the animation window along x
 * @param y Pointer location in the animation window along y
 */
void mouse_motion_callback_passive(int x, int y)
{
  pointer_x = ((float)x / (float)glutGet(GLUT_WINDOW_WIDTH) * 8. - 4.) / (zoom_scale * xrat) - center_x;
  pointer_y = -((float)y / (float)glutGet(GLUT_WINDOW_HEIGHT) * 8. - 4.) / (zoom_scale * yrat) - center_y;
}

/**
 * Detects that the mouse has been clicked (for dragging)
 * Or else detects that the zoom wheel is in use.
 *
 * @param button Detects which button has been pressed on the mouse
 * @param state State of the button (pressed on not)
 * @param x Pointer location in the animation window along x
 * @param y Pointer location in the animation window along y
 */
float wall_x_0, wall_y_0;
void mouse_click_callback(int button, int state, int x, int y)
{
  // Click - left
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    sx = (float)x / (float)glutGet(GLUT_WINDOW_WIDTH);
    sy = -(float)y / (float)glutGet(GLUT_WINDOW_HEIGHT);
    mouse_motion = true;
  }

  if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
    mouse_motion = false;
  }

  // Click - right
  if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
    wall_x_0 = pointer_x;
    wall_y_0 = pointer_y;
  }

  // Click - right
  if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP) {
    float wall_x_1 = ((float)x / (float)glutGet(GLUT_WINDOW_WIDTH) * 8. - 4.) / (zoom_scale * xrat) - center_x;
    float wall_y_1 = -((float)y / (float)glutGet(GLUT_WINDOW_HEIGHT) * 8. - 4.) / (zoom_scale * yrat) - center_y;
    environment.add(wall_x_0, wall_y_0, wall_x_1, wall_y_1);
  }

  // Zoom wheel
  if (button == GLUT_WHEEL_UP) {
    zoom +=  param->mouse_zoom_speed();
  } else if (button == GLUT_WHEEL_DOWN) {
    zoom += -param->mouse_zoom_speed();
  }

  // Guard on too much / too little zoom
  if (zoom > 9) {
    zoom = 9;
  } else if (zoom < -90) {
    zoom = -90;
  }
}

/**
 * @brief Allows to take control of the 0th agent with the keyboard arrows
 *
 * @param key Key being pressed
 * @param a (unused) Required by callback structure
 * @param b (unused) Required by callback structure
 */
void catchKey_arrow(int key, __attribute__((unused)) int a, __attribute__((unused)) int b)
{
  s[0]->manual = true;
  float vnominal = 1.0;
  if (key == GLUT_KEY_LEFT) {
    s[0]->manualy = -vnominal;
    s[0]->manualx = 0;
  } else if (key == GLUT_KEY_RIGHT) {
    s[0]->manualy = vnominal;
    s[0]->manualx = 0;
  } else if (key == GLUT_KEY_DOWN) {
    s[0]->manualx = -vnominal;
    s[0]->manualy = 0;
  } else if (key == GLUT_KEY_UP) {
    s[0]->manualx = vnominal;
    s[0]->manualy = 0;
  }
}

/**
 * @brief Removes control of the 0th agent with the keyboard arrows (key is up!)
 *
 * @param key Key being pressed
 * @param a (unused) Required by callback structure
 * @param b (unused) Required by callback structure
 */
void catckKey_arrow_up(int key, __attribute__((unused)) int a, __attribute__((unused)) int b)
{
  s[0]->manualx = 0;
  s[0]->manualy = 0;
}

void psi_callback_up(unsigned char key, int x, int y)
{
  s[0]->manualpsi_delta = 0;
}

/**
 * mouse_draganddrop handles the drag and drop functionality.
 */
void mouse_draganddrop()
{
  glutMotionFunc(mouse_motion_callback);
  glutPassiveMotionFunc(mouse_motion_callback_passive);
  glutMouseFunc(mouse_click_callback);
  glutSpecialFunc(catchKey_arrow);
  glutSpecialUpFunc(catckKey_arrow_up);
  glutKeyboardFunc(keyboard_callback);
  glutKeyboardUpFunc(psi_callback_up);
  zoom_scale = -(float)10 / (-(float)10 + (float)zoom);
  glTranslatef(center_x, center_y, -10 + zoom);
}

#endif /* MOUSEFUNCTIONS_H */
