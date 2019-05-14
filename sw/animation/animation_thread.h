#ifndef ANIMATION_THREAD_H
#define ANIMATION_THREAD_H

#include <GL/freeglut.h>

#include "main.h"
#include "draw.h"
#include "mousefunctions.h"
#include "terminalinfo.h"
#include "trigonometry.h"

bool animation_running = false;

/**
 * Main animation loop.
 * Takes care of drawing the agents in their corrective location.
 */
void main_loop_function()
{
  if (!animation_running) {
    terminalinfo ti;
    ti.info_msg("Animation started.");
    animation_running = true;
  }

  static draw drawer; // Drawer object

  // Add depth (used internally to block obstructed objects)
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  // Get current window size w.r.t. beginning
  xrat = (float)param->window_width() / (float)glutGet(GLUT_WINDOW_WIDTH);
  yrat = (float)param->window_height() / (float)glutGet(GLUT_WINDOW_HEIGHT);

  mouse_draganddrop(); // Activate mouse functions

  // Draw fixed one time objects
  drawer.draw_data(); // Put data in corner
  drawer.draw_axes(); // Put x and y global axes
#ifdef ARENAWALLS
  drawer.draw_walls(); // Draw the arena walls
#endif

  for (int i = 0; i < 3; i++) {
    drawer.draw_axes_text(i);
  }

  // Draw all agents
  for (uint8_t ID = 0; ID < nagents; ID++) {
    // Rotate local frame velocity (in state) to global frame
    drawer.draw_agent(ID,
                      s[ID]->state.at(0), // p_x global
                      s[ID]->state.at(1), // p_y global
                      s[ID]->state.at(6)); // orientation global
    drawer.draw_velocity_arrow(ID,
                               s[ID]->state.at(0), // p_x global
                               s[ID]->state.at(1), // p_y global
                               0.0, // p_z global
                               s[ID]->state.at(2),  // v_x global
                               s[ID]->state.at(3)); // v_y global
  }

  // Swap buffers (color buffers, makes previous render visible)
  glutSwapBuffers();

  // Wait until the next time-step according to the update frequency parameter
  int t_wait = (int) 1000.0 * (1.0 / param->animation_updatefreq());
  this_thread::sleep_for(chrono::milliseconds(t_wait));
}

/**
 * Initialze OpenGL perspective matrix
 */
void GL_Setup(int width, int height)
{
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glEnable(GL_DEPTH_TEST);
  gluPerspective(45, (float)width / height, .1, 100);
  glMatrixMode(GL_MODELVIEW);
}

/**
 * Thread function that initiates the animation
 */
void main_animation_thread()
{
  // Initialize all variables
  center_x = 0;
  center_y = 0;
  sx = 0;
  sy = 0;
#ifdef ARENAWALLS
  zoom = -ARENAWALLS;
#elif
  zoom = -10;
#endif // ARENAWALLS
  zoom_scale = 0;
  pointer_x = 0;
  pointer_y = 0;
  paused = false;
  xrat = 1.0;
  yrat = 1.0;

  // Set up simulation window
  int argc = 1;
  char *argv[1] = {(char *)"  "};
  glutInit(&argc, argv);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(param->window_width(), param->window_height());
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutCreateWindow("Swarmulator");
  glutIdleFunc(main_loop_function);
  GL_Setup(param->window_width(), param->window_height()); // Set up window parameters
  glutMainLoop(); // Initiate main drawing loop
}

#endif /*ANIMATION_THREAD_H*/
