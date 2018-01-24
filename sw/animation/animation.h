#ifndef ANIMATION_H
#define ANIMATION_H

#include "main.h"
#include "glincludes.h"
#include "draw.h"
#include "mousefunctions.h"
#include "omniscient_observer.h"
#include "terminalinfo.h"

bool animation_running = false;
OmniscientObserver *obs = new OmniscientObserver();

/*
  Main animation loop.
  Takes care of drawing the agents in their corrective location.
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
  xrat = (float)window_width / (float)glutGet(GLUT_WINDOW_WIDTH);
  yrat = (float)window_height / (float)glutGet(GLUT_WINDOW_HEIGHT);

  mouse_draganddrop(); // Activate mouse functions

  // Draw fixed one time objects
  drawer.draw_data(); // Put data in corner
  drawer.draw_axes(); // Put x and y global axes
  for (int i = 0; i < 3; i++) {
    drawer.draw_axes_text(i);
  }

  // Draw all agents
  for (int i = 0; i < nagents; i++) {
    drawer.draw_agent(
      i,
      s[i].state.at(0),
      s[i].state.at(1),
      0.0);
    // TODO: Add orientation once model becomes more complex
  }

  // Draw agent sensor
  for (int i = 0; i < nagents; i++) {
    drawer.draw_agent(
      i,
      s[i].state.at(0),
      s[i].state.at(1),
      0.0);
    // TODO: Add orientation once model becomes more complex
  }

  // Draw velocity directions
  for (int i = 0; i < nagents; ++i) {
    drawer.draw_velocity_arrow(
        i,
        s[i].state.at(0),
        s[i].state.at(1),
        0.0,
        s[i].state.at(2), // v_x
        s[i].state.at(3)); // v_y
  }


  // Draw point at the center of mass of the swarm
  // if (visible_centroid) {
  //   drawer.draw_centroid(obs->get_centroid(0),obs->get_centroid(1),0.0);
  // }

  // Swap buffers (color buffers, makes previous render visible)
  glutSwapBuffers();

  // Wait until the next time-step according to the update frequency parameter
  int t_wait = (int) 1000.0 * (1.0 / animation_updatefreq);
  this_thread::sleep_for(chrono::milliseconds(t_wait));
}

/*
  Initialze OpenGL perspective matrix
*/
void GL_Setup(int width, int height)
{
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glEnable(GL_DEPTH_TEST);
  gluPerspective(45, (float)width / height, .1, 100);
  glMatrixMode(GL_MODELVIEW);
}

/*
  Thread function that initiates the animation
*/
void start_animation(int argc, char *argv[])
{
  // Initialize all variables
  mx = 0;
  my = 0;
  sx = 0;
  sy = 0;
  zms = 0;
  zscale = 0;
  px = 0;
  py = 0;
  paused = false;
  xrat = 0;
  yrat = 0;

  // Set up simulation window
  glutInit(&argc, argv);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(window_width, window_height);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutCreateWindow("Swarmulator");
  glutIdleFunc(main_loop_function);
  GL_Setup(window_width, window_height); // Set up window parameters

  glutSetIconTitle("./logo.ico");
  
#ifdef whitebackground
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);   // White background
#endif

  xrat = (float)window_width / (float)glutGet(GLUT_WINDOW_WIDTH);
  yrat = (float)window_height / (float)glutGet(GLUT_WINDOW_HEIGHT);

  glutMainLoop(); // Initiate main drawing loop
}

#endif /*ANIMATION_H*/