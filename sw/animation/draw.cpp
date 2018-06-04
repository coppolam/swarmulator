#include "draw.h"
#include <cmath>

// float rtfactor = simulation_realtimefactor;

void draw::draw_data()
{
  glRasterPos2f((-3.9 / zscale - mx), (-3.9 / zscale - my));
  glColor3f(0, 0, 1); // Red
  stringstream ss;
  ss << "Time[s]:" << simtime_seconds;
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)ss.str().c_str());
}

void draw::draw_axes_text(uint8_t dim)
{
  stringstream ss;

  if (dim == 0) {
    glRasterPos2f(3.9 / zscale - mx, 0.1 / zscale);
    ss << "y";
  } else if (dim == 1) {
    glRasterPos2f(0.1 / zscale, 3.9 / zscale - my);
    ss << "x";
  }

  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)ss.str().c_str());
}

void draw::draw_agent_number(uint8_t ID)
{
  glRasterPos2f(-0.01, 0.035);
#ifdef whitebackground
  glColor3f(0.0, 0.0, 0.0); // Black
#else
  glColor3f(1.0, 1.0, 1.0); // White
#endif

  stringstream ss;
  ss << (int)ID;
  glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char *)ss.str().c_str());
}

void draw::draw_triangle(double scl)
{
  glPushMatrix();

  glBegin(GL_POLYGON);
  glColor3ub(255, 000, 000); // Red
  glVertex2f(-0.3 * scl,  0.3 * scl);
  glVertex2f(-0.3 * scl, -0.3 * scl);
  glVertex2f(0.6 * scl,  0 * scl);
  glEnd();

  glColor3ub(255, 255, 255); // White
  glPopMatrix();
}

void draw::draw_circle(double d)
{
  double angle, x, y;
  glPushMatrix();
  glBegin(GL_POLYGON);
  glColor3ub(255, 000, 000); // Red
  for (int i = 0; i <= 100; i++) {
    angle = 2 * M_PI * i / 100;
    x = (d * yrat) * cos(angle);
    y = (d * xrat) * sin(angle);
    glVertex2d(x, y);
  }
  glEnd();

  glColor3ub(255, 255, 255); // White
  glPopMatrix();
}

void draw::draw_circle_loop(double d)
{
  int num_segments = 100;
  glPushMatrix();
  glLineWidth(1);
    glBegin(GL_LINE_LOOP);
    for(int ii = 0; ii < num_segments; ii++) {
      float theta = 2.0f * M_PI * float(ii) / float(num_segments);//get the current angle
      float x = 1.8 * yrat  * cosf(theta);//calculate the x component
      float y = 1.8 * xrat  * sinf(theta);//calculate the y component
      glVertex2d(x, y);
    }
    glEnd();

  glColor3ub(255, 255, 255); // White
  glPopMatrix();
}

void draw::draw_line(float x, float y)
{
  glPushMatrix();
  glLineWidth(2.5);
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(x, -y, 0);
  glEnd();
  glPopMatrix();
}


void draw::draw_point()
{
  glPointSize(10.0);
  glBegin(GL_POINTS);
  glVertex3f(0, 0, 0);
  glEnd();
}


void draw::draw_axes()
{
#ifndef whitebackground
  float lineintensity = 0.9;
#endif
  glLineWidth(2.5);
  
  glBegin(GL_LINES);
#ifdef whitebackground
  glColor3ub(0, 0, 0); // black
#else
  glColor3ub(255 * lineintensity, 255 * lineintensity, 255 * lineintensity); // white
#endif
  glVertex3f(-1000,  0.0, 0.0);
  glVertex3f(1000.0,  0.0, 0.0);
  glEnd();

  glBegin(GL_LINES);
#ifdef whitebackground
  glColor3ub(0, 0, 0); // black
#else
  glColor3ub(255 * lineintensity, 255 * lineintensity, 255 * lineintensity); // white
#endif
  glVertex3f(0.0, -1000.0, 0.0);
  glVertex3f(0.0,  1000.0, 0.0);
  glEnd();  
}

// TODO: Add different options dependending on agent type
void draw::draw_agent(uint8_t ID, float x, float y, float z)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, z); // ENU to NED
  glRotatef(90.0, 0.0, 0, 1);
  draw_circle(param->scale());
  draw_circle_loop(param->scale());
  draw_agent_number(ID);
  glPopMatrix();
}

void draw::draw_velocity_arrow(uint8_t ID, float x, float y, float z, float v_x, float v_y)
{
  glPushMatrix();
  glTranslatef(y * xrat, x * yrat, z); // ENU to NED
  glRotatef(90.0, 0.0, 0, 1);
  draw_line(v_x, v_y);
  glPopMatrix();
}

void draw::draw_centroid(float x, float y, float z)
{
  glPushMatrix();
  glTranslatef(y, x, z);
  glRotatef(90, 0.0, 0, 1);
  draw_point();
  glPopMatrix();
}