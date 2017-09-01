#include "draw.h"
#include <cmath>
// Options
// GL_POINTS,
// GL_LINES,
// GL_LINE_STRIP,
// GL_LINE_LOOP,
// GL_TRIANGLES,
// GL_TRIANGLE_STRIP,
// GL_TRIANGLE_FAN,
// GL_QUADS,
// GL_QUAD_STRIP, and
// GL_POLYGON.

// TODO: Make the position relative to zoom.
void draw::draw_data()
{
	glRasterPos2f((-3.9/zscale-mx), (-3.9/zscale-my));
	glColor3f(0,0,1); // Red 
	stringstream ss;
	ss << "Time[s]:" << simulation_realtimefactor*simulation_time/1000000.0;
	glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char*)ss.str().c_str());

}

// TODO: Make the position relative to zoom.
void draw::draw_axes_text(int dim) {

	stringstream ss;

	if (dim == 0)
	{
		glRasterPos2f(3.9/zscale-mx, 0.1/zscale);
		ss << "y";
	}
	else if (dim == 1)
	{
		glRasterPos2f(0.1/zscale, 3.9/zscale-my);
		ss << "x";
	}
	glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char*)ss.str().c_str());

}

void draw::draw_agent_number(int ID) {

	glRasterPos2f(-0.01, 0.035);
	#ifdef whitebackground
	glColor3f(0.0,0.0,0.0); // Black 
	#else
	glColor3f(1.0,1.0,1.0); // White
	#endif

	stringstream ss;
	ss << ID;
	glutBitmapString(GLUT_BITMAP_8_BY_13, (unsigned char*)ss.str().c_str());

}

void draw::draw_triangle(double scl) {
	glPushMatrix();

	glBegin(GL_POLYGON);
	glColor3ub(255, 000, 000); // Red 
	glVertex2f(-0.3*scl,  0.3*scl);
	glVertex2f(-0.3*scl, -0.3*scl);
	glVertex2f( 0.6*scl,  0*scl);
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
	for(int i =0; i <= 100; i++)
	{
		angle = 2 * M_PI * i / 100;
		x = (d*yrat)/2.0*cos(angle);
		y = (d*xrat)/2.0*sin(angle);
		glVertex2d(x,y);
	}
	glEnd(); 

	glColor3ub(255, 255, 255); // White 
	glPopMatrix();

}

void draw::draw_point(){

	glPointSize(10.0);
	glBegin(GL_POINTS);
	glVertex3f(0, 0, 0);
	glEnd();
}


void draw::draw_axes() {
	#ifndef whitebackground
	float lineintensity = 0.9;
	#endif
	
	glLineWidth(2.5); 
	glBegin(GL_LINES); 
	#ifdef whitebackground
	glColor3ub(0, 0, 0); // black 
	#else
	glColor3ub(255*lineintensity, 255*lineintensity, 255*lineintensity); // white 
	#endif

	glVertex3f(-1000,  0.0, 0.0);
	glVertex3f(1000.0,  0.0, 0.0);
	glEnd();
	glBegin(GL_LINES); 
	#ifdef whitebackground
	glColor3ub(0, 0, 0); // black 
	#else
	glColor3ub(255*lineintensity, 255*lineintensity, 255*lineintensity); // white 
	#endif

	glVertex3f(0.0, -1000.0, 0.0);
	glVertex3f(0.0,  1000.0, 0.0);
	glEnd();
}

// TODO: Add different options dependending on agent type
void draw::draw_agent(int ID, float x, float y, float z)
{
	glPushMatrix();
	glTranslatef(y*xrat,x*yrat,z); // ENU to NED
	glRotatef(90.0, 0.0, 0, 1);
	draw_circle(scale);
	draw_agent_number(ID);
	glPopMatrix();
}


void draw::draw_centroid(float x, float y, float z)
{
	glPushMatrix();
	glTranslatef(y,x,z);
	glRotatef(90, 0.0, 0, 1);
	draw_point();
	glPopMatrix();
}