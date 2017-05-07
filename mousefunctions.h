#ifndef MOUSEFUNCTIONS_H
#define MOUSEFUNCTIONS_H
#include "main.h"
#include "parameters.h"

#include <thread>
#include <mutex>
#include <iostream>
#include <assert.h>     /* assert */

// A bit of a hack for compatibility with old GLUT
// http://iihm.imag.fr/blanch/software/glut-macosx/
#if !defined(GLUT_WHEEL_UP)
#  define GLUT_WHEEL_UP    3
#  define GLUT_WHEEL_DOWN  4
#  define GLUT_WHEEL_LEFT  5
#  define GLUT_WHEEL_RIGHT 6
#endif

float mx, my;
float sx = 0;
float sy = 0;
float zms = 0;
float zscale = 0;
float px, py;
void keyboard_callback(unsigned char key, int x, int y){
	cout << key << endl;
	switch(key){
		case 'c':
			mx = 0;
			my = 0;
			cout << endl << "Recentering Animation." << endl;
			break;
		case 'z':
			zms = 0;
			cout << endl << "Resetting zoom." << endl;
 			break;
		case 'q':
			cout << endl << "Quitting." << endl;
			exit(1);
		case 'p':
			cout << endl << "Pause. Press arrow keys to step forward " << endl;
			mtx.try_lock();
			break;
		case 'r':
			cout << endl << "Resume." << endl;
			mtx.unlock();
			break;
		case 's':
			cout << endl << "Stepping through." << endl;
			mtx.try_lock();
			mtx.unlock();
			this_thread::sleep_for(chrono::microseconds( 1000 ));
			mtx.lock();
			break;
		case 'a':
			vector<float> ns = { py, px, 0.0, 0.0};  // Initial positions/states
			
			mtx.lock();
			s.push_back(Particle(nagents,ns,1.0/simulation_updatefreq));
			nagents++;
			if ( knearest == nagents-2 )
			{
				knearest++;
			}
			mtx.unlock();
			break;
	}
}


void mouse_motion_callback(int x, int y){
	mx += mouse_drag_speed/zscale*( (float)x/window_width   - sx);
	my += mouse_drag_speed/zscale*(-(float)y/window_height  - sy);
}

void mouse_motion_callback_passive(int x, int y){
	px = (   (((float)x/window_width  )*8/zscale)-4/zscale )  - mx;
	py = ( -((((float)y/window_height )*8/zscale)-4/zscale )) - my;
}

void mouse_click_callback(int button, int state, int x, int y) {
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        sx =  (float)x/window_width;
        sy = -(float)y/window_height;
    }
    if (button == GLUT_WHEEL_UP)
    	zms += mouse_zoom_speed;
    else if (button == GLUT_WHEEL_DOWN)
    	zms += -mouse_zoom_speed;
}

void mouse_draganddrop(){
	glutMotionFunc(mouse_motion_callback);
	glutPassiveMotionFunc(mouse_motion_callback_passive);
	glutMouseFunc(mouse_click_callback);
	glutKeyboardFunc(keyboard_callback);
	zscale = -10/(-10+zms);
	glTranslatef(mx, my, -10+zms);
}

#endif /* MOUSEFUNCTIONS_H */