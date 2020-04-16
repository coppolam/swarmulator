#ifndef DRAWINGPARAMS_H
#define DRAWINGPARAMS_H

extern float center_x, center_y; // Visualized center of x and y with respect to ground truth
extern float sx, sy; // Visualized transition
extern float zoom; // Current zoom value
extern float zoom_scale; // Scale of the zoom
extern float pointer_x, pointer_y;
extern float xrat; // Window ratio x
extern float yrat; // WIndow ratio y
extern bool paused; // Indicates if the simulation is paused at runtime
extern bool mouse_motion; // Is the mouse moving?

#endif /* DRAWINGPARAMS_H */
