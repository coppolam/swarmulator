#ifndef DRAWINGPARAMS_H
#define DRAWINGPARAMS_H

/**
 * This file holds several parameters needed by the animation
 */

extern float center_x, center_y; // Visualized center of x and y with respect to ground truth
extern float sx, sy; // Visualized transition
extern float zoom, zoom_scale; // Current zoom value + scale of the zoom
extern float pointer_x, pointer_y; // Where the cursor is located along x and y
extern float xrat, yrat; // Window ratio along x and y
extern bool paused; // Indicates if the simulation is paused
extern bool mouse_motion; // Indicates if the mouse is moving

#endif /* DRAWINGPARAMS_H */
