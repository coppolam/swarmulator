#ifndef SETTINGS_H
#define SETTINGS_H

#ifndef ANIMATE // Activate animation thread
#define ANIMATE
#endif

#ifndef LOG // Activate logger thread
#define LOG 
#endif

#ifndef CONTROLLER
// #define CONTROLLER Controller_Cartesian
#define CONTROLLER Controller_Bearing_Shape
// #define CONTROLLER Controller_Aggregate
// #define CONTROLLER Controller_Bearing
// #define CONTROLLER Controller_Keep_Aggregate
#endif

#ifndef CONTROLLER_INCLUDE
// #define CONTROLLER_INCLUDE "controller_cartesian.h"
#define CONTROLLER_INCLUDE "controller_bearing_shape.h"
// #define CONTROLLER_INCLUDE "controller_aggregate.h"
// #define CONTROLLER_INCLUDE "controller_bearing.h"
// #define CONTROLLER_INCLUDE "controller_keep_aggregate.h"
#endif

#endif /*SETTINGS_H*/