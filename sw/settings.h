#ifndef SETTINGS_H
#define SETTINGS_H

#ifndef ANIMATE // Activate animation thread
#define ANIMATE
#endif

#ifndef LOG // Activate logger thread
#define LOG 
#endif

#ifndef CONTROLLER
#define CONTROLLER Controller_Aggregate
#endif

#ifndef CONTROLLER_INCLUDE
#define CONTROLLER_INCLUDE "controller_aggregate.h"
#endif

#endif /*SETTINGS_H*/