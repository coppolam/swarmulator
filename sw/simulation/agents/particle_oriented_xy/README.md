# particle_oriented_xy

Basic accelerated particle with kinematic equations and orientation.

Is represented as a triagle in the direction of the orientation.

Unlike the standard `particle_oriented` class, this one expects the control input to be velocities along x and y in the local body frame (i.e. forward and sideways), whereas the original expects the control inputs to be the velocity along x and the yaw rate.