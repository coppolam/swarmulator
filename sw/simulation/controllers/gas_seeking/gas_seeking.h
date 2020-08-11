#ifndef GAS_SEEKING_H
#define GAS_SEEKING_H

#include <stdio.h>
#include <iostream>
#include "controller.h"

class gas_seeking: public Controller
{
public:
	gas_seeking():Controller(){};
	virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
	virtual void animation(const uint16_t ID);
};

#endif /*GAS_SEEKING_H*/
