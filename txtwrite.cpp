#include "txtwrite.h"

void txtwrite::txtwrite_state(ofstream &logfile)
{
	for (int i = 0; i < nagents; ++i)	
	{
		logfile << i << " ";
		logfile << s[i].state.at(0) << " " << s[i].state.at(1) << endl;
	}
}