#include "kill_functions.h"

void killer::restart_switch(int nagents, int knearest) {
    stringstream ss;
    ss << "pkill swarmulator && ./swarmulator " << nagents << " " << knearest;
    system(ss.str().c_str());
}

void killer::kill_switch(void)
{
    stringstream ss;
    ss << "pkill swarmulator";
    system(ss.str().c_str());
}