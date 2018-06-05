#include "kill_functions.h"
#include "main.h"

void killer::restart_switch(int nagents, int knearest)
{
    stringstream ss;
    ss << "pkill swarmulator && ./swarmulator " << nagents << " " << knearest;
    system(ss.str().c_str());
}

void killer::kill_switch(void)
{
    program_running = false;
}