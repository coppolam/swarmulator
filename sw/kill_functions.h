#ifndef KILL_FUNCTIONS_H
#define KILL_FUNCTIONS_H

#include <iostream>

void restart_switch (int nagents, int knearest) {
    stringstream ss;
    ss << "pkill swarmulator && ./swarmulator " << nagents << " " << knearest;
    system(ss.str().c_str());
}

void kill_switch(int nagents, int knearest)
{
    stringstream ss;
    ss << "pkill swarmulator";
    system(ss.str().c_str());
}

#endif /* KILL_FUNCTIONS_H */