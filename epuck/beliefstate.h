#ifndef BELIEFSTATE_H
#define BELIEFSTATE_H

#include "commondefs.h"

class BeliefState
{
public:
    BeliefState();
    Bot bot[NUMBOTS];
    int node[NODE_ROWS][NODE_COLS];
};

#endif // BELIEFSTATE_H
