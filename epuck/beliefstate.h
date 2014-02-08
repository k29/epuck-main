#ifndef BELIEFSTATE_H
#define BELIEFSTATE_H

#include "commondefs.h"
#define NUMBOTS 5

class BeliefState
{
public:
    BeliefState();
    Bot bot[NUMBOTS];

};

#endif // BELIEFSTATE_H
