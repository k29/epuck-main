#ifndef BELIEFSTATE_H
#define BELIEFSTATE_H

#include "commondefs.h"

class Cell_MDFS
{
public:
    enum State{VISITED, UNEXPLORED, EXPLORED, WALL};
    enum Direction {UP, DOWN, LEFT, RIGHT};
    State state;
    Direction direction;
    std::set <int> bot_ids;

};

class BeliefState
{
public:
    int current_algo;
    BeliefState();
    Bot bot[NUMBOTS];
    int node[NODE_ROWS][NODE_COLS];
    Cell_MDFS node_MDFS[NODE_ROWS][NODE_COLS];
};

#endif // BELIEFSTATE_H
