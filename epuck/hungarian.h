#ifndef HUNGARIAN_H
#define HUNGARIAN_H
#include <cstdio>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <set>
#include <cassert>
#include <map>
#include <set>
#include <queue>
#include <stack>
#include <cmath>
#include <iomanip>
#include <cstring>
//using namespace std;
#define infinity (1000000000)
#define pii pair<int,int>

#define N_HUNGARIAN 55             //max number of vertices in one part
#define INF 100000000    //just infinity

class Hungarian
{
private:

    int lx[N_HUNGARIAN], ly[N_HUNGARIAN];        //labels of X and Y parts
    int yx[N_HUNGARIAN];               //yx[y] - vertex that is matched with y
    bool S[N_HUNGARIAN], T[N_HUNGARIAN];         //sets S and T in algorithm
    int slack[N_HUNGARIAN];            //as in the algorithm description
    int slackx[N_HUNGARIAN];           //slackx[y] such a vertex, that
                             // l(slackx[y]) + l(y) - w(slackx[y],y) = slack[y]
    int prev[N_HUNGARIAN];             //array for memorizing alternating paths

public:
    int cost[N_HUNGARIAN][N_HUNGARIAN];          //cost matrix
    int xy[N_HUNGARIAN];               //xy[x] - vertex that is matched with x,
    int n, max_match;        //n workers and n jobs
    void update_labels();
    void add_to_tree(int x, int prevx);
    void init_labels();
    void augment();
    int hungarian();

};

#endif // HUNGARIAN_H
