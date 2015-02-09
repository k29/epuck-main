#ifndef COMMONDEFS_H
#define COMMONDEFS_H

#include <opencv2/opencv.hpp>

//enum Command {FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN, STOP, SPEED1, SPEED2, SPEED3, SPEED4, F_PULSE, B_PULSE, L_PULSE, R_PULSE};
#define NUMBOTS 5
#define SIMULATION 1
#define NODE_ROWS 20
#define NODE_COLS 20
//plz to store angle in radians
struct Bot
{
	double x;
	double y;
	double angle;
    bool isVisible;
};

class PointList
{
public:
    CvPoint p[NUMBOTS];
};


#define BLOB_MIN_AREA 100
#define unit (10)

struct Position
{
    double x;
    double y;
};

struct Circle
{
    Position center;
    double radius;
    Circle()
    {

    }
    Circle(Position center,double radius)
    {
        this->center=center;

        this->radius=radius;
    }
};


struct Robot
{

    /* insert parameters here */

    /* visibility range */
    double visibilityRadius;
    double width;
    double angle;
    int id;
    Position currentPosition;

    Robot()
    {
        visibilityRadius = 2*unit;
        width=unit;
    }

    Circle getVisibilityCircle()
    {
        return Circle(currentPosition,visibilityRadius);
    }
};




#endif
