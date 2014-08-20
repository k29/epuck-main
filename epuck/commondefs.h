#ifndef COMMONDEFS_H
#define COMMONDEFS_H

#include <opencv2/opencv.hpp>

//enum Command {FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN, STOP, SPEED1, SPEED2, SPEED3, SPEED4, F_PULSE, B_PULSE, L_PULSE, R_PULSE};
#define NUMBOTS 8
#define SIMULATION 1
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

class Circle
{
public:
    cv::Point2f centre;
    float radius;
};

#define BLOB_MIN_AREA 100


#endif
