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
#include "ComputeDestination.hpp"
#include "commondefs.h"
#include "intersection.h"
#include <QDebug>
using namespace std;
#define infinity (1000000000)
#define pii pair<int,int>
#define EPS 1.0
#define pi 3.14159


pair<Position,Position > getCircleIntersection(const Circle &c1,const Circle &c2)
{
    double x0=c1.center.x;
    double y0=c1.center.y;
    double r0=c1.radius;

    double x1=c2.center.x;
    double y1=c2.center.y;
    double r1=c2.radius;

    double x3,y3,x3_prime,y3_prime;
    qDebug() << x0 << y0 << r0 << "printed";
    qDebug() << x1 << y1 << r1 << "printed";
//    x0 = y0 = 0;
//    x1 = 100;
//    y1 = 0;
//    r1 = r0 = 100;
    circle_circle_intersection(x0, y0, r0, x1, y1, r1,
                             &x3, &y3, &x3_prime, &y3_prime);

    pair<Position,Position> ans;

    ans.first.x=x3;
    ans.first.y=y3;

    ans.second.x=x3_prime;
    ans.second.y=y3_prime;

    return ans;
}

Position getRayCircleIntersection(const Circle &c,const Position& point1,const Position& point2)
{
    Position center=c.center;
    double radius=c.radius;
    Position rayStart=point2;
    Position rayEnd=point1;

    return lineSegmentCircleIntersect(center,radius,rayStart,rayEnd);

}


double distanceBetween(const Position &p,const Position &q)
{
	return sqrtl( (p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y) ); 
}
Position midPoint(const Position&p,const Position &q)
{
	Position n;
	n.x=(p.x+q.x)/2;
	n.y=(p.y+q.y)/2;
	return n;
}

/* If the circular area of radius 2 and centered at point p does not
contain the center of any other robot, then p is called a vacant point.*/
bool isVacantPoint(const Position &p, const vector<Robot> &robots, Robot r)
{
	int n=robots.size();	
	for(int i=0;i<n;++i)	
	{	
        if( distanceBetween(p,robots[i].currentPosition) <= 2.0*unit && (r.id != robots[i].id))
			return false;
	}
	return true;
}


Position getProjectionPoint(const Robot &r,const Circle &c2,const Circle &CIR)
{
	// double rad=r.visibilityRadius;
	// double theta=r.angle;
	Position rayStart=CIR.center;
	Position rayEnd=r.currentPosition;

	// Position rayEnd;
	// rayEnd.x=rayStart.x + (rad+rad)*cos(theta);
	// rayEnd.y=rayStart.y + (rad+rad)*sin(theta);

	return getRayCircleIntersection(c2,rayStart,rayEnd);
}

// Position getProjectionPoint(Robot &r,const Circle &c2)
// {
// 	Position rayStart=c2.center;
// 	Position rayEnd=r.currentPosition;
// 		return getRayCircleIntersection(c2,rayStart,rayEnd);
// }


double crossProduct(Position p1,Position p2)
{
	return p1.x*p2.y - p1.y*p2.x;
}
Position getRightIntersectionPoint(const Circle &c1,Robot& r)
{
	pair<Position,Position> intersection=getCircleIntersection(c1,r.getVisibilityCircle());
    qDebug() << "intersection " << intersection.first.x << " " << intersection.first.y;
    qDebug() << "intersection " << intersection.second.x << " " << intersection.second.y;
    qDebug() << "vis c" << r.getVisibilityCircle().center.x << " " << r.getVisibilityCircle().center.y;
	double angle=r.angle;

	Position normal;
	normal.x=cos(angle);
	normal.y=sin(angle);


	Position p1=intersection.first;
	Position p2=intersection.second;

	p1.x-=r.currentPosition.x;
	p1.y-=r.currentPosition.y;

	if( crossProduct(normal,p1) < 0)
		return intersection.first;
	else
		return intersection.second;
	
}

int getConfiguration(Robot r,Circle CIR)
{
	double distance=distanceBetween(r.currentPosition,CIR.center);
//    qDebug() << "distance = " << distance;
//    qDebug() << "circle radius = " << CIR.radius;
	/* If robot is on the circumference */
	if(distance >= CIR.radius - EPS) 
		return 1;

	/* It's VC just touches CIR */
    if(  fabs( distance - (CIR.radius-r.visibilityRadius) ) < EPS   )
		return 2;

	/* at centre exactly */
	if( distance < EPS )
		return 4;

	/* not touching */
	if( distance < CIR.radius-r.visibilityRadius)
		return 3;
	/* touching at 2 */
	return 5;	
}

Position ComputeDestination(const vector<Robot> &robots, Robot r ,const Circle &CIR)
{
	int configuration=getConfiguration(r,CIR);
    qDebug() << configuration;
	Position t,m,h;
	switch (configuration)
	{

		/* When ri is on the circumference of CIR */
		case 1:
			
			return r.currentPosition;
		
		break;

		/* When V C(ri) touches CIR (at some point say h) */
		case 2:
			
			h=getRightIntersectionPoint(CIR,r);
            if(isVacantPoint(h,robots, r))
				return h;
			else
				return midPoint(h,r.currentPosition);
			
		break;

		/* When ri is not at C and V C(ri) does not touch or intersect the circumference of CIR */
		case 3:
			t=getProjectionPoint(r,r.getVisibilityCircle(),CIR);
            if(isVacantPoint(t,robots,r))
				return t;
			else
				return midPoint(t,r.currentPosition);

		break;

		/* When ri is at C */
		case 4:

			/* projection of x -axis?? */
			m.x=r.visibilityRadius;
			m.y=0;

            if(isVacantPoint(m,robots,r))
				return m;
			else
				return midPoint(m,r.currentPosition);

		/* When V C(ri) intersects CIR (at two points say g and l) */
		case 5:
			Position l=getRightIntersectionPoint(CIR,r);
			Position t=getProjectionPoint(r,CIR,CIR);

            if(isVacantPoint(t,robots,r))
				return t;
			else
				{	

//                    qDebug() << l.x << " " << l.y;
					/* atan2 returns in -pi,pi */
                    double thetaInitial=atan2(t.y,t.x)+pi;
                    double thetaFinal=	atan2(l.y,l.x)+pi;
					double angularDistance=thetaFinal-thetaInitial;
					if(angularDistance<0)
                        angularDistance+= 2*pi;
					else if(angularDistance>=pi+pi)
                        angularDistance-= 2*pi;

					int numItr=angularDistance/0.02f;

					for(int i=0;i<numItr;++i)
					{

						double theta=thetaInitial + 0.02f*i;

						Position tempPoint;
                        tempPoint.x=CIR.center.x + CIR.radius*cos(theta);
                        tempPoint.y=CIR.center.y + CIR.radius*sin(theta);
						
                        if(isVacantPoint(tempPoint,robots,r))
							return tempPoint;
					}
                    qDebug() << "coming here";
						return midPoint(l,r.currentPosition);	
				}
			

	}

}
