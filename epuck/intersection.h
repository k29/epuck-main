#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <bits/stdc++.h>
#include "position.hpp"
#include "commondefs.h"
int circle_circle_intersection(double x0, double y0, double r0,
                               double x1, double y1, double r1,
                               double *xi, double *yi,
                               double *xi_prime, double *yi_prime)
{
  double a, dx, dy, d, h, rx, ry;
  double x2, y2;

  /* dx and dy are the vertical and horizontal distances between
   * the circle centers.
   */
  dx = x1 - x0;
  dy = y1 - y0;

  /* Determine the straight-line distance between the centers. */
  //d = sqrt((dy*dy) + (dx*dx));
  d = hypot(dx,dy); // Suggested by Keith Briggs

  /* Check for solvability. */
  if (d > (r0 + r1))
  {
    /* no solution. circles do not intersect. */
    return 0;
  }
  if (d < fabs(r0 - r1))
  {
    /* no solution. one circle is contained in the other */
    return 0;
  }

  /* 'point 2' is the point where the line through the circle
   * intersection points crosses the line between the circle
   * centers.  
   */

  /* Determine the distance from point 0 to point 2. */
  a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

  /* Determine the coordinates of point 2. */
  x2 = x0 + (dx * a/d);
  y2 = y0 + (dy * a/d);

  /* Determine the distance from point 2 to either of the
   * intersection points.
   */
  h = sqrt((r0*r0) - (a*a));

  /* Now determine the offsets of the intersection points from
   * point 2.
   */
  rx = -dy * (h/d);
  ry = dx * (h/d);

  /* Determine the absolute intersection points. */
  *xi = x2 + rx;
  *xi_prime = x2 - rx;
  *yi = y2 + ry;
  *yi_prime = y2 - ry;

  return 1;
}


Position lineSegmentCircleIntersect(Position centre, double radius, Position rayStart, Position rayEnd)
{
    double a = rayStart.x - rayEnd.x;
    double b = rayEnd.x - centre.x;
    double c = rayStart.y - rayEnd.y;
    double d = rayEnd.y - centre.y;

    double discriminantSquare = (2.*a*b + 2.*c*d)*(2.*a*b + 2.*c*d) - 4.*(a*a + c*c)*(b*b + d*d - radius*radius);
    if(discriminantSquare < 0)
    {
        Position p;
        p.x = -999;
        p.y = -999;
        return p;
    }
    double t1 = ((-(2.*a*b + 2.*c*d))+ sqrt(discriminantSquare))/(2.*(a*a + c*c));
    double t2 = ((-(2.*a*b + 2.*c*d))- sqrt(discriminantSquare))/(2.*(a*a + c*c));
    double t;
    if(t1 >= 0)
    {
        t = t1;
    }
    else if(t2 >=0)
    {
        t = t2;
    }
    else
    {
        Position p;
        p.x = -999;
        p.y = -999;
        return p;
    }

    Position p;
    // t = t2;
//    cout << t << endl;
    p.x = t*rayStart.x + (1.0 - t)*rayEnd.x;
    p.y = t*rayStart.y + (1.0 - t)*rayEnd.y;
    return p;
}

#define EPS 1e-5

struct XYZ
{
  double x,y,z;
};
/*
   Calculate the intersection of a ray and a sphere
   The line segment is defined from p1 to p2
   The sphere is of radius r and centered at sc
   There are potentially two points of intersection given by
   p = p1 + mu1 (p2 - p1)
   p = p1 + mu2 (p2 - p1)
   Return FALSE if the ray doesn't intersect the sphere.
*/
int RaySphere(XYZ p1,XYZ p2,XYZ sc,double r,double *mu1,double *mu2)
{
   double a,b,c;
   double bb4ac;
   XYZ dp;

   dp.x = p2.x - p1.x;
   dp.y = p2.y - p1.y;
   dp.z = p2.z - p1.z;
   a = dp.x * dp.x + dp.y * dp.y + dp.z * dp.z;
   b = 2 * (dp.x * (p1.x - sc.x) + dp.y * (p1.y - sc.y) + dp.z * (p1.z - sc.z));
   c = sc.x * sc.x + sc.y * sc.y + sc.z * sc.z;
   c += p1.x * p1.x + p1.y * p1.y + p1.z * p1.z;
   c -= 2 * (sc.x * p1.x + sc.y * p1.y + sc.z * p1.z);
   c -= r * r;
   bb4ac = b * b - 4 * a * c;
   if (fabs(a) < EPS || bb4ac < 0) {
      *mu1 = 0;
      *mu2 = 0;
      return(0);
   }

   *mu1 = (-b + sqrt(bb4ac)) / (2 * a);
   *mu2 = (-b - sqrt(bb4ac)) / (2 * a);

   return(1);
}
#undef EPS

#endif
