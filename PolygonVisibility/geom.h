//Zackery Leman & Ivy Xing

#ifndef __geom_h
#define __geom_h
#include <vector>
const double RADIUS = 9.0;
const double EPSILON = 0.001;



typedef struct _point2d {
  double x,y;
} point2D;


typedef struct _state {
     point2D location;
    double distance;
    std::vector<point2D> path;
} State;



typedef struct _segment2d {
  point2D start; 
  point2D end; 
} segment2D;

/* Return the signed area of triangle abc. The area is positive if c
   is to the left of ab, and negative if c is to the right of ab. */
double signed_area2D(point2D a, point2D b, point2D c);

/* Return 1 if p,q,r collinear, and 0 otherwise. */
int collinear(point2D p, point2D q, point2D r);

/* Return 1 if c is strictly left of ab; 0 otherwise. */
int left (point2D a, point2D b, point2D c);

/* Return 1 if c is between a and b; 0 otherwise. */
int between(point2D a, point2D b, point2D c);

/* Return 1 if s1 and s2 intersect. */
int intersect(segment2D s1, segment2D s2);

/* Return 1 if s1 and s2 intersect at a proper point (point that’s 
 interior to both). */
int intersect_proper (segment2D s1, segment2D s2);

/* Return 1 if  s1 and s2 intersect improperly, i.e. the intersection 
 point is the endpoint of one or both segments. */
int intersect_improper (segment2D s1,  segment2D s2);

/* Return the intersection point of the two lines. */
point2D computeIntersection(segment2D s1, segment2D s2);

/* Return 1 if the two doubles are within EPSILON of each other; 
 0 otherwise. */
double doubleEqual(double a, double b);

/* Return the distance between two points. */
double distance_(point2D a, point2D b);

/* Returns 1 if point a is within distance RADIUS of point b, 0 otherwise. */
int withinRadius(point2D a, point2D b);

/* Return the slope of a line. */
double slope(segment2D s);

#endif
