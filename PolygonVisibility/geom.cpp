//Zackery Leman & Ivy Xing

#include "geom.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Return the signed area of triangle abc. The area is positive if c
//   is to the left of ab, and negative if c is to the right of ab.
double signed_area2D(point2D a, point2D b, point2D c) {
  return ((b.x - a.x) * (c.y - a.y)) -
        ((c.x - a.x) * (b.y - a.y));
}

// Return 1 if p,q,r collinear, and 0 otherwise.
int collinear(point2D p, point2D q, point2D r) {
  return doubleEqual(signed_area2D(p, q, r), 0);
}

// Return 1 if c is  strictly left of ab; 0 otherwise.
int left (point2D a, point2D b, point2D c) {
  return signed_area2D(a, b, c) > 0;
}

// Check whether point c is between a and b.
int between(point2D a, point2D b, point2D c) {
    if (!collinear(a, b, c)) {
        return 0;
    }
    if (!doubleEqual(a.x, b.x)) {
        return ((a.x <= c.x) && (c.x <= b.x)) ||
        ((a.x >= c.x) && (c.x >= b.x));
    } else {
        return ((a.y <= c.y) && (c.y <= b.y)) ||
        ((a.y >= c.y) && (c.y >= b.y));
    }
}

// Exclusive or.
int Xor(int x, int y) {
    return !x ^ !y;
}

// Return 1 if s1 and s2 intersect at a proper point (point that’s interior to both).
int intersect_proper (segment2D s1, segment2D s2) {
    if (collinear(s1.start, s1.end, s2.start) ||
        collinear(s1.start, s1.end, s2.end) ||
        collinear(s2.start, s2.end, s1.start) ||
        collinear(s2.start, s2.end, s1.end)) {
        return 0;
    }
    
    return Xor(left(s1.start, s1.end, s2.start), left(s1.start, s1.end, s2.end)) &&
    Xor(left(s2.start, s2.end, s1.start), left(s2.start, s2.end, s1.end));
}

// Return 1 if s1 and s2 intersect.
int intersect(segment2D s1, segment2D s2) {
    if (intersect_proper(s1, s2)) {
        return 1;
    } else if ( between(s1.start, s1.end, s2.start) ||
               between(s1.start, s1.end, s2.end) ||
               between(s2.start, s2.end, s1.start) ||
               between(s2.start, s2.end, s1.end)) {
        return 1;
    } else {
        return 0;
    }
}

// Return 1 if  s1 and s2 intersect improperly, i.e.
// the intersection point is the endpoint of one or both segments.
int intersect_improper (segment2D s1,  segment2D s2) {
    if (intersect(s1, s2) && intersect_proper(s1, s2) == 0) {
        return 1;
    } else {
        return 0;
    }
}

// Return the intersection point of the two lines.
point2D computeIntersection(segment2D s1, segment2D s2) {
    point2D intersection;
    // Store the values for fast access and easy equations-to-code conversion.
    double x1 = s1.start.x, x2 = s1.end.x, x3 = s2.start.x, x4 = s2.end.x;
    double y1 = s1.start.y, y2 = s1.end.y, y3 = s2.start.y, y4 = s2.end.y;
    
    double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    
    // Get the x and y.
    double pre = (x1 * y2 - y1 * x2), post = (x3 * y4 - y3 * x4);
    double x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
    double y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;
    
    // Return the point of intersection.
    intersection.x = x;
    intersection.y = y;
    return intersection;
}

// Return the distance between two points.
double distance_(point2D a, point2D b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Return 1 if point a is within distance RADIUS of point b, 0 otherwise.
int withinRadius(point2D a, point2D b) {
    if (distance_(a, b) <= RADIUS) {
        return 1;
    }
    return 0;
}

// Return the slope of a line.
double slope(segment2D s) {
    point2D first = s.start;
    point2D second = s.end;
    return ((first.y - second.y) / (first.x - second.x));
}

// Return 1 if the two doubles are within EPSILON of each other; 0 otherwise.
double doubleEqual(double a, double b) {
    if (fabs(a - b) < EPSILON) {
        return 1;
    }
    return 0;
}

