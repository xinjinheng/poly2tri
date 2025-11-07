/*
 * Poly2Tri Copyright (c) 2009-2018, Poly2Tri Contributors
 * https://github.com/jhasse/poly2tri
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

// Otherwise #defines like M_PI are undeclared under Visual Studio
#define _USE_MATH_DEFINES

#include "shapes.h"
#include "exceptions.h"

#include <cmath>
#include <exception>
#include <vector>

// C99 removes M_PI from math.h
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif

namespace p2t {

const double PI_3div4 = 3 * M_PI / 4;
const double PI_div2 = 1.57079632679489661923;
const double EPSILON = 1e-12;

enum Orientation { CW, CCW, COLLINEAR };

// Check if two points are equal with epsilon tolerance
inline bool PointsEqual(const Point& p1, const Point& p2) {
  return std::abs(p1.x - p2.x) < EPSILON && std::abs(p1.y - p2.y) < EPSILON;
}

// Check if two line segments intersect
// Returns true if segments intersect, false otherwise
// Segments are (p1, p2) and (p3, p4)
inline bool SegmentsIntersect(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
  double dx1 = p2.x - p1.x;
  double dy1 = p2.y - p1.y;
  double dx2 = p4.x - p3.x;
  double dy2 = p4.y - p3.y;
  double dx3 = p3.x - p1.x;
  double dy3 = p3.y - p1.y;
  
  double denominator = dy2 * dx1 - dx2 * dy1;
  if (std::abs(denominator) < EPSILON) {
    return false; // Collinear or parallel
  }
  
  double ua = (dx2 * dy3 - dy2 * dx3) / denominator;
  double ub = (dx1 * dy3 - dy1 * dx3) / denominator;
  
  return ua >= 0.0 && ua <= 1.0 && ub >= 0.0 && ub <= 1.0;
}

// Check if a point is inside a polygon using ray casting algorithm
// polygon: vector of points defining the polygon (must be closed)
// point: point to check
// Returns true if point is inside polygon, false otherwise
inline bool PointInPolygon(const std::vector<Point*>& polygon, const Point& point) {
  bool inside = false;
  size_t n = polygon.size();
  
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    if (((polygon[i]->y > point.y) != (polygon[j]->y > point.y)) &&
        (point.x < (polygon[j]->x - polygon[i]->x) * (point.y - polygon[i]->y) / (polygon[j]->y - polygon[i]->y) + polygon[i]->x)) {
      inside = !inside;
    }
  }
  
  return inside;
}

/**
 * Forumla to calculate signed area<br>
 * Positive if CCW<br>
 * Negative if CW<br>
 * 0 if collinear<br>
 * <pre>
 * A[P1,P2,P3]  =  (x1*y2 - y1*x2) + (x2*y3 - y2*x3) + (x3*y1 - y3*x1)
 *              =  (x1-x3)*(y2-y3) - (y1-y3)*(x2-x3)
 * </pre>
 */
Orientation Orient2d(const Point& pa, const Point& pb, const Point& pc)
{
  double detleft = (pa.x - pc.x) * (pb.y - pc.y);
  double detright = (pa.y - pc.y) * (pb.x - pc.x);
  double val = detleft - detright;

// Using a tolerance here fails on concave-by-subepsilon boundaries
//   if (val > -EPSILON && val < EPSILON) {
// Using == on double makes -Wfloat-equal warnings yell at us
  if (std::fpclassify(val) == FP_ZERO) {
    return COLLINEAR;
  } else if (val > 0) {
    return CCW;
  }
  return CW;
}

/*
bool InScanArea(Point& pa, Point& pb, Point& pc, Point& pd)
{
  double pdx = pd.x;
  double pdy = pd.y;
  double adx = pa.x - pdx;
  double ady = pa.y - pdy;
  double bdx = pb.x - pdx;
  double bdy = pb.y - pdy;

  double adxbdy = adx * bdy;
  double bdxady = bdx * ady;
  double oabd = adxbdy - bdxady;

  if (oabd <= EPSILON) {
    return false;
  }

  double cdx = pc.x - pdx;
  double cdy = pc.y - pdy;

  double cdxady = cdx * ady;
  double adxcdy = adx * cdy;
  double ocad = cdxady - adxcdy;

  if (ocad <= EPSILON) {
    return false;
  }

  return true;
}

*/

bool InScanArea(const Point& pa, const Point& pb, const Point& pc, const Point& pd)
{
  double oadb = (pa.x - pb.x)*(pd.y - pb.y) - (pd.x - pb.x)*(pa.y - pb.y);
  if (oadb >= -EPSILON) {
    return false;
  }

  double oadc = (pa.x - pc.x)*(pd.y - pc.y) - (pd.x - pc.x)*(pa.y - pc.y);
  if (oadc <= EPSILON) {
    return false;
  }
  return true;
}

}
