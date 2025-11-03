/*
 * Poly2Tri Copyright (c) 2009-2023, Poly2Tri Contributors
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

#include "polyline_processing.h"

#include <algorithm>
#include <cmath>

namespace p2t {

void PolylineProcessing::RemoveDuplicatePoints(std::vector<Point*>& polyline) {
  if (polyline.size() < 2) {
    return;
  }
  
  std::vector<Point*> result;
  result.push_back(polyline[0]);
  
  for (size_t i = 1; i < polyline.size(); ++i) {
    if (!AreDuplicatePoints(result.back(), polyline[i])) {
      result.push_back(polyline[i]);
    }
  }
  
  // If all points were duplicates, keep at least one
  if (result.empty() && !polyline.empty()) {
    result.push_back(polyline[0]);
  }
  
  polyline.swap(result);
}

void PolylineProcessing::ProcessCollinearPoints(std::vector<Point*>& polyline, bool keepEndpoints, double maxDistance) {
  if (polyline.size() < 3) {
    return;
  }
  
  std::vector<Point*> result;
  result.push_back(polyline[0]);
  
  for (size_t i = 1; i < polyline.size() - 1; ++i) {
    if (!AreCollinear(result.back(), polyline[i], polyline[i+1])) {
      result.push_back(polyline[i]);
    } else if (maxDistance > 0.0) {
      // Calculate distance from previous point to current point
      double dx = polyline[i]->x - result.back()->x;
      double dy = polyline[i]->y - result.back()->y;
      double distance = std::sqrt(dx*dx + dy*dy);
      
      if (distance >= maxDistance) {
        result.push_back(polyline[i]);
      }
    }
  }
  
  if (!polyline.empty()) {
    result.push_back(polyline.back());
  }
  
  polyline.swap(result);
}

bool PolylineProcessing::IsSelfIntersecting(const std::vector<Point*>& polyline) {
  size_t n = polyline.size();
  if (n < 4) {
    return false;
  }
  
  for (size_t i = 0; i < n; ++i) {
    size_t j = (i + 1) % n;
    const Point* a1 = polyline[i];
    const Point* a2 = polyline[j];
    
    for (size_t k = i + 2; k < n; ++k) {
      size_t l = (k + 1) % n;
      if (l == i) {
        continue;
      }
      
      const Point* b1 = polyline[k];
      const Point* b2 = polyline[l];
      
      if (EdgesIntersect(a1, a2, b1, b2)) {
        return true;
      }
    }
  }
  
  return false;
}

void PolylineProcessing::ValidatePolyline(std::vector<Point*>& polyline, bool keepEndpoints, double maxDistance) {
  RemoveDuplicatePoints(polyline);
  
  if (polyline.size() < 3) {
    throw std::runtime_error("Polyline must have at least 3 non-duplicate points");
  }
  
  ProcessCollinearPoints(polyline, keepEndpoints, maxDistance);
  
  if (polyline.size() < 3) {
    throw std::runtime_error("Polyline must have at least 3 non-collinear points");
  }
  
  if (IsSelfIntersecting(polyline)) {
    throw std::runtime_error("Polyline is self-intersecting");
  }
}

bool PolylineProcessing::AreDuplicatePoints(const Point* p1, const Point* p2) {
  double dx = p1->x - p2->x;
  double dy = p1->y - p2->y;
  double dz = p1->z - p2->z;
  return (dx*dx + dy*dy + dz*dz) < EPSILON;
}

bool PolylineProcessing::AreCollinear(const Point* p1, const Point* p2, const Point* p3) {
  return Orient2d(*p1, *p2, *p3) == COLLINEAR;
}

bool PolylineProcessing::EdgesIntersect(const Point* a1, const Point* a2, const Point* b1, const Point* b2) {
  // Check if any of the points are the same
  if (AreDuplicatePoints(a1, b1) || AreDuplicatePoints(a1, b2) ||
      AreDuplicatePoints(a2, b1) || AreDuplicatePoints(a2, b2)) {
    return false;
  }
  
  // Calculate orientations
  Orientation o1 = Orient2d(*a1, *a2, *b1);
  Orientation o2 = Orient2d(*a1, *a2, *b2);
  Orientation o3 = Orient2d(*b1, *b2, *a1);
  Orientation o4 = Orient2d(*b1, *b2, *a2);
  
  // General case - edges cross each other
  if ((o1 != o2 && o3 != o4) &&
      (o1 != COLLINEAR && o2 != COLLINEAR && o3 != COLLINEAR && o4 != COLLINEAR)) {
    return true;
  }
  
  // Special cases - edges are collinear and overlapping
  if (o1 == COLLINEAR && ArePointOnSegment(*b1, *a1, *a2)) {
    return true;
  }
  
  if (o2 == COLLINEAR && ArePointOnSegment(*b2, *a1, *a2)) {
    return true;
  }
  
  if (o3 == COLLINEAR && ArePointOnSegment(*a1, *b1, *b2)) {
    return true;
  }
  
  if (o4 == COLLINEAR && ArePointOnSegment(*a2, *b1, *b2)) {
    return true;
  }
  
  return false;
}

// Helper function to check if a point is on a line segment
bool PolylineProcessing::ArePointOnSegment(const Point& p, const Point& a, const Point& b) {
  if (Orient2d(p, a, b) != COLLINEAR) {
    return false;
  }
  
  // Check if point is within bounding box of segment
  double minX = std::min(a.x, b.x);
  double maxX = std::max(a.x, b.x);
  double minY = std::min(a.y, b.y);
  double maxY = std::max(a.y, b.y);
  
  return (p.x >= minX - EPSILON && p.x <= maxX + EPSILON &&
          p.y >= minY - EPSILON && p.y <= maxY + EPSILON);
}

} // namespace p2t