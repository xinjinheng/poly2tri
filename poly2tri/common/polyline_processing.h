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

#pragma once

#include "shapes.h"
#include "utils.h"

#include <vector>
#include <stdexcept>

namespace p2t {

class PolylineProcessing {
public:
  /// Remove duplicate points from a polyline
  static void RemoveDuplicatePoints(std::vector<Point*>& polyline);
  
  /// Process collinear points in a polyline
  static void ProcessCollinearPoints(std::vector<Point*>& polyline, bool keepEndpoints = true, double maxDistance = 0.0);
  
  /// Check if a polyline is self-intersecting
  static bool IsSelfIntersecting(const std::vector<Point*>& polyline);
  
  /// Validate a polyline (remove duplicates, process collinear points, check for self-intersections)
  static void ValidatePolyline(std::vector<Point*>& polyline, bool keepEndpoints = true, double maxDistance = 0.0);
  
private:
  /// Check if two points are considered duplicates (distance < EPSILON)
  static bool AreDuplicatePoints(const Point* p1, const Point* p2);
  
  /// Check if three points are collinear
  static bool AreCollinear(const Point* p1, const Point* p2, const Point* p3);
  
  /// Check if two edges intersect
  static bool EdgesIntersect(const Point* a1, const Point* a2, const Point* b1, const Point* b2);
};

} // namespace p2t