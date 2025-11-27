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

#include <vector>
#include <unordered_set>
#include "shapes.h"
#include "exceptions.h"

namespace p2t {

class P2T_DLL_SYMBOL InputValidator {
public:
  /// Constructor with default epsilon
  InputValidator() : epsilon_(1e-8) {}
  
  /// Constructor with custom epsilon
  explicit InputValidator(double epsilon) : epsilon_(epsilon) {
    if (epsilon <= 0.0 || epsilon >= 1.0) {
      throw InvalidInputException::Create("epsilon", epsilon);
    }
  }
  
  /// Check if a point is valid (not NaN or infinite)
  bool IsValidPoint(const Point* point) const;
  
  /// Check if a point is valid (not NaN or infinite)
  bool IsValidPoint(const Point& point) const;
  
  /// Check for duplicate points in a vector
  void CheckDuplicatePoints(const std::vector<Point*>& points) const;
  
  /// Check if a polygon is valid (at least 3 points, no duplicates, closed, not self-intersecting)
  void CheckPolygonValidity(const std::vector<Point*>& polygon) const;
  
  /// Check if a polygon is closed (first and last points are the same)
  bool IsPolygonClosed(const std::vector<Point*>& polygon) const;
  
  /// Check if a polygon is self-intersecting
  bool IsPolygonSelfIntersecting(const std::vector<Point*>& polygon) const;
  
  /// Check if a point is inside a polygon
  bool IsPointInPolygon(const Point* point, const std::vector<Point*>& polygon) const;
  
  /// Check if two polygons intersect
  bool DoPolygonsIntersect(const std::vector<Point*>& polygon1, const std::vector<Point*>& polygon2) const;
  
  /// Check if a hole is valid (inside the main polygon, not intersecting other holes)
  void CheckHoleValidity(const std::vector<Point*>& hole, const std::vector<Point*>& mainPolygon, const std::vector<std::vector<Point*>>& existingHoles) const;
  
  /// Get the current epsilon value
  double GetEpsilon() const { return epsilon_; }
  
  /// Set a new epsilon value
  void SetEpsilon(double epsilon) {
    if (epsilon <= 0.0 || epsilon >= 1.0) {
      throw InvalidInputException::Create("epsilon", epsilon);
    }
    epsilon_ = epsilon;
  }
  
private:
  /// Epsilon value for floating point comparisons
  double epsilon_;
  
  /// Check if two points are equal within epsilon
  bool ArePointsEqual(const Point* p1, const Point* p2) const;
  
  /// Check if two points are equal within epsilon
  bool ArePointsEqual(const Point& p1, const Point& p2) const;
  
  /// Check if three points are collinear within epsilon
  bool ArePointsCollinear(const Point* p1, const Point* p2, const Point* p3) const;
  
  /// Calculate the cross product of (b - a) and (c - a)
  double CrossProduct(const Point* a, const Point* b, const Point* c) const;
  
  /// Check if two line segments intersect
  bool DoSegmentsIntersect(const Point* a1, const Point* a2, const Point* b1, const Point* b2) const;
};

/// Global validator instance with default epsilon
P2T_DLL_SYMBOL extern InputValidator g_validator;

} // namespace p2t
