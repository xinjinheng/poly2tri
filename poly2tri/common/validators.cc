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

#include "validators.h"
#include <cmath>
#include <sstream>

namespace p2t {

// Global validator instance
InputValidator g_validator;

bool InputValidator::IsValidPoint(const Point* point) const {
  if (point == nullptr) {
    return false;
  }
  return IsValidPoint(*point);
}

bool InputValidator::IsValidPoint(const Point& point) const {
  return !std::isnan(point.x) && !std::isinf(point.x) &&
         !std::isnan(point.y) && !std::isinf(point.y);
}

void InputValidator::CheckDuplicatePoints(const std::vector<Point*>& points) const {
  std::unordered_set<Point*> unique_points;
  for (const Point* point : points) {
    if (point == nullptr) {
      throw NullPointerException::Create("point");
    }
    if (!IsValidPoint(point)) {
      std::ostringstream oss;
      oss << "(" << point->x << ", " << point->y << ")";
      throw InvalidInputException::Create("point", oss.str());
    }
    for (const Point* existing_point : unique_points) {
      if (ArePointsEqual(point, existing_point)) {
        std::ostringstream oss;
        oss << "(" << point->x << ", " << point->y << ")";
        throw InvalidInputException::DuplicatePoint(oss.str());
      }
    }
    unique_points.insert(point);
  }
}

void InputValidator::CheckPolygonValidity(const std::vector<Point*>& polygon) const {
  if (polygon.empty()) {
    throw InvalidInputException::EmptyContainer("polygon");
  }
  if (polygon.size() < 3) {
    throw InvalidInputException::InvalidPolygon("must have at least 3 points");
  }
  CheckDuplicatePoints(polygon);
  if (!IsPolygonClosed(polygon)) {
    throw InvalidInputException::InvalidPolygon("is not closed");
  }
  if (IsPolygonSelfIntersecting(polygon)) {
    throw InvalidInputException::InvalidPolygon("is self-intersecting");
  }
}

bool InputValidator::IsPolygonClosed(const std::vector<Point*>& polygon) const {
  if (polygon.size() < 2) {
    return false;
  }
  const Point* first = polygon.front();
  const Point* last = polygon.back();
  return ArePointsEqual(first, last);
}

bool InputValidator::IsPolygonSelfIntersecting(const std::vector<Point*>& polygon) const {
  const size_t n = polygon.size();
  if (n < 4) {
    return false; // A polygon with less than 4 points can't be self-intersecting
  }
  for (size_t i = 0; i < n - 1; ++i) {
    const Point* a1 = polygon[i];
    const Point* a2 = polygon[i + 1];
    for (size_t j = i + 2; j < n - 1; ++j) {
      const Point* b1 = polygon[j];
      const Point* b2 = polygon[j + 1];
      if (DoSegmentsIntersect(a1, a2, b1, b2)) {
        return true;
      }
    }
  }
  return false;
}

bool InputValidator::IsPointInPolygon(const Point* point, const std::vector<Point*>& polygon) const {
  if (point == nullptr || polygon.empty()) {
    return false;
  }
  bool inside = false;
  const size_t n = polygon.size();
  const Point* p1 = polygon[0];
  for (size_t i = 1; i <= n; ++i) {
    const Point* p2 = polygon[i % n];
    if (point->y > std::min(p1->y, p2->y)) {
      if (point->y <= std::max(p1->y, p2->y)) {
        if (point->x <= std::max(p1->x, p2->x)) {
          double x_intersect = (point->y - p1->y) * (p2->x - p1->x) / (p2->y - p1->y) + p1->x;
          if (p1->x == p2->x || point->x <= x_intersect) {
            inside = !inside;
          }
        }
      }
    }
    p1 = p2;
  }
  return inside;
}

bool InputValidator::DoPolygonsIntersect(const std::vector<Point*>& polygon1, const std::vector<Point*>& polygon2) const {
  const size_t n1 = polygon1.size();
  const size_t n2 = polygon2.size();
  if (n1 < 3 || n2 < 3) {
    return false;
  }
  // Check if any edges of polygon1 intersect with edges of polygon2
  for (size_t i = 0; i < n1; ++i) {
    const Point* a1 = polygon1[i];
    const Point* a2 = polygon1[(i + 1) % n1];
    for (size_t j = 0; j < n2; ++j) {
      const Point* b1 = polygon2[j];
      const Point* b2 = polygon2[(j + 1) % n2];
      if (DoSegmentsIntersect(a1, a2, b1, b2)) {
        return true;
      }
    }
  }
  // Check if one polygon is completely inside the other
  if (IsPointInPolygon(polygon1[0], polygon2) || IsPointInPolygon(polygon2[0], polygon1)) {
    return true;
  }
  return false;
}

void InputValidator::CheckHoleValidity(const std::vector<Point*>& hole, const std::vector<Point*>& mainPolygon, const std::vector<std::vector<Point*>>& existingHoles) const {
  CheckPolygonValidity(hole);
  // Check if the hole is inside the main polygon
  if (!IsPointInPolygon(hole[0], mainPolygon)) {
    throw InvalidInputException::InvalidPolygon("hole is not inside the main polygon");
  }
  // Check if the hole intersects with any existing holes
  for (const std::vector<Point*>& existingHole : existingHoles) {
    if (DoPolygonsIntersect(hole, existingHole)) {
      throw InvalidInputException::InvalidPolygon("hole intersects with another hole");
    }
  }
}

bool InputValidator::ArePointsEqual(const Point* p1, const Point* p2) const {
  if (p1 == nullptr || p2 == nullptr) {
    return p1 == p2;
  }
  return ArePointsEqual(*p1, *p2);
}

bool InputValidator::ArePointsEqual(const Point& p1, const Point& p2) const {
  return std::abs(p1.x - p2.x) < epsilon_ && std::abs(p1.y - p2.y) < epsilon_;
}

bool InputValidator::ArePointsCollinear(const Point* p1, const Point* p2, const Point* p3) const {
  if (p1 == nullptr || p2 == nullptr || p3 == nullptr) {
    throw NullPointerException::Create("point");
  }
  double cross = CrossProduct(p1, p2, p3);
  return std::abs(cross) < epsilon_;
}

double InputValidator::CrossProduct(const Point* a, const Point* b, const Point* c) const {
  if (a == nullptr || b == nullptr || c == nullptr) {
    throw NullPointerException::Create("point");
  }
  double ab_x = b->x - a->x;
  double ab_y = b->y - a->y;
  double ac_x = c->x - a->x;
  double ac_y = c->y - a->y;
  return ab_x * ac_y - ab_y * ac_x;
}

bool InputValidator::DoSegmentsIntersect(const Point* a1, const Point* a2, const Point* b1, const Point* b2) const {
  if (a1 == nullptr || a2 == nullptr || b1 == nullptr || b2 == nullptr) {
    throw NullPointerException::Create("point");
  }
  // Check if any of the endpoints are the same
  if (ArePointsEqual(a1, b1) || ArePointsEqual(a1, b2) || ArePointsEqual(a2, b1) || ArePointsEqual(a2, b2)) {
    return true;
  }
  // Calculate orientation of each triplet of points
  double o1 = CrossProduct(a1, a2, b1);
  double o2 = CrossProduct(a1, a2, b2);
  double o3 = CrossProduct(b1, b2, a1);
  double o4 = CrossProduct(b1, b2, a2);
  // General case: segments intersect in their interiors
  if (o1 * o2 < 0 && o3 * o4 < 0) {
    return true;
  }
  // Special cases: colinear and overlapping
  if (std::abs(o1) < epsilon_ && IsPointOnSegment(b1, a1, a2)) {
    return true;
  }
  if (std::abs(o2) < epsilon_ && IsPointOnSegment(b2, a1, a2)) {
    return true;
  }
  if (std::abs(o3) < epsilon_ && IsPointOnSegment(a1, b1, b2)) {
    return true;
  }
  if (std::abs(o4) < epsilon_ && IsPointOnSegment(a2, b1, b2)) {
    return true;
  }
  return false;
}

// Helper function to check if a point is on a line segment
bool InputValidator::IsPointOnSegment(const Point* p, const Point* a, const Point* b) const {
  if (p == nullptr || a == nullptr || b == nullptr) {
    throw NullPointerException::Create("point");
  }
  // Check if the point is colinear with a and b
  if (std::abs(CrossProduct(a, b, p)) > epsilon_) {
    return false;
  }
  // Check if the point's coordinates are within the segment's bounding box
  if (p->x < std::min(a->x, b->x) - epsilon_ || p->x > std::max(a->x, b->x) + epsilon_) {
    return false;
  }
  if (p->y < std::min(a->y, b->y) - epsilon_ || p->y > std::max(a->y, b->y) + epsilon_) {
    return false;
  }
  return true;
}

} // namespace p2t
