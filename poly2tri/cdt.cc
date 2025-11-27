#include "cdt.h"
#include "sweep/sweep.h"
#include "common/exceptions.h"
#include <cassert>
#include <cmath>
#include <algorithm>

namespace p2t {

CDT::CDT(const std::vector<Point*>& polyline) : sweep_context_(new SweepContext(polyline)) {
  if (sweep_context_ == nullptr) {
    throw NullPointerException::Create("CDT::CDT sweep_context_");
  }
  // Validate input polyline
  if (polyline.empty()) {
    throw InvalidInputException::Create("CDT::CDT polyline", 0);
  }
  if (polyline.size() < 3) {
    throw InvalidInputException::Create("CDT::CDT polyline must have at least 3 points");
  }
  for (const Point* p : polyline) {
    if (p == nullptr) {
      throw NullPointerException::Create("CDT::CDT polyline point");
    }
  }
  // Validate that polyline is a closed polygon
  if (*polyline.front() != *polyline.back()) {
    throw InvalidInputException::Create("CDT::CDT polyline is not closed");
  }
  // Check for duplicate points in polyline
  for (size_t i = 0; i < polyline.size(); ++i) {
    for (size_t j = i + 1; j < polyline.size(); ++j) {
      if (*polyline[i] == *polyline[j]) {
        throw InvalidInputException::Create("CDT::CDT polyline contains duplicate points");
      }
    }
  }
}

CDT::~CDT() {
  delete sweep_context_;
}

bool CDT::IsPointInsidePolygon(const Point* point, const std::vector<Point*>& polygon) const {
    if (point == nullptr || polygon.empty()) {
        return false;
    }
    bool inside = false;
    size_t n = polygon.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        if (((polygon[i]->y > point->y) != (polygon[j]->y > point->y)) &&
            (point->x < (polygon[j]->x - polygon[i]->x) * (point->y - polygon[i]->y) / (polygon[j]->y - polygon[i]->y) + polygon[i]->x)) {
            inside = !inside;
        }
    }
    return inside;
}

bool CDT::HasSelfIntersections(const std::vector<Point*>& polygon) const {
    if (polygon.size() < 4) {
        return false;
    }
    size_t n = polygon.size();
    for (size_t i = 0; i < n - 1; i++) {
        for (size_t j = i + 2; j < n - 1; j++) {
            if (i == 0 && j == n - 2) {
                continue; // Skip last segment connecting to first
            }
            if (DoSegmentsIntersect(polygon[i], polygon[i + 1], polygon[j], polygon[j + 1])) {
                return true;
            }
        }
    }
    return false;
}

bool CDT::DoPolygonsIntersect(const std::vector<Point*>& poly1, const std::vector<Point*>& poly2) const {
    size_t n1 = poly1.size();
    size_t n2 = poly2.size();
    for (size_t i = 0; i < n1 - 1; i++) {
        for (size_t j = 0; j < n2 - 1; j++) {
            if (DoSegmentsIntersect(poly1[i], poly1[i + 1], poly2[j], poly2[j + 1])) {
                return true;
            }
        }
    }
    return false;
}

bool CDT::DoSegmentsIntersect(const Point* p1, const Point* p2, const Point* p3, const Point* p4) const {
    if (p1 == nullptr || p2 == nullptr || p3 == nullptr || p4 == nullptr) {
        return false;
    }
    int o1 = Orientation(p1, p2, p3);
    int o2 = Orientation(p1, p2, p4);
    int o3 = Orientation(p3, p4, p1);
    int o4 = Orientation(p3, p4, p2);
    if (o1 != o2 && o3 != o4) {
        return true;
    }
    if (o1 == 0 && OnSegment(p1, p3, p2)) {
        return true;
    }
    if (o2 == 0 && OnSegment(p1, p4, p2)) {
        return true;
    }
    if (o3 == 0 && OnSegment(p3, p1, p4)) {
        return true;
    }
    if (o4 == 0 && OnSegment(p3, p2, p4)) {
        return true;
    }
    return false;
}

int CDT::Orientation(const Point* p, const Point* q, const Point* r) const {
    if (p == nullptr || q == nullptr || r == nullptr) {
        return 0;
    }
    double val = (q->y - p->y) * (r->x - q->x) - (q->x - p->x) * (r->y - q->y);
    if (val == 0) {
        return 0;
    }
    return (val > 0) ? 1 : 2;
}

bool CDT::OnSegment(const Point* p, const Point* q, const Point* r) const {
    if (p == nullptr || q == nullptr || r == nullptr) {
        return false;
    }
    if (q->x <= std::max(p->x, r->x) && q->x >= std::min(p->x, r->x) &&
        q->y <= std::max(p->y, r->y) && q->y >= std::min(p->y, r->y)) {
        return true;
    }
    return false;
}

void CDT::AddHole(const std::vector<Point*>& polyline) {
  if (sweep_context_ == nullptr) {
    throw NullPointerException::Create("CDT::AddHole sweep_context_");
  }
  // Validate input polyline
  if (polyline.empty()) {
    throw InvalidInputException::Create("CDT::AddHole polyline", 0);
  }
  if (polyline.size() < 3) {
    throw InvalidInputException::Create("CDT::AddHole polyline must have at least 3 points");
  }
  for (const Point* p : polyline) {
    if (p == nullptr) {
      throw NullPointerException::Create("CDT::AddHole polyline point");
    }
  }
  // Validate that polyline is a closed polygon
  if (*polyline.front() != *polyline.back()) {
    throw InvalidInputException::Create("CDT::AddHole polyline is not closed");
  }
  // Check for duplicate points in hole
  for (size_t i = 0; i < polyline.size(); ++i) {
    for (size_t j = i + 1; j < polyline.size(); ++j) {
      if (*polyline[i] == *polyline[j]) {
        throw InvalidInputException::Create("CDT::AddHole hole contains duplicate points");
      }
    }
  }
  sweep_context_->AddHole(polyline);
}

void CDT::AddPoint(Point* point) {
  if (sweep_context_ == nullptr) {
    throw NullPointerException::Create("CDT::AddPoint sweep_context_");
  }
  if (point == nullptr) {
    throw NullPointerException::Create("CDT::AddPoint point");
  }
  // Check if point is already in the triangulation
  const std::vector<Point*>& points = sweep_context_->GetPoints();
  for (const Point* p : points) {
    if (p == nullptr) {
      throw NullPointerException::Create("CDT::AddPoint existing point");
    }
    if (*p == *point) {
      throw InvalidInputException::Create("CDT::AddPoint duplicate point");
    }
  }
  sweep_context_->AddPoint(point);
}

void CDT::Triangulate() {
  if (sweep_context_ == nullptr) {
    throw NullPointerException::Create("CDT::Triangulate sweep_context_");
  }
  try {
    Sweep sweep;
    sweep.Triangulate(*sweep_context_);
  } catch (const Poly2TriException& e) {
    throw TriangulationFailedException::Create("CDT::Triangulate: Triangulation failed - " + std::string(e.what()));
  } catch (const std::exception& e) {
    throw TriangulationFailedException::Create("CDT::Triangulate: Triangulation failed - " + std::string(e.what()));
  }
}

const std::vector<Triangle*>& CDT::GetTriangles() const {
  if (sweep_context_ == nullptr) {
    throw NullPointerException::Create("CDT::GetTriangles sweep_context_");
  }
  const std::vector<Triangle*>& triangles = sweep_context_->GetTriangles();
  if (triangles.empty()) {
    throw InvalidInputException::Create("CDT::GetTriangles: No triangles available");
  }
  for (const Triangle* t : triangles) {
    if (t == nullptr) {
      throw NullPointerException::Create("CDT::GetTriangles triangle");
    }
  }
  return triangles;
}

const std::map<EdgeKey, Edge*>& CDT::GetMap() const {
  if (sweep_context_ == nullptr) {
    throw NullPointerException::Create("CDT::GetMap sweep_context_");
  }
  const std::map<EdgeKey, Edge*>& map = sweep_context_->GetMap();
  if (map.empty()) {
    throw InvalidInputException::Create("CDT::GetMap: No map available");
  }
  for (const auto& pair : map) {
    if (pair.second == nullptr) {
      throw NullPointerException::Create("CDT::GetMap edge");
    }
  }
  return map;
}

} // namespace p2t