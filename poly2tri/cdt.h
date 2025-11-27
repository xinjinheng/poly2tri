#ifndef CDT_H
#define CDT_H

#include "sweep/sweep_context.h"
#include "common/exceptions.h"
#include "common/validators.h"
#include <vector>
#include <map>

namespace p2t {

class CDT {
public:

  CDT(const std::vector<Point*>& polyline);
  ~CDT();

  void AddHole(const std::vector<Point*>& polyline);
  void AddPoint(Point* point);
  void Triangulate();

  const std::vector<Triangle*>& GetTriangles() const;
  const std::map<EdgeKey, Edge*>& GetMap() const;

private:

  SweepContext* sweep_context_;

  // Helper functions for input validation
  bool IsPointInsidePolygon(const Point* point, const std::vector<Point*>& polygon) const;
  bool HasSelfIntersections(const std::vector<Point*>& polygon) const;
  bool DoPolygonsIntersect(const std::vector<Point*>& poly1, const std::vector<Point*>& poly2) const;
  bool DoSegmentsIntersect(const Point* p1, const Point* p2, const Point* p3, const Point* p4) const;
  int Orientation(const Point* p, const Point* q, const Point* r) const;
  bool OnSegment(const Point* p, const Point* q, const Point* r) const;

};

} // namespace p2t

#endif // CDT_H
