/*
 * Poly2Tri Copyright (c) 2009-2022, Poly2Tri Contributors
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
#include "sweep_context.h"
#include <algorithm>
#include "advancing_front.h"

namespace p2t {

SweepContext::SweepContext(std::vector<Point*> polyline) : points_(std::move(polyline)),
  front_(nullptr),
  head_(nullptr),
  tail_(nullptr),
  af_head_(nullptr),
  af_middle_(nullptr),
  af_tail_(nullptr)
{
  // Check for duplicate points
  for (size_t i = 0; i < points_.size(); ++i) {
    for (size_t j = i + 1; j < points_.size(); ++j) {
      if (PointsEqual(*points_[i], *points_[j])) {
        throw DuplicatePointException(points_[i]->x, points_[i]->y);
      }
    }
  }
  
  // Check for self-intersecting polygon
  size_t n = points_.size();
  for (size_t i = 0; i < n; ++i) {
    size_t i1 = (i + 1) % n;
    for (size_t j = i + 2; j < n; ++j) {
      size_t j1 = (j + 1) % n;
      if (i != j1 && i1 != j) {
        if (SegmentsIntersect(*points_[i], *points_[i1], *points_[j], *points_[j1])) {
          throw SelfIntersectingPolygonException(static_cast<int>(i), static_cast<int>(j));
        }
      }
    }
  }
  
  InitEdges(points_);
}

void SweepContext::AddHole(const std::vector<Point*>& polyline)
{
  // Check for duplicate points in the hole
  for (size_t i = 0; i < polyline.size(); ++i) {
    for (size_t j = i + 1; j < polyline.size(); ++j) {
      if (PointsEqual(*polyline[i], *polyline[j])) {
        throw DuplicatePointException(polyline[i]->x, polyline[i]->y);
      }
    }
  }
  
  // Check for self-intersecting hole
  size_t n = polyline.size();
  for (size_t i = 0; i < n; ++i) {
    size_t i1 = (i + 1) % n;
    for (size_t j = i + 2; j < n; ++j) {
      size_t j1 = (j + 1) % n;
      if (i != j1 && i1 != j) {
        if (SegmentsIntersect(*polyline[i], *polyline[i1], *polyline[j], *polyline[j1])) {
          throw SelfIntersectingPolygonException(static_cast<int>(i), static_cast<int>(j));
        }
      }
    }
  }
  
  // Check if hole is inside the main polygon
  if (!PointInPolygon(points_, *polyline[0])) {
    throw InvalidHoleException("Hole is not inside the main polygon");
  }
  
  InitEdges(polyline);
  for (auto i : polyline) {
    points_.push_back(i);
  }
}

void SweepContext::AddPoint(Point* point) {
  // Check for duplicate point
  for (const auto& p : points_) {
    if (PointsEqual(*p, *point)) {
      throw DuplicatePointException(point->x, point->y);
    }
  }
  points_.push_back(point);
}

std::vector<Triangle*> &SweepContext::GetTriangles()
{
  return triangles_;
}

std::list<Triangle*> &SweepContext::GetMap()
{
  return map_;
}

void SweepContext::InitTriangulation()
{
  double xmax(points_[0]->x), xmin(points_[0]->x);
  double ymax(points_[0]->y), ymin(points_[0]->y);

  // Calculate bounds.
  for (auto& point : points_) {
    Point& p = *point;
    if (p.x > xmax)
      xmax = p.x;
    if (p.x < xmin)
      xmin = p.x;
    if (p.y > ymax)
      ymax = p.y;
    if (p.y < ymin)
      ymin = p.y;
  }

  double dx = kAlpha * (xmax - xmin);
  double dy = kAlpha * (ymax - ymin);
  head_ = new Point(xmin - dx, ymin - dy);
  tail_ = new Point(xmax + dx, ymin - dy);

  // Sort points along y-axis
  std::sort(points_.begin(), points_.end(), cmp);

}

void SweepContext::InitEdges(const std::vector<Point*>& polyline)
{
  size_t num_points = polyline.size();
  for (size_t i = 0; i < num_points; i++) {
    size_t j = i < num_points - 1 ? i + 1 : 0;
    edge_list.push_back(std::make_shared<Edge>(*polyline[i], *polyline[j]));
  }
}

Point* SweepContext::GetPoint(size_t index)
{
  return points_[index];
}

void SweepContext::AddToMap(Triangle* triangle)
{
  map_.push_back(triangle);
}

Node* SweepContext::LocateNode(const Point& point)
{
  if (front_ == nullptr) {
    throw NullPointerException("SweepContext::LocateNode", front_);
  }
  // TODO implement search tree
  Node* node = front_->LocateNode(point.x);
  if (node == nullptr) {
    throw NullPointerException("SweepContext::LocateNode", node);
  }
  return node;
}

void SweepContext::CreateAdvancingFront()
{
  if (points_.empty()) {
    throw std::runtime_error("No points available to create advancing front");
  }
  if (head_ == nullptr || tail_ == nullptr) {
    throw NullPointerException("SweepContext::CreateAdvancingFront", nullptr);
  }

  // Initial triangle
  Triangle* triangle = new Triangle(*points_[0], *head_, *tail_);

  map_.push_back(triangle);

  af_head_ = new Node(*triangle->GetPoint(1), *triangle);
  af_middle_ = new Node(*triangle->GetPoint(0), *triangle);
  af_tail_ = new Node(*triangle->GetPoint(2));
  front_ = new AdvancingFront(*af_head_, *af_tail_);

  if (af_head_ == nullptr || af_middle_ == nullptr || af_tail_ == nullptr || front_ == nullptr) {
    throw NullPointerException("SweepContext::CreateAdvancingFront", nullptr);
  }

  // TODO: More intuitive if head is middles next and not previous?
  //       so swap head and tail
  af_head_->next = af_middle_;
  af_middle_->next = af_tail_;
  af_middle_->prev = af_head_;
  af_tail_->prev = af_middle_;
}

void SweepContext::RemoveNode(Node* node)
{
  delete node;
}

void SweepContext::MapTriangleToNodes(Triangle& t)
{
  for (int i = 0; i < 3; i++) {
    if (!t.GetNeighbor(i)) {
      Node* n = front_->LocatePoint(t.PointCW(*t.GetPoint(i)));
      if (n)
        n->triangle = &t;
    }
  }
}

void SweepContext::RemoveFromMap(Triangle* triangle)
{
  map_.remove(triangle);
}

void SweepContext::MeshClean(Triangle& triangle)
{
  std::vector<Triangle *> triangles;
  triangles.push_back(&triangle);

  while(!triangles.empty()){
	Triangle *t = triangles.back();
	triangles.pop_back();

    if (t != nullptr && !t->IsInterior()) {
      t->IsInterior(true);
      triangles_.push_back(t);
      for (int i = 0; i < 3; i++) {
        if (!t->constrained_edge[i])
          triangles.push_back(t->GetNeighbor(i));
      }
    }
  }
}

SweepContext::~SweepContext()
{

    // Clean up memory

    delete head_;
    delete tail_;
    delete front_;
    delete af_head_;
    delete af_middle_;
    delete af_tail_;

    for (auto ptr : map_) {
      delete ptr;
    }

    // Edge objects are managed by shared_ptr, no need to delete manually
}

} // namespace p2t
