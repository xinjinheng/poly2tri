/*
 * Poly2Tri Copyright (c) 2009-2021, Poly2Tri Contributors
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
#include "cdt.h"

namespace p2t {

CDT::CDT(const std::vector<Point*>& polyline)
{
  if (polyline.empty()) {
    throw InvalidInputException::EmptyContainer("polyline");
  }
  try {
    g_validator.CheckPolygonValidity(polyline);
  } catch (const Poly2TriException& e) {
    throw InvalidInputException("Invalid main polygon: " + std::string(e.what()));
  }
  sweep_context_ = new SweepContext(polyline);
  if (sweep_context_ == nullptr) {
    throw NullPointerException::Create("sweep_context_");
  }
  sweep_ = new Sweep;
  if (sweep_ == nullptr) {
    delete sweep_context_;
    throw NullPointerException::Create("sweep_");
  }
}

void CDT::AddHole(const std::vector<Point*>& polyline)
{
  if (sweep_context_ == nullptr) {
    throw NullPointerException::Create("sweep_context_");
  }
  if (polyline.empty()) {
    throw InvalidInputException::EmptyContainer("hole polyline");
  }
  try {
    // Get the main polygon from the sweep context
    const std::vector<Point*>& main_polygon = sweep_context_->GetPoints();
    // Check if this is the first hole
    const std::vector<std::vector<Point*>>& existing_holes = sweep_context_->GetHoles();
    g_validator.CheckHoleValidity(polyline, main_polygon, existing_holes);
  } catch (const Poly2TriException& e) {
    throw InvalidInputException("Invalid hole: " + std::string(e.what()));
  }
  sweep_context_->AddHole(polyline);
}

void CDT::AddPoint(Point* point) {
  if (sweep_context_ == nullptr) {
    throw NullPointerException::Create("sweep_context_");
  }
  if (point == nullptr) {
    throw NullPointerException::Create("point");
  }
  if (!g_validator.IsValidPoint(point)) {
    std::ostringstream oss;
    oss << "Invalid Steiner point (" << point->x << ", " << point->y << ")";
    throw InvalidInputException(oss.str());
  }
  // Check if the point is already in the main polygon or any hole
  const std::vector<Point*>& main_polygon = sweep_context_->GetPoints();
  for (const Point* p : main_polygon) {
    if (g_validator.ArePointsEqual(point, p)) {
      std::ostringstream oss;
      oss << "Duplicate Steiner point (" << point->x << ", " << point->y << ")";
      throw InvalidInputException::DuplicatePoint(oss.str());
    }
  }
  const std::vector<std::vector<Point*>>& holes = sweep_context_->GetHoles();
  for (const std::vector<Point*>& hole : holes) {
    for (const Point* p : hole) {
      if (g_validator.ArePointsEqual(point, p)) {
        std::ostringstream oss;
        oss << "Duplicate Steiner point (" << point->x << ", " << point->y << ")";
        throw InvalidInputException::DuplicatePoint(oss.str());
      }
    }
  }
  // Check if the point is inside the main polygon and not inside any hole
  if (!g_validator.IsPointInPolygon(point, main_polygon)) {
    std::ostringstream oss;
    oss << "Steiner point (" << point->x << ", " << point->y << ") is outside the main polygon";
    throw InvalidInputException(oss.str());
  }
  for (const std::vector<Point*>& hole : holes) {
    if (g_validator.IsPointInPolygon(point, hole)) {
      std::ostringstream oss;
      oss << "Steiner point (" << point->x << ", " << point->y << ") is inside a hole";
      throw InvalidInputException(oss.str());
    }
  }
  sweep_context_->AddPoint(point);
}

void CDT::Triangulate()
{
  if (sweep_ == nullptr || sweep_context_ == nullptr) {
    throw NullPointerException::Create("sweep_ or sweep_context_");
  }
  try {
    sweep_->Triangulate(*sweep_context_);
  } catch (const Poly2TriException& e) {
    throw TriangulationFailedException("Triangulation failed: " + std::string(e.what()));
  } catch (const std::exception& e) {
    throw TriangulationFailedException("Unexpected error during triangulation: " + std::string(e.what()));
  }
}

std::vector<p2t::Triangle*> CDT::GetTriangles()
{
  if (sweep_context_ == nullptr) {
    throw NullPointerException::Create("sweep_context_");
  }
  std::vector<Triangle*> triangles = sweep_context_->GetTriangles();
  if (triangles.empty()) {
    throw InvalidInputException::EmptyContainer("triangles");
  }
  return triangles;
}

std::list<p2t::Triangle*> CDT::GetMap()
{
  if (sweep_context_ == nullptr) {
    throw NullPointerException::Create("sweep_context_");
  }
  std::list<Triangle*> map = sweep_context_->GetMap();
  if (map.empty()) {
    throw InvalidInputException::EmptyContainer("triangle map");
  }
  return map;
}

CDT::~CDT()
{
  delete sweep_context_;
  delete sweep_;
}

} // namespace p2t
