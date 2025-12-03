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
#include <vector>
#include <algorithm>
#include <stdexcept>
#include "cdt.h"
#include "../common/utils.h"

namespace p2t {

CDT::CDT(const std::vector<Point*>& polyline)
{
  // Validate input polyline
  if (polyline.size() < 3) {
    throw std::invalid_argument("Polyline must have at least 3 points");
  }
  for (const auto& point : polyline) {
    if (point == nullptr) {
      throw std::invalid_argument("Polyline contains null point");
    }
  }
  sweep_context_ = new SweepContext(polyline);
  sweep_ = new Sweep;
}

void CDT::AddHole(const std::vector<Point*>& polyline)
{
  // Validate hole polyline
  if (polyline.size() < 3) {
    throw std::invalid_argument("Hole polyline must have at least 3 points");
  }
  for (const auto& point : polyline) {
    if (point == nullptr) {
      throw std::invalid_argument("Hole polyline contains null point");
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  sweep_context_->AddHole(polyline);
}

void CDT::AddPoint(Point* point) {
  // Validate point
  if (point == nullptr) {
    throw std::invalid_argument("Point cannot be null");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  sweep_context_->AddPoint(point);
}

void CDT::Triangulate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  sweep_->Triangulate(*sweep_context_);
}

std::vector<p2t::Triangle*> CDT::GetTriangles()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return sweep_context_->GetTriangles();
}

std::list<p2t::Triangle*> CDT::GetMap()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return sweep_context_->GetMap();
}

CDT::~CDT()
{
  delete sweep_context_;
  delete sweep_;
}

} // namespace p2t
