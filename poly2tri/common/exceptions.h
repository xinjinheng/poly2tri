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

#include <stdexcept>
#include <string>
#include <sstream>

namespace p2t {

// Base exception class for all poly2tri exceptions
class P2T_DLL_SYMBOL P2TException : public std::runtime_error {
public:
  explicit P2TException(const std::string& message) : std::runtime_error(message) {}
};

// Exception for duplicate points
class P2T_DLL_SYMBOL DuplicatePointException : public P2TException {
public:
  DuplicatePointException(double x, double y)
    : P2TException(CreateMessage(x, y)) {}
  
private:
  static std::string CreateMessage(double x, double y) {
    std::ostringstream oss;
    oss << "Duplicate point detected: (" << x << ", " << y << ")";
    return oss.str();
  }
};

// Exception for self-intersecting polygons
class P2T_DLL_SYMBOL SelfIntersectingPolygonException : public P2TException {
public:
  SelfIntersectingPolygonException(int edge1_idx, int edge2_idx)
    : P2TException(CreateMessage(edge1_idx, edge2_idx)) {}
  
private:
  static std::string CreateMessage(int edge1_idx, int edge2_idx) {
    std::ostringstream oss;
    oss << "Self-intersecting polygon detected between edges " << edge1_idx << " and " << edge2_idx;
    return oss.str();
  }
};

// Exception for invalid holes (touching boundary or other holes)
class P2T_DLL_SYMBOL InvalidHoleException : public P2TException {
public:
  InvalidHoleException(const std::string& reason)
    : P2TException(reason) {}
};

// Exception for null pointer access
class P2T_DLL_SYMBOL NullPointerException : public P2TException {
public:
  NullPointerException(const std::string& location, const void* pointer)
    : P2TException(CreateMessage(location, pointer)) {}
  
private:
  static std::string CreateMessage(const std::string& location, const void* pointer) {
    std::ostringstream oss;
    oss << "Null pointer exception at " << location << ", pointer address: " << pointer;
    return oss.str();
  }
};

// Exception for degenerate triangles (collinear points)
class P2T_DLL_SYMBOL DegenerateTriangleException : public P2TException {
public:
  DegenerateTriangleException(const Point* p1, const Point* p2, const Point* p3)
    : P2TException(CreateMessage(p1, p2, p3)) {}
  
private:
  static std::string CreateMessage(const Point* p1, const Point* p2, const Point* p3) {
    std::ostringstream oss;
    oss << "Degenerate triangle detected with points: (" 
        << p1->x << ", " << p1->y << "), (" 
        << p2->x << ", " << p2->y << "), (" 
        << p3->x << ", " << p3->y << ")";
    return oss.str();
  }
};

// Exception for timeout
class P2T_DLL_SYMBOL TimeoutException : public P2TException {
public:
  TimeoutException() : P2TException("Triangulation timeout") {}
};

} // namespace p2t