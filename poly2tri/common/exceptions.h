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

#include <stdexcept>
#include <string>
#include <sstream>

namespace p2t {

class P2T_DLL_SYMBOL Poly2TriException : public std::runtime_error {
public:
  explicit Poly2TriException(const std::string& message)
      : std::runtime_error(message) {}
  
  explicit Poly2TriException(const char* message)
      : std::runtime_error(message) {}
  
  virtual ~Poly2TriException() noexcept = default;
};

class P2T_DLL_SYMBOL NullPointerException : public Poly2TriException {
public:
  explicit NullPointerException(const std::string& message)
      : Poly2TriException(message) {}
  
  explicit NullPointerException(const char* message)
      : Poly2TriException(message) {}
  
  template<typename T>
  static NullPointerException Create(const std::string& variableName) {
    std::ostringstream oss;
    oss << "Null pointer exception: " << variableName << " is null";
    return NullPointerException(oss.str());
  }
};

class P2T_DLL_SYMBOL InvalidInputException : public Poly2TriException {
public:
  explicit InvalidInputException(const std::string& message)
      : Poly2TriException(message) {}
  
  explicit InvalidInputException(const char* message)
      : Poly2TriException(message) {}
  
  template<typename T>
  static InvalidInputException Create(const std::string& parameterName, const T& value) {
    std::ostringstream oss;
    oss << "Invalid input: " << parameterName << " has invalid value " << value;
    return InvalidInputException(oss.str());
  }
  
  static InvalidInputException EmptyContainer(const std::string& containerName) {
    std::ostringstream oss;
    oss << "Invalid input: " << containerName << " is empty";
    return InvalidInputException(oss.str());
  }
  
  static InvalidInputException DuplicatePoint(const std::string& pointInfo) {
    std::ostringstream oss;
    oss << "Invalid input: Duplicate point found: " << pointInfo;
    return InvalidInputException(oss.str());
  }
  
  static InvalidInputException InvalidPolygon(const std::string& reason) {
    std::ostringstream oss;
    oss << "Invalid polygon: " << reason;
    return InvalidInputException(oss.str());
  }
};

class P2T_DLL_SYMBOL TriangulationFailedException : public Poly2TriException {
public:
  explicit TriangulationFailedException(const std::string& message)
      : Poly2TriException(message) {}
  
  explicit TriangulationFailedException(const char* message)
      : Poly2TriException(message) {}
  
  static TriangulationFailedException PointProcessingFailed(const std::string& pointInfo) {
    std::ostringstream oss;
    oss << "Triangulation failed: Error processing point " << pointInfo;
    return TriangulationFailedException(oss.str());
  }
  
  static TriangulationFailedException InvalidTriangle(const std::string& triangleInfo) {
    std::ostringstream oss;
    oss << "Triangulation failed: Invalid triangle encountered " << triangleInfo;
    return TriangulationFailedException(oss.str());
  }
};

class P2T_DLL_SYMBOL GeometryException : public Poly2TriException {
public:
  explicit GeometryException(const std::string& message)
      : Poly2TriException(message) {}
  
  explicit GeometryException(const char* message)
      : Poly2TriException(message) {}
  
  static GeometryException CollinearPoints(const std::string& pointInfo) {
    std::ostringstream oss;
    oss << "Geometry error: Collinear points found: " << pointInfo;
    return GeometryException(oss.str());
  }
  
  static GeometryException DegenerateTriangle(const std::string& triangleInfo) {
    std::ostringstream oss;
    oss << "Geometry error: Degenerate triangle encountered " << triangleInfo;
    return GeometryException(oss.str());
  }
  
  static GeometryException NumericalInstability(const std::string& operation) {
    std::ostringstream oss;
    oss << "Geometry error: Numerical instability in " << operation;
    return GeometryException(oss.str());
  }
};

} // namespace p2t
