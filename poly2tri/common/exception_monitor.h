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

#include <functional>
#include <string>
#include <vector>
#include <mutex>

namespace p2t {

class ExceptionMonitor {
public:
  // Singleton instance
  static ExceptionMonitor& Instance() {
    static ExceptionMonitor instance;
    return instance;
  }
  
  // Register a callback to be called when an exception occurs
  using ExceptionCallback = std::function<void(const std::string&, const std::string&)>;
  void RegisterCallback(ExceptionCallback callback);
  
  // Unregister all callbacks
  void UnregisterAllCallbacks();
  
  // Notify all callbacks about an exception
  void NotifyException(const std::string& type, const std::string& message);
  
  // Enable/disable exception monitoring
  void Enable(bool enabled);
  bool IsEnabled() const;
  
private:
  ExceptionMonitor() : enabled_(true) {}
  ~ExceptionMonitor() = default;
  
  ExceptionMonitor(const ExceptionMonitor&) = delete;
  ExceptionMonitor& operator=(const ExceptionMonitor&) = delete;
  
  std::vector<ExceptionCallback> callbacks_;
  mutable std::mutex mutex_;
  bool enabled_;
};

} // namespace p2t