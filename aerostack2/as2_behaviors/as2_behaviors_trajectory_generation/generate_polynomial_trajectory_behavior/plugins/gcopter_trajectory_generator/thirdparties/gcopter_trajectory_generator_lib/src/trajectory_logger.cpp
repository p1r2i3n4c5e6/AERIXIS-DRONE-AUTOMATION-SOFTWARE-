// Copyright 2025 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "gcopter_lib/trajectory_logger.hpp"

#include <ios>
#include <stdexcept>

namespace gcopter_lib {

TrajectoryLogger::TrajectoryLogger(const std::string& file_name) : file_name_(file_name) {
  file_.open(file_name_, std::ios::out | std::ios::trunc);
  if (!file_.is_open()) {
    throw std::runtime_error("gcopter_lib::TrajectoryLogger: failed to open file '" + file_name_ +
                             "'");
  }
  file_ << "t,px,py,pz,vx,vy,vz,ax,ay,az\n";
}

TrajectoryLogger::~TrajectoryLogger() { close(); }

void TrajectoryLogger::save(double t,
                            const Eigen::Vector3d& position,
                            const Eigen::Vector3d& velocity,
                            const Eigen::Vector3d& acceleration) {
  if (!file_.is_open()) {
    return;
  }
  file_ << t << ',';
  writeVector(position, true);
  writeVector(velocity, true);
  writeVector(acceleration, false);
  file_ << '\n';
}

void TrajectoryLogger::close() {
  if (file_.is_open()) {
    file_.flush();
    file_.close();
  }
}

void TrajectoryLogger::writeVector(const Eigen::Vector3d& v, bool trailing_comma) {
  file_ << v.x() << ',' << v.y() << ',' << v.z();
  if (trailing_comma) {
    file_ << ',';
  }
}

}  // namespace gcopter_lib
