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

/**
 * @file trajectory_logger.hpp
 * @brief Minimal CSV time-series logger for sampled trajectories.
 *
 * Follows the CsvLogger style from mpc_examples/examples/utils/utils.hpp but
 * reduced to the data produced by a bare trajectory generator (no control,
 * no reference tracking). Each call to save() appends a single row with the
 * time stamp and the vehicle position, velocity and acceleration in the
 * caller's world frame.
 */

#ifndef GCOPTER_LIB__TRAJECTORY_LOGGER_HPP_
#define GCOPTER_LIB__TRAJECTORY_LOGGER_HPP_

#include <Eigen/Eigen>
#include <fstream>
#include <string>

namespace gcopter_lib {

class TrajectoryLogger {
public:
  /**
   * @brief Open @p file_name for writing and emit the CSV header.
   *        Columns: t, px, py, pz, vx, vy, vz, ax, ay, az.
   * @throws std::runtime_error if the file cannot be opened.
   */
  explicit TrajectoryLogger(const std::string& file_name);
  ~TrajectoryLogger();

  TrajectoryLogger(const TrajectoryLogger&)            = delete;
  TrajectoryLogger& operator=(const TrajectoryLogger&) = delete;

  /**
   * @brief Append one sample to the CSV file.
   * @param t            [s] time stamp since trajectory start
   * @param position     [m]
   * @param velocity     [m/s]
   * @param acceleration [m/s^2]
   */
  void save(double t,
            const Eigen::Vector3d& position,
            const Eigen::Vector3d& velocity,
            const Eigen::Vector3d& acceleration);

  /// Flush and close the underlying stream. Safe to call multiple times.
  void close();

private:
  void writeVector(const Eigen::Vector3d& v, bool trailing_comma);

  std::string file_name_;
  std::ofstream file_;
};

}  // namespace gcopter_lib

#endif  // GCOPTER_LIB__TRAJECTORY_LOGGER_HPP_
