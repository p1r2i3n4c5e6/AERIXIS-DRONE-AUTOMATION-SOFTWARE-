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
 * @file types.cpp
 *
 * Implementation of the Spline accessors and CSV persistence routines.
 */

#include "trajectory_generator_jerk_limited/types.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace trajectory_generator_jerk_limited {

namespace {

constexpr int kSplineFormatVersion = 1;
constexpr int kDimension           = 3;

bool parseMetaLine(const std::string& line, std::string* key, std::string* value) {
  if (line.empty() || line.front() != '#') return false;
  const auto colon = line.find(':');
  if (colon == std::string::npos) return false;
  std::string k = line.substr(1, colon - 1);
  std::string v = line.substr(colon + 1);
  auto trim     = [](std::string& s) {
    const char* ws = " \t\r\n";
    const auto l   = s.find_first_not_of(ws);
    s              = (l == std::string::npos) ? std::string{} : s.substr(l);
    const auto r   = s.find_last_not_of(ws);
    if (r != std::string::npos) s.erase(r + 1);
  };
  trim(k);
  trim(v);
  if (k.empty() || v.empty()) return false;
  *key   = k;
  *value = v;
  return true;
}

bool parseCsvRow(const std::string& line, std::vector<double>* out) {
  out->clear();
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    try {
      out->push_back(std::stod(cell));
    } catch (const std::exception&) {
      return false;
    }
  }
  return !out->empty();
}

}  // namespace

std::vector<double> Spline::segmentTimes() const {
  std::vector<double> out;
  out.reserve(segments.size());
  for (const auto& s : segments) {
    out.push_back(s.duration);
  }
  return out;
}

std::vector<double> Spline::breakpoints() const {
  std::vector<double> out;
  if (segments.empty()) return out;
  out.reserve(segments.size() + 1);
  double t = start_time;
  out.push_back(t);
  for (const auto& s : segments) {
    t += s.duration;
    out.push_back(t);
  }
  return out;
}

double Spline::duration() const {
  double total = 0.0;
  for (const auto& s : segments) total += s.duration;
  return total;
}

bool Spline::saveToCsv(const std::string& path) const {
  std::ofstream out(path);
  if (!out) {
    std::cerr << "[trajectory_generator_jerk_limited] Spline::saveToCsv(): cannot open '" << path
              << "'\n";
    return false;
  }
  const int dim =
      segments.empty() ? kDimension : static_cast<int>(segments.front().coefficients.rows());

  // The one-shot generator can produce short boundary segments with fewer fitted
  // samples than the requested polynomial order. Persist with the maximum order
  // and zero-pad shorter segments so the CSV keeps a rectangular shape.
  int n_poly = 0;
  for (const auto& sp : segments) {
    n_poly = std::max(n_poly, static_cast<int>(sp.coefficients.cols()));
  }

  out << "# spline_format_version: " << kSplineFormatVersion << "\n";
  out.precision(std::numeric_limits<double>::max_digits10);
  out << "# start_time: " << start_time << "\n";
  out << "# dimension: " << dim << "\n";
  out << "# polynomial_order: " << n_poly << "\n";

  out << "duration";
  const char axis_name[] = {'x', 'y', 'z'};
  for (int d = 0; d < dim; ++d) {
    for (int k = 0; k < n_poly; ++k) {
      const char axis = (d < 3) ? axis_name[d] : '?';
      out << ",c" << axis << k;
    }
  }
  out << "\n";

  for (const auto& sp : segments) {
    if (sp.coefficients.rows() != dim || sp.coefficients.cols() > n_poly) {
      std::cerr << "[trajectory_generator_jerk_limited] Spline::saveToCsv(): inconsistent segment "
                   "shapes; file may be malformed\n";
      return false;
    }
    out << sp.duration;
    for (int d = 0; d < dim; ++d) {
      for (int k = 0; k < n_poly; ++k) {
        const double coef = (k < sp.coefficients.cols()) ? sp.coefficients(d, k) : 0.0;
        out << "," << coef;
      }
    }
    out << "\n";
  }
  return static_cast<bool>(out);
}

bool Spline::loadFromCsv(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    std::cerr << "[trajectory_generator_jerk_limited] Spline::loadFromCsv(): cannot open '" << path
              << "'\n";
    return false;
  }

  std::string line;
  int version = -1, dim = -1, n_poly = -1;
  double start_time_meta = 0.0;

  while (std::getline(in, line)) {
    if (!line.empty() && line.front() == '#') {
      std::string key, value;
      if (!parseMetaLine(line, &key, &value)) continue;
      try {
        if (key == "spline_format_version")
          version = std::stoi(value);
        else if (key == "start_time")
          start_time_meta = std::stod(value);
        else if (key == "dimension")
          dim = std::stoi(value);
        else if (key == "polynomial_order")
          n_poly = std::stoi(value);
      } catch (const std::exception&) {
        std::cerr << "[trajectory_generator_jerk_limited] Spline::loadFromCsv(): bad metadata '"
                  << line << "'\n";
        return false;
      }
    } else {
      break;
    }
  }
  if (version != kSplineFormatVersion) {
    std::cerr << "[trajectory_generator_jerk_limited] Spline::loadFromCsv(): unknown format "
                 "version "
              << version << " (expected " << kSplineFormatVersion << ")\n";
    return false;
  }
  if (dim != kDimension) {
    std::cerr << "[trajectory_generator_jerk_limited] Spline::loadFromCsv(): unsupported "
                 "dimension "
              << dim << " (only " << kDimension << " is supported)\n";
    return false;
  }
  if (n_poly <= 0) {
    std::cerr << "[trajectory_generator_jerk_limited] Spline::loadFromCsv(): invalid "
                 "polynomial_order "
              << n_poly << "\n";
    return false;
  }

  const int expected_cols = 1 + dim * n_poly;
  std::vector<SegmentPolynomial> parsed;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::vector<double> cells;
    if (!parseCsvRow(line, &cells)) {
      std::cerr << "[trajectory_generator_jerk_limited] Spline::loadFromCsv(): malformed row '"
                << line << "'\n";
      return false;
    }
    if (static_cast<int>(cells.size()) != expected_cols) {
      std::cerr << "[trajectory_generator_jerk_limited] Spline::loadFromCsv(): row has "
                << cells.size() << " cells, expected " << expected_cols << "\n";
      return false;
    }
    SegmentPolynomial sp;
    sp.duration     = cells[0];
    sp.coefficients = Eigen::MatrixXd::Zero(dim, n_poly);
    std::size_t idx = 1;
    for (int d = 0; d < dim; ++d) {
      for (int k = 0; k < n_poly; ++k) {
        sp.coefficients(d, k) = cells[idx++];
      }
    }
    parsed.push_back(std::move(sp));
  }
  if (parsed.empty()) {
    std::cerr << "[trajectory_generator_jerk_limited] Spline::loadFromCsv(): no segments found in '"
              << path << "'\n";
    return false;
  }

  this->start_time = start_time_meta;
  this->segments   = std::move(parsed);
  return true;
}

}  // namespace trajectory_generator_jerk_limited
