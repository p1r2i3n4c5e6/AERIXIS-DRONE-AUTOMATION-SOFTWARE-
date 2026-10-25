# Copyright 2026 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Tests for the high-level Python ``Trajectory`` wrapper and YAML loader."""

from pathlib import Path

import numpy as np
import pytest

from mav_trajectory_generation_py import (
    GeneratorConfig,
    PyGeneratorConfig,
    Solver,
    Trajectory,
    TrajectoryPoint,
    load_generator_config,
)


def _three_waypoints():
    return [
        np.array([0.0, 0.0, 1.0]),
        np.array([5.0, 2.0, 1.5]),
        np.array([8.0, 0.0, 2.0]),
    ]


def test_py_generator_config_to_native():
    gc = PyGeneratorConfig(derivative_to_optimize=2, solver="nonlinear", a_max=3.0)
    native = gc.to_native()
    assert isinstance(native, GeneratorConfig)
    assert native.optimization.derivative_to_optimize == 2
    assert native.optimization.solver == Solver.nonlinear
    assert native.optimization.a_max == pytest.approx(3.0)


def test_py_generator_config_rejects_unknown_solver():
    with pytest.raises(ValueError):
        PyGeneratorConfig(solver="bogus").to_native()


def test_load_generator_config(tmp_path: Path):
    yaml_text = """
optimization:
  derivative_to_optimize: 3
  solver: nonlinear
  a_max: 5.5
  nl_max_iterations: 777
"""
    cfg_path = tmp_path / "cfg.yaml"
    cfg_path.write_text(yaml_text)
    cfg = load_generator_config(cfg_path)
    assert isinstance(cfg, GeneratorConfig)
    assert cfg.optimization.derivative_to_optimize == 3
    assert cfg.optimization.solver == Solver.nonlinear
    assert cfg.optimization.a_max == pytest.approx(5.5)
    assert cfg.optimization.nl_max_iterations == 777


def test_load_generator_config_defaults_on_empty(tmp_path: Path):
    cfg_path = tmp_path / "empty.yaml"
    cfg_path.write_text("optimization: {}\n")
    cfg = load_generator_config(cfg_path)
    default = GeneratorConfig()
    assert cfg.optimization.derivative_to_optimize == default.optimization.derivative_to_optimize
    assert cfg.optimization.a_max == default.optimization.a_max


def test_trajectory_generate_and_evaluate():
    traj = Trajectory()
    assert traj.generate(_three_waypoints(), 3.0) is True
    assert traj.is_valid
    assert traj.duration > 0.0

    point = traj.evaluate(0.5 * traj.duration)
    assert isinstance(point, TrajectoryPoint)
    assert point.position.shape == (3,)
    assert point.velocity.shape == (3,)
    assert point.acceleration.shape == (3,)


def test_trajectory_rejects_bad_inputs():
    traj = Trajectory()
    with pytest.raises(ValueError):
        traj.generate(_three_waypoints(), 0.0)
    with pytest.raises(ValueError):
        traj.generate([np.zeros(3)], 3.0)


def test_trajectory_accepts_py_generator_config():
    cfg = PyGeneratorConfig(solver="nonlinear", nl_max_iterations=500)
    traj = Trajectory(cfg)
    assert traj.generate(_three_waypoints(), 3.0) is True
    assert traj.is_valid


def test_trajectory_accepts_native_generator_config():
    cfg = GeneratorConfig()
    cfg.optimization.solver = Solver.nonlinear
    cfg.optimization.nl_max_iterations = 500
    traj = Trajectory(cfg)
    assert traj.generate(_three_waypoints(), 3.0) is True
    assert traj.is_valid
