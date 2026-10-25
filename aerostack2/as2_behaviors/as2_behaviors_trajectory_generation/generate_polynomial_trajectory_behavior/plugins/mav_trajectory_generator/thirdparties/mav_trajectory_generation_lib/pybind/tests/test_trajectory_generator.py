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


"""Smoke tests over the native pybind11 bindings."""

from pathlib import Path

import numpy as np
import pytest

from mav_trajectory_generation_py import (
    EndWaypoint,
    GeneratorConfig,
    OptimizationConfig,
    SegmentPolynomial,
    Solver,
    Spline,
    TrajectoryGenerator,
    TrajectorySample,
    Waypoint,
)


def _three_waypoints_with_rest_endpoints():
    return [
        EndWaypoint(np.array([0.0, 0.0, 1.0])),
        Waypoint(np.array([5.0, 2.0, 1.5])),
        EndWaypoint(np.array([8.0, 0.0, 2.0])),
    ]


def test_default_config_roundtrip():
    cfg = GeneratorConfig()
    assert cfg.optimization.derivative_to_optimize == 4
    assert cfg.optimization.solver == Solver.linear
    assert cfg.optimization.a_max > 0.0


def test_generate_linear_succeeds():
    gen = TrajectoryGenerator()
    assert gen.generate(_three_waypoints_with_rest_endpoints(), 3.0) is True
    assert gen.is_valid()
    assert gen.duration() > 0.0
    assert gen.min_time() == pytest.approx(0.0)


def test_generate_fails_with_bad_inputs():
    gen = TrajectoryGenerator()
    assert gen.generate([Waypoint(np.zeros(3))], 3.0) is False
    assert not gen.is_valid()
    assert gen.generate(_three_waypoints_with_rest_endpoints(), 0.0) is False
    assert gen.generate(_three_waypoints_with_rest_endpoints(), -1.0) is False


def test_evaluate_returns_trajectory_sample():
    gen = TrajectoryGenerator()
    assert gen.generate(_three_waypoints_with_rest_endpoints(), 3.0) is True
    sample = gen.evaluate(0.5 * gen.duration())
    assert isinstance(sample, TrajectorySample)
    assert sample.position.shape == (3,)
    assert sample.velocity.shape == (3,)
    assert sample.acceleration.shape == (3,)


def test_endpoints_match_waypoints():
    waypoints = _three_waypoints_with_rest_endpoints()
    gen = TrajectoryGenerator()
    assert gen.generate(waypoints, 3.0) is True
    start = gen.evaluate(gen.min_time())
    end = gen.evaluate(gen.max_time())
    np.testing.assert_allclose(start.position, waypoints[0].position, atol=5e-3)
    np.testing.assert_allclose(end.position, waypoints[-1].position, atol=5e-3)
    # EndWaypoint imposes zero boundary velocity.
    assert np.linalg.norm(start.velocity) < 5e-2
    assert np.linalg.norm(end.velocity) < 5e-2


def test_nonlinear_solver_runs_and_converges():
    cfg = GeneratorConfig()
    cfg.optimization.solver = Solver.nonlinear
    cfg.optimization.nl_max_iterations = 500
    gen = TrajectoryGenerator(cfg)
    assert gen.generate(_three_waypoints_with_rest_endpoints(), 3.0) is True
    assert gen.is_valid()


def test_evaluate_derivative_matches_components():
    gen = TrajectoryGenerator()
    assert gen.generate(_three_waypoints_with_rest_endpoints(), 3.0) is True
    t = 0.5 * gen.duration()
    sample = gen.evaluate(t)
    np.testing.assert_allclose(gen.evaluate_derivative(t, 0), sample.position, atol=1e-9)
    np.testing.assert_allclose(gen.evaluate_derivative(t, 1), sample.velocity, atol=1e-9)
    np.testing.assert_allclose(gen.evaluate_derivative(t, 2), sample.acceleration, atol=1e-9)


def test_named_accessors_match_evaluate():
    gen = TrajectoryGenerator()
    assert gen.generate(_three_waypoints_with_rest_endpoints(), 3.0) is True
    t = 0.5 * gen.duration()
    sample = gen.evaluate(t)
    np.testing.assert_allclose(gen.position(t), sample.position, atol=1e-12)
    np.testing.assert_allclose(gen.velocity(t), sample.velocity, atol=1e-12)
    np.testing.assert_allclose(gen.acceleration(t), sample.acceleration, atol=1e-12)
    # jerk == evaluate_derivative(t, 3)
    np.testing.assert_allclose(gen.jerk(t), gen.evaluate_derivative(t, 3), atol=1e-12)


def test_max_velocity_and_acceleration_are_finite():
    cfg = GeneratorConfig()
    cfg.optimization.solver = Solver.nonlinear
    cfg.optimization.nl_max_iterations = 500
    gen = TrajectoryGenerator(cfg)
    v_max = 3.0
    assert gen.generate(_three_waypoints_with_rest_endpoints(), v_max) is True
    v_peak = gen.max_velocity()
    a_peak = gen.max_acceleration()
    assert v_peak > 0.0
    assert a_peak > 0.0
    assert np.isfinite(v_peak)
    assert np.isfinite(a_peak)
    # Nonlinear enforces v <= v_max with NLopt inequality tolerance.
    assert v_peak <= v_max * 1.2


def test_accessors_throw_before_generate():
    gen = TrajectoryGenerator()
    assert not gen.is_valid()
    with pytest.raises(RuntimeError):
        gen.duration()
    with pytest.raises(RuntimeError):
        gen.evaluate(0.0)
    with pytest.raises(RuntimeError):
        gen.position(0.0)
    with pytest.raises(RuntimeError):
        gen.max_velocity()


def test_spline_getter_agrees_with_evaluate():
    """spline() must be consistent with evaluate()."""
    waypoints = _three_waypoints_with_rest_endpoints()
    gen = TrajectoryGenerator()
    assert gen.generate(waypoints, 3.0) is True

    sp = gen.spline()
    assert isinstance(sp, Spline)
    segs = sp.segments
    times = sp.segment_times()
    knots = sp.breakpoints()

    assert len(segs) == len(waypoints) - 1
    assert len(times) == len(segs)
    assert len(knots) == len(segs) + 1
    assert all(isinstance(s, SegmentPolynomial) for s in segs)

    for s, t in zip(segs, times):
        assert s.duration == pytest.approx(t, abs=1e-12)
    for i, t in enumerate(times):
        assert knots[i + 1] - knots[i] == pytest.approx(t, abs=1e-12)
    assert knots[0] == pytest.approx(gen.min_time(), abs=1e-12)
    assert knots[-1] == pytest.approx(gen.max_time(), abs=1e-12)
    assert sp.duration() == pytest.approx(gen.duration(), abs=1e-12)

    # Reconstruct position at tau inside segment 0 and compare to evaluate().
    coeffs = np.asarray(segs[0].coefficients)
    assert coeffs.shape[0] == 3
    N = coeffs.shape[1]
    tau = 0.3 * segs[0].duration
    t_global = knots[0] + tau
    powers = np.array([tau ** k for k in range(N)])
    reconstructed = coeffs @ powers
    np.testing.assert_allclose(
        reconstructed, gen.evaluate(t_global).position, atol=1e-9,
    )


def test_spline_throws_when_invalid():
    gen = TrajectoryGenerator()
    assert not gen.is_valid()
    with pytest.raises(RuntimeError):
        gen.spline()


def test_set_spline_round_trip():
    """Snapshot a spline and evaluate it on a fresh generator via set_spline()."""
    gen = TrajectoryGenerator()
    assert gen.generate(_three_waypoints_with_rest_endpoints(), 3.0)
    sp = gen.spline()

    gen2 = TrajectoryGenerator()
    assert not gen2.is_valid()
    assert gen2.set_spline(sp) is True
    assert gen2.is_valid()
    assert gen2.duration() == pytest.approx(gen.duration(), abs=1e-12)

    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        t = gen.min_time() + frac * gen.duration()
        a = gen.evaluate(t)
        b = gen2.evaluate(t)
        np.testing.assert_allclose(a.position, b.position, atol=1e-9)
        np.testing.assert_allclose(a.velocity, b.velocity, atol=1e-9)
        np.testing.assert_allclose(a.acceleration, b.acceleration, atol=1e-9)


def test_set_spline_rejects_wrong_shape():
    bad = Spline()
    seg = SegmentPolynomial()
    seg.duration = 1.0
    seg.coefficients = np.zeros((3, 4))  # N != 10
    bad.segments = [seg]

    gen = TrajectoryGenerator()
    assert gen.set_spline(bad) is False
    assert not gen.is_valid()

    empty = Spline()
    assert gen.set_spline(empty) is False
    assert not gen.is_valid()


def test_spline_csv_round_trip(tmp_path: Path):
    """Save + load must round-trip coefficients to ~17 digits of precision."""
    gen = TrajectoryGenerator()
    assert gen.generate(_three_waypoints_with_rest_endpoints(), 3.0)
    a = gen.spline()

    path = tmp_path / "spline.csv"
    assert a.save_to_csv(str(path)) is True

    b = Spline()
    assert b.load_from_csv(str(path)) is True
    assert len(a.segments) == len(b.segments)
    assert b.start_time == pytest.approx(a.start_time, abs=1e-12)
    assert b.duration() == pytest.approx(a.duration(), abs=1e-12)
    for sa, sb in zip(a.segments, b.segments):
        assert sb.duration == pytest.approx(sa.duration, abs=1e-12)
        np.testing.assert_allclose(
            np.asarray(sb.coefficients), np.asarray(sa.coefficients), atol=1e-12,
        )

    # Bytewise reload into a fresh generator and verify evaluate() parity.
    gen2 = TrajectoryGenerator()
    assert gen2.set_spline(b)
    t = 0.5 * gen.duration()
    np.testing.assert_allclose(
        gen.evaluate(t).position, gen2.evaluate(t).position, atol=1e-9,
    )


def test_spline_load_rejects_bad_inputs(tmp_path: Path):
    s = Spline()
    assert s.load_from_csv(str(tmp_path / "does_not_exist.csv")) is False

    path = tmp_path / "bad_spline.csv"
    path.write_text(
        "# spline_format_version: 1\n"
        "# start_time: 0\n"
        "# dimension: 2\n"              # unsupported
        "# polynomial_order: 10\n"
        "duration\n"
    )
    assert s.load_from_csv(str(path)) is False


def test_chaining_preserves_initial_velocity():
    """A Waypoint with non-zero velocity at t=0 is honoured by the optimiser."""
    first = [
        EndWaypoint(np.array([0.0, 0.0, 1.0])),
        EndWaypoint(np.array([5.0, 0.0, 1.0])),
    ]
    gen = TrajectoryGenerator()
    assert gen.generate(first, 3.0) is True

    # Seed the second trajectory's start with the velocity of the first's end —
    # here it is ~0 since both are EndWaypoints, but we still exercise the path.
    v_final = gen.evaluate(gen.max_time()).velocity
    chained_start = Waypoint(np.array([5.0, 0.0, 1.0]))
    chained_start.velocity = v_final
    second = [chained_start, EndWaypoint(np.array([0.0, 0.0, 1.0]))]
    assert gen.generate(second, 3.0) is True
    np.testing.assert_allclose(gen.evaluate(gen.min_time()).velocity, v_final, atol=1e-6)
