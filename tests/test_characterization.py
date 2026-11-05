"""Legacy characterization, not acceptance criteria for the corrected simulator.

Jacobian and CARE checks establish mathematical consistency. Tests reusing RK4
establish simulation orchestration, not independent integration accuracy.
"""
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

import numpy as np
from scipy.linalg import solve_continuous_are

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from controllers import LQR, PID
from dynamics import Params, linearized_matrices, nonlinear_rhs, rk4_step
from simulate import is_success, simulate
import simulate as simulation_module

FIXTURE = json.loads((Path(__file__).parent / "fixtures/legacy-v1.json").read_text())
TOL = FIXTURE["tolerances"]


def make_controller(spec, p):
    if spec["kind"] == "PID":
        return PID(**spec["parameters"])
    return LQR(p, Q=spec["Q"], R=spec["R"])


class PhysicsTests(unittest.TestCase):
    def setUp(self):
        self.p = Params()

    def test_upright_equilibrium(self):
        np.testing.assert_array_equal(nonlinear_rhs(np.zeros(4), 0., self.p), np.zeros(4))

    def test_finite_difference_linearization(self):
        # Multiple physical configurations avoid validating only hard-coded defaults.
        for p in (self.p, Params(M=2., m=.3, l=.8, g=9.7, b=.2)):
            with self.subTest(params=p):
                h = 1e-6
                z = np.zeros(4)
                a, b = linearized_matrices(p)
                jac = np.column_stack([(nonlinear_rhs(h*e, 0., p)-nonlinear_rhs(-h*e, 0., p))/(2*h)
                                       for e in np.eye(4)])
                inp = (nonlinear_rhs(z, h, p)-nonlinear_rhs(z, -h, p))/(2*h)
                np.testing.assert_allclose(a, jac, rtol=0, atol=TOL["jacobian_atol"])
                np.testing.assert_allclose(b[:, 0], inp, rtol=0, atol=TOL["jacobian_atol"])

    def test_positive_force_angular_response(self):
        rhs = nonlinear_rhs(np.zeros(4), 2., self.p)
        self.assertGreater(rhs[1], 0.)
        self.assertLess(rhs[3], 0.)
        self.assertEqual(rhs[1], 2./self.p.M)
        self.assertEqual(rhs[3], -2./(self.p.M*self.p.l))

    def test_horizontal_translation_invariance(self):
        s = np.array([.3, -.2, .17, .4])
        shifted = s + np.array([123., 0., 0., 0.])
        np.testing.assert_array_equal(nonlinear_rhs(s, 1.2, self.p), nonlinear_rhs(shifted, 1.2, self.p))

    def test_care_residual_and_lqr_stability(self):
        """Check CARE consistency using the same solver, not an independent oracle."""
        a, b = linearized_matrices(self.p)
        for q in (1., 10., 100., 1000.):
            with self.subTest(q_theta=q):
                c = LQR(self.p, Q=np.diag([1., 1., q, 1.]))
                p = solve_continuous_are(a, b, c.Q, c.R)
                feedback = np.linalg.solve(c.R, b.T @ p)
                residual = a.T@p + p@a - p@b@feedback + c.Q
                scale = max(1., np.linalg.norm(c.Q), np.linalg.norm(a.T@p), np.linalg.norm(p@b@feedback))
                self.assertLess(np.linalg.norm(residual)/scale, TOL["care_atol"])
                self.assertGreater(np.linalg.eigvalsh(p).min(), 0.)
                np.testing.assert_allclose(c.K, feedback.ravel(), rtol=TOL["rtol"], atol=TOL["atol"])
                self.assertLess(np.linalg.eigvals(a-b@c.K[None, :]).real.max(), 0.)


class LegacyRegressionTests(unittest.TestCase):
    def setUp(self):
        self.p = Params(**FIXTURE["physical_parameters"])

    def assert_numeric(self, actual, expected):
        np.testing.assert_allclose(actual, expected, rtol=TOL["rtol"], atol=TOL["atol"])

    def test_fixture_generator_refuses_overwrite(self):
        path = Path(__file__).parent / "fixtures/legacy-v1.json"
        before = path.read_bytes()
        result = subprocess.run([sys.executable, "-B", str(Path(__file__).parent / "generate_fixtures.py"),
                                 "--output", str(path)], cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("Output exists", result.stderr)
        self.assertEqual(path.read_bytes(), before)

    def test_legacy_trajectories_and_disturbances(self):
        for case in FIXTURE["trajectories"]:
            with self.subTest(case=case["name"]):
                spec = FIXTURE["controllers"][case["controller"]]
                t, s, u, failure = simulate(make_controller(spec, self.p), case["initial_state"], self.p,
                                             **case["simulation"])
                indices = case["indices"]
                self.assert_numeric(s[indices], case["states"])
                self.assert_numeric(u[indices], case["forces"])
                np.testing.assert_allclose(t[indices], case["times"], rtol=0, atol=TOL["time_atol"])
                self.assertEqual(failure, case["failed_at"])
                self.assert_numeric(max(abs(u)), case["peak_force"])
                self.assertEqual(bool(is_success(s, failure)), case["success"])

    def test_legacy_basin_success_masks(self):
        basin = FIXTURE["basin"]
        for name, spec in FIXTURE["controllers"].items():
            actual = []
            for angle in basin["angles_deg"]:
                _, s, _, failure = simulate(make_controller(spec, self.p), [0, 0, np.deg2rad(angle), 0],
                                             self.p, **basin["simulation"])
                actual.append(bool(is_success(s, failure)))
            with self.subTest(controller=name):
                self.assertEqual(actual, basin["masks"][name])

    def test_legacy_lqr_tuning_metrics(self):
        for case in FIXTURE["tuning"]:
            with self.subTest(q_theta=case["q_theta"]):
                c = make_controller(case["controller"], self.p)
                _, _, u, failure = simulate(c, case["initial_state"], self.p, **case["simulation"])
                self.assert_numeric(c.K, case["gain"])
                self.assert_numeric(max(abs(u)), case["peak_force"])
                self.assertEqual(failure, case["failed_at"])

    def test_legacy_endpoint_omission_and_horizon_truncation(self):
        dt = .01
        s0 = np.array([0., .1, .1, 0.])
        calls = []
        def control(s):
            calls.append(s.copy())
            return 1.
        t, s, _, _ = simulate(control, s0, self.p, T=.035, dt=dt)
        self.assertEqual(len(calls), 3)
        self.assert_numeric(t, [0., .01, .02])
        endpoint = s0
        for i in range(3):
            self.assert_numeric(s[i], endpoint)
            endpoint = rk4_step(endpoint, 1., self.p, dt)
        self.assertGreater(np.linalg.norm(endpoint-s[-1]), 1e-5)

    def test_legacy_failure_timestamp_and_padding(self):
        s0 = [2.399, 1., 0., 0.]
        t, s, u, failure = simulate(lambda _: 0., s0, self.p, T=.04, dt=.01)
        endpoint = rk4_step(np.array(s0), 0., self.p, .01)
        self.assertGreater(endpoint[0], 2.4)
        self.assertEqual(failure, 0.)  # Detects endpoint .01 but reports start 0.
        self.assert_numeric(s[0], s0)
        self.assert_numeric(s[1:], np.tile(endpoint, (3, 1)))
        self.assert_numeric(u, np.zeros(4))
        self.assert_numeric(t, [0., .01, .02, .03])

    def test_legacy_initial_and_final_step_failure(self):
        """Spy on actual RK4 advancement before legacy initial-state rejection."""
        initial = np.array([2.5, 1., 0., 0.])
        endpoints = []
        def record_step(state, force, params, dt):
            endpoint = rk4_step(state, force, params, dt)
            endpoints.append(endpoint.copy())
            return endpoint
        with patch.object(simulation_module, "rk4_step", side_effect=record_step) as step:
            t, s, u, failure = simulate(lambda _: 0., initial, self.p, T=.03, dt=.01)
        self.assertEqual(step.call_count, 1)
        state, force, params, dt = step.call_args.args
        self.assert_numeric(state, initial)
        self.assertEqual(force, 0.)
        self.assertIs(params, self.p)
        self.assertEqual(dt, .01)
        self.assertGreater(endpoints[0][0], initial[0])
        self.assertEqual(failure, 0.)
        self.assert_numeric(t, [0., .01, .02])
        self.assert_numeric(s[0], initial)
        self.assert_numeric(s[1:], np.tile(endpoints[0], (2, 1)))
        self.assert_numeric(u, np.zeros(3))
        # A crossing on the only/final step still omits the failed endpoint.
        _, s, _, failure = simulate(lambda _: 0., [2.399, 1., 0., 0.], self.p, T=.01, dt=.01)
        self.assertEqual(failure, 0.)
        self.assertLess(s[-1, 0], 2.4)

    def test_legacy_sample_hold_saturation_and_disturbance_window(self):
        # dt=.125 gives exactly representable event boundaries.
        dt = .125
        initial = np.zeros(4)
        calls = []
        def control(s):
            calls.append(s.copy())
            return 100.
        t, s, u, failure = simulate(control, initial, self.p, T=.5, dt=dt,
                                     u_max=1., push=(.125, .125, 2.))
        self.assertIsNone(failure)
        self.assertEqual(len(calls), 4)
        expected = initial
        for i, total in enumerate([1., 3., 1., 1.]):
            self.assert_numeric(s[i], expected)
            expected = rk4_step(expected, total, self.p, dt)
        self.assert_numeric(u, np.ones(4))

    def test_legacy_pid_state_lifecycle(self):
        """Freeze plant states to isolate integral updates and PID-owned dt."""
        pid = PID(kp=0., kd=0., ki=1., dt=.02)
        s = np.array([0., 0., .1, 0.])
        # Absolute 1e-15 rad*s allows rounding in this four-term sum, not drift.
        for expected in (.002, .004):
            np.testing.assert_allclose(pid(s), expected, rtol=0, atol=1e-15)
            np.testing.assert_allclose(pid.integral, expected, rtol=0, atol=1e-15)
            self.assertEqual(pid.dt, .02)
        fresh = PID(kp=0., kd=0., ki=1., dt=.02)
        np.testing.assert_allclose(fresh(s), .002, rtol=0, atol=1e-15)
        # Two calls on the same fixed angle add .002 each despite simulator dt=.01.
        with patch.object(simulation_module, "rk4_step",
                          side_effect=lambda state, force, params, dt: state.copy()) as step:
            _, states, forces, failure = simulate(pid, s, self.p, T=.02, dt=.01)
        self.assertEqual(step.call_count, 2)
        self.assertIsNone(failure)
        self.assert_numeric(states, np.tile(s, (2, 1)))
        np.testing.assert_allclose(forces, [.006, .008], rtol=0, atol=1e-15)
        np.testing.assert_allclose(pid.integral, .008, rtol=0, atol=1e-15)
        self.assertEqual(pid.dt, .02)

    def test_legacy_success_is_final_angle_only(self):
        s = np.array([[2., 100., .019, 100.]])
        self.assertTrue(is_success(s, None))
        self.assertFalse(is_success(s, 0.))
        s[0, 2] = .02
        self.assertFalse(is_success(s, None))


if __name__ == "__main__":
    unittest.main()
