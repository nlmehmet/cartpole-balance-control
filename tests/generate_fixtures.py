"""Explicit baseline proposal generator.

Tests invoke this script only to verify rejection of an existing output path.
Ordinary test runs do not regenerate baseline fixtures.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import platform
import shlex
import subprocess
import sys

import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from controllers import LQR, PID
from dynamics import Params
from simulate import is_success, simulate


def controller(spec, p):
    if spec["kind"] == "PID":
        return PID(**spec["parameters"])
    return LQR(p, Q=spec["Q"], R=spec["R"])


def build():
    p = Params()
    specs = {
        "angle_pd": {"kind": "PID", "parameters": dict(kp=60., ki=0., kd=10., kx=0., kv=0., dt=.005)},
        "position_pd": {"kind": "PID", "parameters": dict(kp=60., ki=0., kd=10., kx=2., kv=4., dt=.005)},
        "lqr": {"kind": "LQR", "Q": np.diag([1., 1., 10., 1.]).tolist(), "R": [[.1]]},
    }
    cases = []
    for name, spec in specs.items():
        for push in (None, [3., .2, 15.]):
            config = dict(T=10., dt=.005, push=push, u_max=30.)
            s0 = [0., 0., float(np.deg2rad(10)), 0.]
            t, s, u, failure = simulate(controller(spec, p), s0, p, **config)
            indices = [0, 1, 100, 599, 600, 639, 640, 1000, 1272, 1273, 1999]
            cases.append(dict(name=name + ("_push" if push else "_recovery"), controller=name,
                              initial_state=s0, simulation=config, indices=indices,
                              times=t[indices].tolist(), states=s[indices].tolist(),
                              forces=u[indices].tolist(), failed_at=failure,
                              peak_force=float(max(abs(u))), success=bool(is_success(s, failure))))
    angles = list(range(2, 62, 2))
    masks = {}
    for name, spec in specs.items():
        mask = []
        for angle in angles:
            _, s, _, failure = simulate(controller(spec, p), [0, 0, np.deg2rad(angle), 0], p,
                                        T=10., dt=.005, u_max=30.)
            mask.append(bool(is_success(s, failure)))
        masks[name] = mask
    tuning = []
    for q in [1, 10, 100, 1000]:
        spec = dict(kind="LQR", Q=np.diag([1., 1., float(q), 1.]).tolist(), R=[[.1]])
        c = controller(spec, p)
        s0 = [0., 0., float(np.deg2rad(10)), 0.]
        config = dict(T=6., dt=.005, push=None, u_max=30.)
        _, _, u, failure = simulate(c, s0, p, **config)
        tuning.append(dict(q_theta=q, controller=spec, initial_state=s0, simulation=config,
                           gain=c.K.tolist(), peak_force=float(max(abs(u))), failed_at=failure))
    return dict(schema_version=1, source_commit=subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        source_sha256={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                       for path in sorted((ROOT / "src").glob("*.py"))},
        versions=dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
                      matplotlib=version("matplotlib"), pillow=version("pillow")),
        provenance=dict(generator="tests/generate_fixtures.py", generated_at_utc=datetime.now(timezone.utc).isoformat(),
                        command=shlex.join(["python3", "-B", *sys.argv]),
                        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                        method="Direct calls to unmodified legacy simulate; fresh controller per trial; no rendering or randomness"),
        tolerances=dict(rtol=1e-8, atol=1e-10, time_atol=1e-12, jacobian_atol=1e-8, care_atol=1e-9),
        physical_parameters=asdict(p), controllers=specs, trajectories=cases,
        basin=dict(angles_deg=angles, initial_state_template=[0., 0., "radians(angle_deg)", 0.],
                   simulation=dict(T=10., dt=.005, push=None, u_max=30.), masks=masks), tuning=tuning)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output exists; use a new proposal path and review changes explicitly")
    data = build()
    with args.output.open("x") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")
