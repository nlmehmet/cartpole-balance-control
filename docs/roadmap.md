# Development roadmap

The governing [ADR-0001](decisions/ADR-0001-simulation-contracts.md) remains
Proposed. Each phase is a separately reviewed change; PR-001 does not implement
the proposed simulator or alter the model.

| Phase | Scope | Exit criterion |
| --- | --- | --- |
| PR-001 | Architecture, contributor rules, deterministic characterization fixtures and tests | Legacy numerical results captured; mathematical checks pass; production files unchanged |
| PR-002 | Installable package, explicit entry points, output paths and dependency tooling | Equivalent legacy trajectories; side-effect-free imports; CLI smoke checks |
| PR-003 | Time/result/termination contracts and input validation | Endpoint and failure-bracket tests pass; legacy defects isolated or retired; numeric changes reviewed |
| PR-004 | Controller lifecycle, gain/weight validation and anti-windup | Reset and nonzero-integral saturation tests; sampled stability and integration-convergence evidence |
| PR-005 | Provenance exports, metric definitions, rendering checks and CI | One command reproduces numeric reports and temporary artifacts |

## PR-001 acceptance and validation

Run `python3 -B -m unittest discover -s tests -v` at the repository root.
Tests require the existing NumPy and SciPy dependencies, not pytest or rendering.
See [fixture instructions](../tests/fixtures/README.md) for immutable-baseline
policy, provenance and numeric tolerances. Generated figures are not test oracles.
No production code, dependency declarations, gains, equations or committed
results are changed. No publishing action is part of this phase.

## Remaining engineering questions

Agree corrected boundary equality semantics and robust equilibrium metrics
before PR-003. Demonstrate RK4 convergence on smooth held-input problems;
analyze disturbance discontinuities separately. Verify sampled closed-loop
stability at supported control periods. Design anti-windup without conflating
external disturbances with actuator saturation. Pin dependencies and establish
cross-platform tolerance evidence before claiming broad reproducibility.
