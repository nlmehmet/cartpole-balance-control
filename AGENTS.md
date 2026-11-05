# Contributor contract

## Scope and invariants

Work only in the requested repository. Use its existing isolated checkout; create
a worktree only when explicitly requested. Do not push, merge, or open a PR
without authorization.

State order is `[x, x_dot, theta, theta_dot]` in m, m/s, rad, rad/s.
Force is in N. Positive angle leans toward positive x from upright; positive
force initially gives positive cart acceleration and negative angular
acceleration. The plant is a point mass on a massless rod. Changes to equations,
signs, gains, or units require a derivation and regression evidence.

## Architecture and lifecycle

Follow [architecture.md](docs/architecture.md) and
[ADR-0001](docs/decisions/ADR-0001-simulation-contracts.md), whose status remains
Proposed. Distinguish existing behavior from proposed contracts.
Keep numerical core imports free of plotting, CLI, and filesystem side effects.
Never update stateful controllers inside integrator stages. Each independent
trial uses a fresh controller (or an explicit reset once that API exists).
Distinguish requested force, saturated actuator force, external disturbance,
and total plant force. Do not infer nonlinear stability from linear eigenvalues.

## Validation and baselines

Run `python3 -B -m unittest discover -s tests -v` from the repository root.
Characterization tests preserve the audited legacy implementation; tests named
`test_legacy_*` document defects, not requirements for its future replacement.
Migrate those tests deliberately when correcting contracts.
Never weaken assertions or silently regenerate fixtures to resolve a failure.
Investigate the source, dependency versions, and numerical tolerances first.
Fixture proposals must use the explicit generator, a new output path, and review
of the numeric changes and provenance. Do not overwrite the accepted baseline.
Keep generated figures and validation outputs outside committed `results/`.

PR-001 leaves `src/`, `requirements.txt`, controller gains, physical equations,
and committed figures unchanged. Report passed, failed, skipped, and unrun
checks separately. Later behavior changes require updated documentation and
explicitly reviewed regression changes.
