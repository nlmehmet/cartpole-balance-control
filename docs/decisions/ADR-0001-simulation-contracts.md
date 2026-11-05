# ADR-0001: Explicit sampled-control simulation and trajectory contracts

Status: Proposed

## Context

The audited scripts reproduce useful results, but timing, controller state,
failure padding, and output semantics are implicit. Current characterization
fixtures refer to commit `c678c0c624e50c532cae13a0378dcafc3b4c21ea`.
See [architecture](../architecture.md) for equations, SI units and signs.

## Decision proposal

1. Retain the point-mass plant, `[x, x_dot, theta, theta_dot]` ordering,
   upright equilibrium, and documented coordinate/force conventions.
2. Make sample-and-hold control explicit, initially with one controller update
   per integration interval. Hold input through all RK stages.
3. Pass elapsed control time explicitly to stateful controllers and provide
   reset. Independent experiments create fresh or reset controllers.
4. Return N+1 state samples, including the final endpoint, and N interval inputs.
5. Record requested actuator force, applied actuator force, external disturbance,
   and termination reason separately. Saturate actuator force before adding
   external disturbance.
6. Validate initial state and parameters, including finiteness; truncate returned
   trajectories at termination rather than fabricate continuation samples.
7. Initially report the detecting endpoint and crossing bracket. Do not claim
   an exact event time without event localization.
8. Require an explicit horizon policy, initially rejecting nonintegral T/dt.
9. Separate numerical experiments, metrics, rendering and output serialization.
   Keep imports free of simulation and filesystem side effects.
10. Preserve legacy characterization evidence before changing semantics.

## Legacy versus corrected behavior

Legacy n-sample results omit the final integrated state, label failure at the
step start, and pad failed traces. Legacy PID owns dt and advances its integral
on every call. Legacy success checks only final stored angle and failure status.
PR-001 deliberately tests these behaviors under `test_legacy_*` names.
They are not requirements for a corrected simulator. A later PR must retire or
isolate those tests, add corrected-contract tests, and explain changed fixtures
and metrics. Never silently replace a baseline to resolve failures.

## Consequences

Explicit contracts improve testability and prevent padded samples being treated
as observations. Correct timing can change success masks, metrics and figures;
these changes require review. Continuous-time eigenvalues remain separate from
sampled and nonlinear validation. This proposal does not claim the current
implementation already meets these contracts.

## Alternatives and deferrals

Retaining implicit tuples minimizes code change but leaves ambiguous semantics.
Continuous feedback at RK stages changes the control model and is incompatible
with naive stateful-controller calls. Adopt neither as an accidental refactor.
Defer swing-up, estimators, adaptive integration, precise event localization,
controller plugins and separate integration/control rates. Anti-windup and
explicit success metrics need separate designs and tests in later phases.
