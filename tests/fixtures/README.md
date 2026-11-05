# Legacy numerical fixtures

`legacy-v1.json` is an immutable characterization baseline, generated explicitly
from the unmodified audited source. It records source commit and SHA-256 hashes,
Python/NumPy/SciPy/Matplotlib/Pillow versions, physical and controller parameters, initial states,
simulation settings, checkpoint indices, masks, tolerances and provenance.
No randomness or plots are involved. Checkpoints include disturbance boundaries
and the angle-PD rail-failure transition. Every independent trial uses a fresh
controller. All 30 basin outcomes per controller are stored, not just counts.

Run from the repository root:

```sh
python3 -B -m unittest discover -s tests -v
```

The suite never generates or modifies fixtures. To propose a replacement:

```sh
python3 -B tests/generate_fixtures.py --output /tmp/cartpole-baseline-proposal.json
```

Use a new, nonexistent path. The generator refuses overwrite. Investigate test
failures before generating anything; review source/version provenance and every
numeric difference before accepting a new baseline. The initial generation used
`--output tests/fixtures/legacy-v1.json`; that path must not be reused. Generator
execution is not independent proof of physical correctness: the suite separately
checks equilibrium, signs, translation invariance, Jacobians and CARE residuals.
Before accepting PR-001's baseline, a second run to a separate temporary proposal
path reproduced all numeric fields and source hashes exactly. Its complete
version and generator provenance was adopted after that comparison; the recorded
command therefore names that temporary generation path. No test failure prompted
a numeric baseline change.

## Tolerances

Trajectory states, forces and gains use rtol=1e-8, atol=1e-10 (in each component's
SI units). This permits small floating-point/CARE differences while remaining
well below the 0.02 rad success threshold. These are initial engineering bounds,
not measured cross-platform guarantees. Masks and failure-grid outcomes are
exact classifications. Times use absolute tolerance 1e-12 s. Central Jacobians
use h=1e-6 and absolute tolerance 1e-8 to accommodate roundoff and O(h²) error.
CARE residual Frobenius norms are normalized by the largest of 1, ||Q||,
||A.T P|| and ||P B R^-1 B.T P||, and must be below 1e-9.

Legacy timing/padding tests explicitly document defects and must be migrated
when the corrected simulator is implemented, not imposed on its new contract.
Fixtures sample trajectories rather than store every step; metrics and complete
basin masks supplement those samples. Renderer output, integration convergence,
global stability and cross-platform reproducibility are not validated here.

## What the checks establish

Finite-difference Jacobians and CARE residuals are mathematical consistency
checks; CARE uses the same SciPy solver as the controller. Timing and held-input
tests reuse production RK4 to check orchestration, not independent numerical
accuracy or convergence. The initial-failure spy executes real RK4 and verifies
one advancement and the resulting padded endpoint. The lifecycle test freezes
plant states to isolate four PID calls at theta=0.1 rad with PID dt=0.02 s:
integrals are 0.002, 0.004, 0.006 and 0.008 rad*s, even when simulation dt=0.01 s.
Its absolute tolerance of 1e-15 rad*s allows floating-point summation roundoff;
the PID dt itself must remain unchanged.

Tests invoke the generator only to exercise overwrite rejection; its build
function is not executed. Baselines remain untouched. The recorded generator
hash identifies the historical generation version; a later docstring-only edit
does not justify regenerating fixtures or changing their provenance.
