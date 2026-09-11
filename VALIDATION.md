# Validation

Core measurements below were recorded 2026-09-09; course additions were checked 2026-09-11. The complete updated suite passes **35 tests across 14 files**. TypeScript checking and production build pass. These results apply only to the named fixtures.

| Check | Measurement / bound | Scope |
|---|---|---|
| Double pendulum, 60 s | maximum relative energy drift 0.0191165% | Semi-implicit Euler, 1 ms outer step, 16 substeps; initial q = [0.3,0.15] rad |
| Triple pendulum, 60 s | maximum relative energy drift 0.0229617% | Same, initial q = [0.3,0.15,0.1] rad |
| RK4, 60 s | relative drift 9.58e-13 double, 2.59e-12 triple | Smooth pinned systems, 1 ms step |
| Free-flight momentum, 10 s | max component error 3.19e-7 | Single asymmetric rigid body only; articulated test outstanding |
| Static box, 100,000 steps | horizontal drift approximately 4.1e-13 m; peak penetration 4.94e-13 m | Four symmetric ground sites, rigid flat terrain |
| Elastic sphere bounce | release/apex discrepancy <1 cm | Isolated impact, restitution 1; not multi-contact Poisson validation |
| Sensor-driven standing, 10 s | final base height 0.348761 m; peak penetration 9.51e-9 m | Flat ground, 2 substeps; finite lateral drift exists |
| Standing step runtime | 0.269 ms mean outer step | Node fixture, robot only, concurrent tests; not a real-time deadline guarantee |
| ABA vs independent dense solve | absolute agreement <1e-10 | Pinned and floating fixtures |
| Mass matrix, bias identities | tests pass | SPD, symmetry, Christoffel skew-symmetry, gravity compensation |
| Jacobian and Jdot | central finite-difference checks pass | Three-link pinned fixture |
| Determinism | bit-identical states over 10,000 steps | Pinned fixture; complete-world seeded trajectory hash outstanding |
| Mesh inertia / GJK / EPA | analytic cube tests pass | Component tests, not rock-stack validation |
| 25° friction threshold | single impulse fixture slides at μ=0.3, sticks at μ=0.8 | Full slope-box trajectory still outstanding |

Energy normalization uses energy above the hanging equilibrium, not a freely shiftable gravitational potential baseline. Raw energy results are regenerated at `public/validation/energy.csv`.

The energy bound requires substepping in these tests. RK4 uses fewer dynamics evaluations for the smooth passive fixtures and has substantially lower energy drift. Semi-implicit Euler is used for the browser's velocity-impulse contact coupling; the browser's two-substep setting is not claimed to satisfy the 60-second passive energy bound.

EKF/MPC/WBC, sustained 1 kHz coupled-world throughput, and scripted push recovery remain outstanding. Ground contact uses point/sphere samples; collision against vertical stair risers is not complete. A tiny static-fixture penetration is not a universal penetration guarantee.

## Walking and loose gravel — 2026-09-11

- Flat training: deterministic cross-entropy parameter search, population 10 × 8 generations, 5 s evaluations after a 0.7 s stand. This trains 11 gait parameters, not a neural network or whole-body MPC.
- Three held-out sensor-noise seeds traveled 0.99568, 0.99483 and 0.99436 m in 5 s, all upright. The separate 6 s walking test traveled 0.901 m at a 0.15 m/s command, with minimum base height 0.3458 m.
- Gravel adaptation tested three swing clearances. All remained upright for 10 s on the same 40-rock layout. Selected clearance 0.075 m traveled 1.132 m; lateral displacement −0.163 m, minimum height 0.3372 m, maximum penetration 1.33 mm. Aggregate horizontal rock displacement was 0.927 m; this is the sum over 40 rocks, not distance of a single rock. No held-out terrain generalization is claimed.
- A 20-body stack remains stacked for the tested 3 s; top center 1.3703 m, maximum penetration 7.69e-7 m. This fixture uses convex blocks, not a random stack of irregular stones.
- Runtime: selected gravel rollout averaged 9.89 ms per outer step. Later candidate wall-clock timings include host interruption and are unsuitable benchmarks. The 1 ms throughput target is **not met**.

Raw training results: `public/policies/walk.json`, `gravel.json`; scripts and logs in `scripts/` and `artifacts/training/`.

## Experimental fluid

Three SPH tests verify pair-force momentum conservation, bit-identical replay, and finite pool settling for 1 s. The 1,120-particle pool reached maximum density 1,104.45 kg/m³ (10.4% above reference), with vertical range 0.0108–0.1225 m. This density error is material: the current model is visibly compressible. A fourth test verifies equal-and-opposite fluid/robot linear impulse. These checks do not validate drag, hydrostatics, buoyancy, turbulence or no-slip boundaries.

Fluid particles are 40 mm apart with 80 mm support. Foot pads are therefore under-resolved. Pressure has an atmospheric zero floor. Computational box walls act on fluid only and are outlined in the viewer. Robot collision uses subgrid spheres, not a boundary-resolved fluid mesh. Water walking evaluation, when generated, is recorded separately in `public/policies/water-evaluation.json`; it is not water-specific training.

Water transfer evaluation: the flat-trained policy remained upright for 10 s, traveled 1.9087 m, with lateral displacement −0.0932 m and minimum height 0.3453 m. Maximum density reached 1,197.12 kg/m³ (19.7% above reference), further limiting incompressible-water fidelity. Mean step time was 13.14 ms under concurrent development workloads; this is not a deadline guarantee. No policy optimization on water or hydrostatic benchmark has been completed.
