# Current coverage (details)

[Back to README](../README.md)

## Current coverage

Implemented: CRBA, RNEA, ABA, floating base with quaternion exponential integration, geared rotor inertia, hard joint-limit impulses, ground-contact cone solver, analytic point Jacobians/Jdot, strict aligned-frame revolute URDF subset, representative PMSM envelope, noisy IMU and quantized encoders, sensor-only PD, fixed 1 ms Worker timestep, shared triple buffer, PBR rendering with cascaded shadows, diagnostics and input controls.

The browser gravel scene uses the coupled multibody world: convex mesh inertias, sweep-and-prune, GJK/EPA with SAT fallback, persistent impulse warm starts, circular friction cones, and rolling/spinning friction. Forty individually simulated rocks can move underfoot.

Walking uses a sensor-only CPG/IK parameter policy optimized by cross-entropy search: 80 flat-ground rollouts, then three gravel clearance trials. IK generates actuator targets; torque and contact produce all motion. Reproduce with `node_modules/.bin/vite-node scripts/train-walk.ts` and `scripts/train-gravel.ts`. Saved results are in `public/policies/`.

Water uses a coarse weakly compressible SPH discretization with pressure, viscosity, gravity and subgrid sphere coupling. It is **not a validated incompressible CFD model**; thin feet, turbulence, wetting, detailed buoyancy and no-slip boundaries are unresolved. No water-specific policy has been trained. See [PHYSICS.md](../PHYSICS.md).

Outstanding: full capsule collision and manifolds, multi-contact Poisson restitution, physically compliant actuator/friction integration, EKF, MPC, whole-body QP, robust terrain-general locomotion and push-recovery validation, full debug overlays, and the robot-plus-40-rock 1 kHz throughput target. Measured gravel steps are around 10 ms in one training run; the real-time target is not met. The worker retains backlog instead of changing physical time or gravity. Optional backlash is absent. Do not infer completion from the number of passing component tests.

See [VALIDATION.md](../VALIDATION.md) for measured results and limitations. The URDF inertias are primitive-derived, not measured Unitree parameters.
