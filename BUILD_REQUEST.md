Build a real-time, physically-accurate quadruped robot simulator that runs in the browser.
three.js is ONLY the renderer. The physics is hand-written, first-principles rigid-body
dynamics. Accuracy is the product; do not trade it for visual shortcuts.

## Hard constraints
- TypeScript + Vite. three.js for rendering only.
- Do NOT use cannon-es, ammo.js, Rapier, or any black-box physics engine. Write the dynamics.
- No kinematic animation, no ground snapping, no "lift the body when a foot penetrates",
  no gravity scaling, no velocity clamping to hide instability. If it walks, it walks
  because contact forces and joint torques made it walk.
- Physics is fully decoupled from rendering: fixed timestep accumulator, render interpolates
  between physics states. Physics runs in a Web Worker over SharedArrayBuffer.
- Deterministic: fixed dt, fixed contact ordering, no Math.random in the solver path,
  zero heap allocation inside the step loop (preallocated typed arrays / scratch buffers).

## 1. Multibody dynamics
- Spatial (6D) vector algebra à la Featherstone. Implement:
  - Composite Rigid Body Algorithm (CRBA) for the joint-space mass matrix M(q)
  - Recursive Newton-Euler (RNEA) for bias forces C(q,qd) + gravity
  - Articulated Body Algorithm (ABA) for forward dynamics
  - Geometric Jacobians and their time derivatives per contact/end-effector
- Floating base: 6-DoF free joint, unit-quaternion orientation, integrate on SE(3) with
  proper quaternion exponential map and renormalization. No Euler angles anywhere.
- Joint limits as hard inequality constraints in the solver (soft stops are a fallback, not
  the default). Joint dry friction (Coulomb) + viscous damping. Reflected rotor inertia
  through the gear ratio added to the diagonal of M — do not skip this, it dominates
  small-timestep stability on geared legs.
- Integrator: semi-implicit / implicit-in-velocity Euler at dt = 1/1000 s with substepping.
  Compare against RK4 on a passive test and document the tradeoff.

## 2. Contact and collision
- Broadphase: sweep-and-prune or a BVH over AABBs, with temporal coherence.
- Narrowphase: GJK + EPA for convex-convex, analytic capsule/sphere-heightfield,
  persistent contact manifolds with warm-started multipliers across steps.
- Contact model: non-penetration + Coulomb friction as a nonlinear complementarity problem.
  Solve as a convex relaxation with projected Gauss-Seidel (or PGS + Jacobi hybrid),
  warm started. Friction cone, not a box approximation — do the second-order cone
  projection; keep the pyramid as a comparison mode behind a flag.
- Include rolling and spinning friction so rocks come to rest instead of jittering forever.
- Soft-constraint / compliance (CFM+ERP style, or MuJoCo's solref/solimp) with the stiffness
  and damping exposed as physical quantities, not magic numbers. Restitution via Poisson,
  and it must go to zero cleanly for resting contact.
- Per-material friction and restitution: rubber foot pads, rock, dirt, steel.

## 3. World
- Heightfield terrain with configurable roughness plus a set of loose rocks as full
  multibody rigid bodies: convex hulls with inertia tensors computed from the actual mesh
  (divergence-theorem integration), so they tumble, stack, and shift underfoot correctly.
- Scenes: flat ground, 15° and 25° slopes, stairs, a rock field, a single-plank balance beam,
  and a low-friction ice patch (mu = 0.15).

## 4. Robot model
- 12-DoF quadruped, Unitree Go2 / ANYmal class: 3 joints per leg (hip abduction/adduction,
  hip flexion, knee). Realistic link masses, COMs, and full 3x3 inertia tensors — state your
  source or derive from primitive decomposition and document the numbers.
- Parse a URDF subset so the model is data, not code.
- Actuators: geared PMSM model — torque-speed envelope, current limit, gear ratio,
  gearbox efficiency and its friction, optional backlash. Torque command saturates against
  the real envelope; expose the derated torque in the UI.
- Sensors at their own rates: IMU at 1 kHz with Gaussian noise + random-walk bias,
  joint encoders at 1 kHz with quantization, foot contact/force sensors. Controller must
  consume ONLY sensor output, never ground truth.

## 5. Control stack
- 1 kHz joint-level: tau = Kp(q* - q) + Kd(qd* - qd) + tau_ff.
- ~250-500 Hz whole-body layer. Implement both, switchable:
  (a) convex MPC over a single-rigid-body model with contact-schedule constraints
      (MIT Cheetah 3 formulation), solving for ground reaction forces over a ~0.5 s horizon;
  (b) task-space whole-body QP: prioritized tasks (COM, orientation, swing foot),
      contact-consistent null-space projection, friction-cone and torque-limit constraints.
  Include a small dense QP solver (active-set or ADMM/OSQP-style) — no external solver deps.
- Gait scheduler: stand, trot, pace, bound, with adjustable duty factor and period.
  Raibert heuristic footstep placement + capture-point correction. Bezier swing trajectories.
- State estimator: EKF fusing IMU with leg kinematics/contact for base pose and velocity.
- Push recovery: the robot must survive a scripted lateral impulse and recover.

## 6. Validation — this is a deliverable, not an afterthought
Write a test suite (vitest) and a VALIDATION.md with plots/numbers:
- Energy conservation: passive double/triple pendulum, drift < 0.1% over 60 s.
- Momentum conservation in free flight (zero gravity, zero contact).
- Analytic comparison: compound pendulum period, and free-fall + elastic bounce apex height.
- Pinned-base gravity compensation: commanded tau = RNEA gravity term holds the pose exactly.
- M(q) is symmetric positive-definite; check dM/dt - 2C is skew-symmetric.
- ABA forward dynamics and M⁻¹(tau - C) agree to 1e-10.
- Contact: max penetration depth statistics, no drift of a static box over 10^5 steps,
  a stack of 20 rocks stays stacked, a box on a 25° slope with mu=0.3 slides and with
  mu=0.8 sticks (compare against the analytic tan(theta) threshold).
- Determinism: identical seed -> bit-identical trajectory hash across runs.

## 7. Viewer / UI
- three.js scene: PBR materials, cascaded shadow maps, instanced rock meshes, orbit + follow cams.
- Debug overlays, individually toggleable: contact points, GRF vectors scaled to Newtons,
  friction cones, COM and its ground projection, support polygon, planned footholds,
  MPC predicted trajectory, joint torque bars, velocity vectors.
- Live plots: joint torques, GRF magnitude per foot, base height/pitch/roll, solver iterations,
  constraint residual, step time (physics ms vs render ms).
- Controls: WASD velocity command, gait selector, sliders for Kp/Kd/mu/mass/gait period,
  pause, single-step, slow motion 0.05x-1x, reset, and mouse-drag to apply an impulse
  to any link so I can shove it.

## 8. Deliverables
- Clean repo: src/physics (engine-agnostic, no three.js import anywhere in it),
  src/robot, src/control, src/render, src/ui, tests/.
- README with build/run, and PHYSICS.md deriving the spatial algebra, the contact NCP,
  and the MPC formulation with the actual equations, plus references
  (Featherstone, Anitescu-Potra, Todorov, Di Carlo et al.).
- Must hold 1 kHz physics with the robot + 40 rocks on a modern laptop; report the numbers.

Work incrementally and keep it runnable: dynamics core + tests first, then contacts + tests,
then the robot standing under PD, then the whole-body controller, then gaits, then UI polish.
Run the validation suite at each stage and do not proceed past a failing check.
