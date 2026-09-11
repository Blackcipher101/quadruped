# Implemented physics and control

All physics uses SI units and Float64 scratch arrays. three.js is restricted to rendering. This document distinguishes implemented equations from requested future solvers.

## Spatial dynamics

A spatial motion vector is v = [ω; u], with angular velocity first. Its motion cross product is

```
v× = [ [ω]×   0   ]
     [ [u]× [ω]× ]
v×* = −(v×)ᵀ
```

For center of mass c and COM inertia Ic, the spatial inertia is

```
I = [ Ic + m[c]×[c]×ᵀ   m[c]× ]
    [       m[c]×ᵀ       m 1   ]
```

Transforms map parent motion into child coordinates; forces transform by the transpose. RNEA propagates vᵢ = Xᵢvₚ + Sᵢq̇ᵢ and aᵢ = Xᵢaₚ + Sᵢq̈ᵢ + vᵢ×Sᵢq̇ᵢ. The backward recursion sums fᵢ = Iᵢaᵢ + vᵢ×*Iᵢvᵢ − fext and τᵢ = Sᵢᵀfᵢ. Gravity enters as negative root acceleration.

CRBA accumulates composite inertia Iᶜₚ += XᵢᵀIᶜᵢXᵢ and projects along ancestor joint subspaces to construct M. ABA instead eliminates each joint using U = IᴬS, d = SᵀU + JrG², and Īᴬ = Iᴬ − UUᵀ/d, then solves accelerations in a forward pass. The tests independently compare ABA with M⁻¹(τ − b). Reflected rotor inertia JrG² is included in both algorithms.

The base has a body-coordinate spatial twist and a unit quaternion. Semi-implicit Euler kicks velocity, updates translation using world velocity, applies the quaternion exponential of angular velocity, normalizes, then transports linear velocity into the new body frame. Passive energy tests use 16 substeps per millisecond; browser contact uses two. RK4 is much more accurate for the smooth passive fixture but does not directly incorporate this velocity-impulse contact solve.

Robot masses and inertias come from box primitives in `q14.urdf`, not manufacturer measurements. Total mass is 13.9 kg. Each geared leg joint has Jr = 2e-5 kg·m² and ratio 9. Rock inertia integrates signed mesh tetrahedra, including COM translation.

## Coupled contact

After unconstrained velocity prediction, contact impulses satisfy

```
v⁺ = v⁻ + M⁻¹Jᵀλ
0 ≤ λn ⟂ (Jn v⁺ + bias) ≥ 0
‖λt‖₂ ≤ μ λn
```

Compliance adds diagonal regularization to the contact Delassus matrix W = JM⁻¹Jᵀ. Penetration bias supplies velocity-level stabilization. A fixed-order block projected Gauss–Seidel iteration uses the circular Coulomb cone, with warm-started multipliers. Rolling/spinning impulses are bounded using force times a friction length in metres. This is a time-discrete convex relaxation; general multi-impact Poisson restitution is not implemented.

Sweep-and-prune finds convex pairs. GJK/EPA supplies separation/penetration, with SAT for degenerate and persistent touching configurations. Robot-to-rock foot contacts use sphere/convex distance. Convex hull vertices supply terrain support. Contact fixtures do not establish complete collision coverage for every stair/shape geometry.

## Trained walking

The implemented policy is a CPG with stance scheduling, polynomial swing height, leg IK, proprioceptive velocity feedback and tilt correction. IK outputs joint targets only. A 1 kHz joint controller sends

```
τ = Kp(q* − qsensor) + Kd(q̇* − q̇sensor) + τff
```

through a geared motor torque-speed/current envelope. Observations are noisy gyro, accelerometer, quantized encoders and contact sensors. Gyro integration and stance odometry form a complementary observer, not an EKF. Cross-entropy optimization updates the mean and variance of bounded gait parameters from elite measured rollouts. Ground truth is used only for training rewards and validation, not runtime policy inputs.

The requested convex MPC would discretize the single-rigid-body equations

```
ṗ = v
m v̇ = Σ fi + m g
I ω̇ = Σ (ri − p) × fi − ω × Iω
```

and minimize a horizon cost Σ ‖xk − xref‖Q² + ‖fk‖R² subject to the linearized dynamics, scheduled zero swing forces, normal force bounds and friction constraints. **This MPC, the whole-body QP, hierarchical null-space tasks and an EKF are not implemented.** The CPG policy must not be described as these solvers.

## Experimental water

The fluid uses a weakly compressible SPH discretization of momentum balance, with ρ₀ = 1000 kg/m³, particle mass 0.064 kg, spacing 0.04 m, support radius h = 0.08 m and two 0.5 ms substeps:

```
ρi = Σj mj Wpoly6(rij, h)
pi = max(0, ρ₀ c²/7 [(ρi/ρ₀)⁷ − 1]), c = 12 m/s
Fi,p = −Σj mi mj (pi/ρi² + pj/ρj²) ∇Wspiky(rij)
Fi,μ = Σj mi mj μ/(ρiρj) (vj − vi) ∇²Wvisc(rij)
```

Physical dynamic viscosity is 0.001 Pa·s. Monaghan compression viscosity with α = 0.02 adds explicit numerical dissipation. Every pair force is accumulated equal and opposite. Semi-implicit integration applies gravity and penalty boundary forces without velocity clamping or positional projection.

Subgrid link spheres exchange penalty-contact forces with particles. The same force and moment, with opposite sign, enters the rigid body's external spatial wrench; two fluid substeps average their reaction over the robot step. This is explicit partitioned coupling. It does not impose a resolved no-slip fluid boundary or solve an incompressible pressure Poisson equation. Missing boundary support, coarse resolution and the atmospheric pressure floor limit hydrostatic and drag accuracy. The fluid-only domain walls are visible as a wire outline. Marching cubes reconstructs the water surface from worker particle positions; there are no animated shader waves.

## References

- [Featherstone, Spatial Vector and Rigid-Body Dynamics Software](https://royfeatherstone.org/spatial/v2/index.html).
- Anitescu and Potra, *Formulating Dynamic Multi-Rigid-Body Contact Problems with Friction as Solvable Linear Complementarity Problems*, Nonlinear Dynamics 14, 1997.
- [Todorov, Convex and analytically-invertible dynamics with contacts and constraints, 2014](https://homes.cs.washington.edu/~todorov/papers/TodorovICRA14.pdf).
- [Di Carlo et al., Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control](https://dspace.mit.edu/bitstream/handle/1721.1/138000/convex_mpc_2fix.pdf).
- [Müller, Charypar and Gross, Particle-Based Fluid Simulation for Interactive Applications, 2003](https://matthias-research.github.io/pages/publications/sca03.pdf).
- [Becker and Teschner, Weakly Compressible SPH for Free Surface Flows, 2007](https://cg.informatik.uni-freiburg.de/publications/2007_SCA_SPH.pdf).
