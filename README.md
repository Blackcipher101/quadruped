# Q14 Dynamics Lab

A runnable three.js quadruped playground backed by hand-written spatial rigid-body dynamics, with a trained
sensor-driven trot, 40 movable gravel rocks and experimental 3D particle water.

| Trained trot on loose gravel (40 dynamic rocks) | Experimental SPH water |
|---|---|
| [![Quadruped trotting over loose gravel](docs/media/gravel-walk-5x.gif)](docs/media/gravel-walk.mp4) | [![Quadruped beside the SPH water volume](docs/media/water-5x.gif)](docs/media/water.mp4) |
| [mp4](docs/media/gravel-walk.mp4) · physics ran at 0.17x real time; GIF sped up 5x | [mp4](docs/media/water.mp4) · 0.16x real time; GIF sped up 5x |

| Flat-ground trot with the footprint overlay, real time (1.01x) |
|---|
| [![Trained trot on flat ground](docs/media/flat-trot.gif)](docs/media/flat-trot.mp4) |
| [mp4](docs/media/flat-trot.mp4) · every link pose comes from the physics Worker; the renderer adds no animation |

## Results

Walking (sensor-only CPG/IK gait parameters from cross-entropy search; [VALIDATION.md](VALIDATION.md)):

| scene | duration | travelled | notes |
|---|---|---|---|
| flat, 3 held-out sensor-noise seeds | 5 s | 0.99568 / 0.99483 / 0.99436 m | all upright |
| gravel, 40 rocks, clearance 0.075 m | 10 s | 1.132 m | upright; min height 0.3372 m, max penetration 1.33 mm |
| water (flat-trained policy, no water training) | 10 s | 1.9087 m | upright; max density 19.7% above reference |

Physics checks (named fixtures only):

| check | measurement |
|---|---|
| double pendulum, 60 s, semi-implicit Euler | max relative energy drift 0.0191165% |
| ABA vs independent dense solve | absolute agreement <1e-10 |
| determinism, pinned fixture | bit-identical states over 10,000 steps |
| gravel step runtime | 9.89 ms per outer step: the 1 ms real-time target is **not met** |

## Quick start

```sh
npm install
npm run dev     # open http://localhost:5173 (Vite supplies the COOP/COEP headers; index.html directly will not work)
npm test
npm run build
```

## More

- [docs/USAGE.md](docs/USAGE.md): demo capture details, controls (gravel, water, impulses, gains, overlays) and checks.
- [docs/COVERAGE.md](docs/COVERAGE.md): what is implemented (dynamics, contact, gravel world, walking policy, water)
  and what is outstanding.
- [PHYSICS.md](PHYSICS.md): implemented equations; [VALIDATION.md](VALIDATION.md): all measured results and
  limitations; [BUILD_REQUEST.md](BUILD_REQUEST.md): the full simulator specification.

Known limitations: not yet the complete simulator in `BUILD_REQUEST.md`; the SPH water is visibly compressible and not
validated CFD; only straight trot on flat ground and gravel has walking validation.
