import {it,expect} from 'vitest';
import {readFileSync} from 'node:fs';
import {RobotSimulation} from '../src/robot/simulation';
it('fluid boundary coupling transfers equal and opposite linear impulse',()=>{const s=new RobotSimulation(readFileSync('src/robot/q14.urdf','utf8'));s.reset('water');const w=s.water!,f=w.fluid;f.walls=false;f.gravity.fill(0);s.dynamics.kinematics();const b=s.dynamics.bodies[3];f.position.fill(100);f.position[0]=b.p[0]+b.R[2]*(-.22)+.025;f.position[1]=b.p[1]+b.R[5]*(-.22);f.position[2]=b.p[2]+b.R[8]*(-.22);w.step();let total=0;for(let i=0;i<f.count;i++)total+=f.mass*f.velocity[i*3];expect(Math.abs(total)).toBeGreaterThan(1e-6);expect(total+w.reaction[0]*.001).toBeCloseTo(0,10);});
