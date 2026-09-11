import {it,expect} from 'vitest';
import {readFileSync} from 'node:fs';
import {RobotSimulation} from '../src/robot/simulation';
it('sensor-driven PD supports the floating quadruped for 10 seconds',()=>{const s=new RobotSimulation(readFileSync('src/robot/q14.urdf','utf8'));let penetration=0;const start=performance.now();for(let i=0;i<10000;i++){s.step();penetration=Math.max(penetration,s.ground.maxPenetration);}console.log('PD standing',s.dynamics.position,'penetration',penetration,'ms/step',(performance.now()-start)/10000);expect(s.dynamics.position[2]).toBeGreaterThan(.25);expect(Math.hypot(s.dynamics.orientation[0],s.dynamics.orientation[1])).toBeLessThan(.1);expect(penetration).toBeLessThan(.005);},120000);
