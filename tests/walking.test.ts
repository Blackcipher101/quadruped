import {it,expect} from 'vitest';
import {readFileSync} from 'node:fs';
import {RobotSimulation} from '../src/robot/simulation';
it('sensor-driven gait produces forward motion through actuators and contact',()=>{const s=new RobotSimulation(readFileSync('src/robot/q14.urdf','utf8'));for(let i=0;i<1000;i++)s.step();s.locomotion.gait='trot';s.locomotion.command[0]=.15;let minHeight=1;for(let i=0;i<6000;i++){s.step();minHeight=Math.min(minHeight,s.dynamics.position[2]);}console.log('walk',Array.from(s.dynamics.position),'orientation',Array.from(s.dynamics.orientation),'minHeight',minHeight);expect(s.dynamics.position[0]).toBeGreaterThan(.35);expect(minHeight).toBeGreaterThan(.22);},120000);
