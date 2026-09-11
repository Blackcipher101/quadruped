import {it,expect} from 'vitest';
import {readFileSync} from 'node:fs';
import {parseURDF} from '../src/robot/urdf';
import {Actuator} from '../src/robot/actuator';
it('URDF yields 12 actuated DOFs plus a genuine 6D floating base',()=>{const m=parseURDF(readFileSync('src/robot/q14.urdf','utf8'));expect(m.nv).toBe(18);expect(m.nj).toBe(12);expect(m.links.reduce((s,l)=>s+l.spec.mass,0)).toBeCloseTo(13.9,10);expect(m.links[1].rotor).toBeCloseTo(.00162,12);});
it('PMSM saturates current and derates at back-EMF limit',()=>{const a=new Actuator();a.command(100,0);const zero=a.deratedTorque;expect(Math.abs(a.current)).toBeLessThanOrEqual(a.currentLimit);a.command(100,54);expect(a.deratedTorque).toBeLessThan(zero);a.command(100,60);expect(a.deratedTorque).toBe(0);});
