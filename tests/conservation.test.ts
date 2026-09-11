import { it,expect } from 'vitest';
import { Dynamics } from '../src/physics/dynamics';
import { pendulum,Model } from '../src/physics/model';
import { RK4 } from '../src/physics/integrators';
import { vec } from '../src/physics/spatial';
import { writeFileSync,mkdirSync } from 'node:fs';
it('60-second passive pendulum energy gate and RK4 comparison',()=>{
 const rows:string[]=['links,integrator,substeps,max_relative_drift,final_relative_drift,ms_per_outer_step'];const traces:string[]=['time,semi_implicit_relative_drift,rk4_relative_drift'];let worst=0;
 for(const links of [2,3])for(const mode of ['semi','rk4']){
  const d=new Dynamics(pendulum(links));for(let i=0;i<links;i++)d.q[i]=.3/(i+1);const e0=d.energy();d.q.fill(0);const floor=d.energy();for(let i=0;i<links;i++)d.q[i]=.3/(i+1);const scale=e0-floor;const rk=new RK4(d);let max=0;const start=performance.now();
  for(let i=0;i<60000;i++){if(mode==='semi')d.step(.001,16);else rk.step(.001);const drift=Math.abs((d.energy()-e0)/scale);max=Math.max(max,drift);}
  rows.push(`${links},${mode},${mode==='semi'?16:1},${max},${(d.energy()-e0)/scale},${(performance.now()-start)/60000}`);if(mode==='semi')worst=Math.max(worst,max);
 }
 mkdirSync('public/validation',{recursive:true});writeFileSync('public/validation/energy.csv',rows.join('\n')+'\n');console.log(rows.join('\n'));expect(worst).toBeLessThan(.001);
},120000);
it('free rigid body momentum conservation over 10 seconds',()=>{const d=new Dynamics(new Model([{name:'body',parent:-1,mass:3,com:[0,0,0],inertia:[.2,0,0,0,.3,0,0,0,.4],axis:[0,0,1],offset:[0,0,0]}],true));d.gravity.fill(0);d.v.set([.3,.2,.1,.4,-.2,.3]);const p0=vec(6),p=vec(6);d.momentum(p0);for(let i=0;i<10000;i++)d.step(.001,16);d.momentum(p);console.log('10s momentum max error',Math.max(...p.map((x,i)=>Math.abs(x-p0[i]))));expect(Math.max(...p.map((x,i)=>Math.abs(x-p0[i])))).toBeLessThan(.0001);});
