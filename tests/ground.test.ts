import {it,expect} from 'vitest';
import {Dynamics} from '../src/physics/dynamics';
import {Model} from '../src/physics/model';
import {GroundSystem,type ContactSite} from '../src/physics/ground';
function box(){return new Dynamics(new Model([{name:'box',parent:-1,mass:1,com:[0,0,0],inertia:[1/6,0,0,0,1/6,0,0,0,1/6],offset:[0,0,0],axis:[0,0,1]}],true));}
it('static box has no measurable drift over 100000 steps',()=>{const d=box(),sites:ContactSite[]=[];d.position[2]=.5;for(let x=-1;x<=1;x+=2)for(let y=-1;y<=1;y+=2)sites.push({body:0,point:Float64Array.of(x*.5,y*.5,-.5),radius:0,mu:.8,restitution:0});const w=new GroundSystem(d,sites);let penetration=0;for(let i=0;i<100000;i++){w.step();penetration=Math.max(penetration,w.maxPenetration);}console.log('static box',d.position,'penetration',penetration);expect(Math.hypot(d.position[0],d.position[1],d.position[2]-.5)).toBeLessThan(1e-7);expect(penetration).toBeLessThan(1e-7);},120000);
it('elastic sphere bounce returns to its release apex within time-step error',()=>{const d=box();d.position[2]=1;const w=new GroundSystem(d,[{body:0,point:Float64Array.of(0,0,0),radius:.1,mu:0,restitution:1}]);let rebounded=false,apex=0;for(let i=0;i<1800;i++){w.step();if(d.v[5]>1)rebounded=true;if(rebounded)apex=Math.max(apex,d.position[2]);}expect(rebounded).toBe(true);expect(Math.abs(apex-1)).toBeLessThan(.01);});
