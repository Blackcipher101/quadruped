import { describe,it,expect } from 'vitest';
import { Dynamics } from '../src/physics/dynamics';
import { Model, pendulum, type LinkSpec } from '../src/physics/model';
import { vec, factor, expQuaternion } from '../src/physics/spatial';
function fixture(floating:boolean){const specs:LinkSpec[]=[];for(let i=0;i<7;i++)specs.push({name:`b${i}`,parent:i===0?-1:Math.floor((i-1)/2),mass:1+i/3,com:[.03,-.02,-.15],inertia:[.04,.002,.001,.002,.05,-.003,.001,-.003,.02],offset:[.2,.1,-.25],axis:i%2?[0,1,0]:[1,0,0],rotor:.00003,gear:9});return new Dynamics(new Model(specs,floating));}
function state(d:Dynamics){for(let i=0;i<d.model.nv;i++){d.q[i]=.3*Math.sin(i+1);d.v[i]=.7*Math.cos(i+1);d.tau[i]=Math.sin(i*3);}expQuaternion(d.orientation,.3,-.6,.2,1);d.position.set([.2,.3,1]);}
describe('independent dynamics identities',()=>{
 for(const floating of [false,true]){
  it(`ABA matches CRBA + Cholesky to 1e-10 (${floating?'floating':'pinned'})`,()=>{const d=fixture(floating),out=vec(d.model.nv);for(let s=0;s<30;s++){state(d);for(let i=0;i<d.q.length;i++)d.q[i]+=.013*s;d.aba();const actual=d.qdd.slice();d.denseForward(out);expect(Math.max(...actual.map((v,i)=>Math.abs(v-out[i])))).toBeLessThan(1e-10);}});
  it(`M is symmetric positive definite (${floating})`,()=>{const d=fixture(floating);state(d);d.crba();const n=d.model.nv;factor(d.M,vec(n*n),n);for(let i=0;i<n;i++)for(let j=0;j<n;j++)expect(Math.abs(d.M[i*n+j]-d.M[j*n+i])).toBeLessThan(1e-13);});
 }
 it('pinned gravity compensation holds pose for 10 seconds',()=>{const d=fixture(false);state(d);d.v.fill(0);d.rnea(vec(d.model.nv),d.tau);const q=d.q.slice();for(let i=0;i<10000;i++)d.step();expect(Math.max(...d.q.map((x,i)=>Math.abs(x-q[i])))).toBeLessThan(1e-10);});
 it('dM/dt - 2C is skew symmetric (Christoffel C)',()=>{const d=fixture(false);state(d);const n=d.model.nv,h=1e-5,dm=vec(n*n*n),C=vec(n*n),mdot=vec(n*n);for(let k=0;k<n;k++){d.q[k]+=h;const plus=d.crba().slice();d.q[k]-=2*h;const minus=d.crba().slice();d.q[k]+=h;for(let i=0;i<n*n;i++)dm[k*n*n+i]=(plus[i]-minus[i])/(2*h);}
 for(let i=0;i<n;i++)for(let j=0;j<n;j++)for(let k=0;k<n;k++){C[i*n+j]+=.5*(dm[k*n*n+i*n+j]+dm[j*n*n+i*n+k]-dm[i*n*n+j*n+k])*d.v[k];mdot[i*n+j]+=dm[k*n*n+i*n+j]*d.v[k];}
 d.gravity.fill(0);d.rnea(vec(n),d.bias);for(let i=0;i<n;i++){let cv=0;for(let j=0;j<n;j++){cv+=C[i*n+j]*d.v[j];expect(Math.abs(mdot[i*n+j]+mdot[j*n+i]-2*C[i*n+j]-2*C[j*n+i])).toBeLessThan(1e-9);}expect(Math.abs(cv-d.bias[i])).toBeLessThan(1e-9);}});
 it('compound pendulum acceleration matches closed form',()=>{const d=new Dynamics(pendulum(1));d.q[0]=.7;d.aba();expect(d.qdd[0]).toBeCloseTo(-9.81*.5*Math.sin(.7)/(1/3),12);});
 it('free fall follows analytic position to integration truncation tolerance',()=>{const d=fixture(true);d.v.fill(0);d.position[2]=10;for(let i=0;i<1000;i++)d.step();expect(d.position[2]).toBeCloseTo(10-.5*9.81*1.001,10);for(let i=6;i<d.q.length;i++)expect(Math.abs(d.q[i])).toBeLessThan(1e-10);});
 it('quaternion exponential maintains unit norm over 100000 updates',()=>{const q=Float64Array.of(0,0,0,1);for(let i=0;i<100000;i++)expQuaternion(q,.3,-.7,1.1,.001);expect(Math.abs(Math.hypot(...q)-1)).toBeLessThan(3e-16);});
 it('same initial state gives bit-identical 10000-step trajectory',()=>{const a=new Dynamics(pendulum()),b=new Dynamics(pendulum());a.q.set([.3,-.1]);b.q.set(a.q);for(let i=0;i<10000;i++){a.step();b.step();expect(new Uint8Array(a.q.buffer)).toEqual(new Uint8Array(b.q.buffer));}});
});
