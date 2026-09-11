import { inertia, vec, type V } from './spatial';
export interface LinkSpec { name:string; parent:number; mass:number; com:number[]; inertia:number[]; offset:number[]; axis:number[]; rotor?:number; gear?:number; lower?:number; upper?:number; damping?:number; friction?:number; }
export class Link {
  readonly I=vec(36); readonly axis:V; readonly offset:V; readonly com:V; readonly S=vec(6);
  readonly rotor:number; readonly lower:number; readonly upper:number;
  constructor(readonly spec:LinkSpec,readonly index:number,readonly dof:number){
    this.axis=Float64Array.from(spec.axis);this.offset=Float64Array.from(spec.offset);this.com=Float64Array.from(spec.com);
    if(!(spec.mass>0)||spec.inertia.length!==9)throw Error('Invalid link inertia');
    if(dof>=0&&Math.abs(Math.hypot(...spec.axis)-1)>1e-12)throw Error('Joint axis must be unit');
    for(let i=0;i<3;i++)this.S[i]=this.axis[i];
    inertia(spec.mass,this.com,Float64Array.from(spec.inertia),this.I);
    this.rotor=(spec.rotor??0)*(spec.gear??1)**2;this.lower=spec.lower??-Infinity;this.upper=spec.upper??Infinity;
  }
}
export class Model {
  readonly links:Link[];readonly nv:number;readonly nj:number;
  constructor(specs:LinkSpec[],readonly floating:boolean){
    if(specs.length===0)throw Error('Empty model');
    this.links=specs.map((s,i)=>{if(s.parent>=i||s.parent< -1||(i>0&&floating&&s.parent<0))throw Error('Links must be topologically ordered');return new Link(s,i,floating?(i===0?-1:i+5):i);});
    this.nv=specs.length+(floating?5:0);this.nj=specs.length-(floating?1:0);
  }
}
export function pendulum(n=2){const specs:LinkSpec[]=[];for(let i=0;i<n;i++)specs.push({name:`rod${i}`,parent:i-1,mass:1,com:[0,0,-0.5],inertia:[1/12,0,0,0,1/12,0,0,0,0.002],offset:i===0?[0,0,0]:[0,0,-1],axis:[0,1,0]});return new Model(specs,false);}
