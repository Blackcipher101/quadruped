import {Dynamics} from './dynamics';
import {PointKinematics} from './kinematics';
import {vec,factor,solve,type V} from './spatial';
export class Impulse {
 private k:PointKinematics;private L:V;private rhs:V;private dv:V;
 constructor(readonly d:Dynamics){const n=d.model.nv;this.k=new PointKinematics(d);this.L=vec(n*n);this.rhs=vec(n);this.dv=vec(n);}
 apply(body:number,localPoint:V,worldImpulse:V){const d=this.d,n=d.model.nv;d.crba();this.k.compute(body,localPoint);this.rhs.fill(0);for(let i=0;i<n;i++)for(let k=0;k<3;k++)this.rhs[i]+=this.k.J[k*n+i]*worldImpulse[k];factor(d.M,this.L,n);solve(this.L,this.rhs,this.dv,n);for(let i=0;i<n;i++)d.v[i]+=this.dv[i];}
}
