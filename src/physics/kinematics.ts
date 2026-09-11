import { Dynamics } from './dynamics';
import { vec,mv,type V } from './spatial';
/** World-frame geometric point Jacobian and exact directional time derivative. */
export class PointKinematics {
 readonly J:V;readonly Jdot:V;readonly position=vec(3);readonly velocity=vec(3);
 private r=vec(3);private axis=vec(3);private axisDot=vec(3);private w=vec(3);private vo=vec(3);private t=vec(3);
 constructor(readonly dynamics:Dynamics){this.J=vec(3*dynamics.model.nv);this.Jdot=vec(3*dynamics.model.nv);}
 compute(body:number,local:V){
  const d=this.dynamics,n=d.model.nv,b=d.bodies[body];this.J.fill(0);this.Jdot.fill(0);
  mv(b.R,local,this.r,3);for(let k=0;k<3;k++)this.position[k]=b.p[k]+this.r[k];
  for(let k=0;k<3;k++){const j=(k+1)%3,l=(k+2)%3;this.t[k]=b.v[k+3]+b.v[j]*local[l]-b.v[l]*local[j];}mv(b.R,this.t,this.velocity,3);
  for(let i=body;i>=0;i=d.model.links[i].spec.parent){const link=d.model.links[i],work=d.bodies[i];
   for(let k=0;k<3;k++){this.t[k]=work.v[k];}mv(work.R,this.t,this.w,3);for(let k=0;k<3;k++)this.t[k]=work.v[k+3];mv(work.R,this.t,this.vo,3);
   const free=d.model.floating&&i===0;
   for(let column=0;column<(free?6:1);column++){
    const c=free?column:link.dof;
    for(let k=0;k<3;k++){this.axis[k]=free?work.R[k*3+column%3]:work.R[k*3]*link.axis[0]+work.R[k*3+1]*link.axis[1]+work.R[k*3+2]*link.axis[2];this.r[k]=this.position[k]-work.p[k];}
    for(let k=0;k<3;k++){const j=(k+1)%3,l=(k+2)%3;this.axisDot[k]=this.w[j]*this.axis[l]-this.w[l]*this.axis[j];}
    for(let k=0;k<3;k++){const j=(k+1)%3,l=(k+2)%3;if(free&&column>=3){this.J[k*n+c]=this.axis[k];this.Jdot[k*n+c]=this.axisDot[k];}else{this.J[k*n+c]=this.axis[j]*this.r[l]-this.axis[l]*this.r[j];this.Jdot[k*n+c]=this.axisDot[j]*this.r[l]-this.axisDot[l]*this.r[j]+this.axis[j]*(this.velocity[l]-this.vo[l])-this.axis[l]*(this.velocity[j]-this.vo[j]);}}
   }
  }
 }
}
