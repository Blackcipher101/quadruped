import { Model } from './model';
import { vec, mv, mm, dot, crm, crf, transform, rotation, axisRotation, factor, solve, expQuaternion, identity, type V } from './spatial';
class BodyWork { X=vec(36);R=vec(9);p=vec(3);v=vec(6);c=vec(6);a=vec(6);f=vec(6);IA=vec(36);pA=vec(6);U=vec(6);d=0;u=0; }
/** Allocation-free recursions; one workspace per simulation. Base twist is body-local. */
export class Dynamics {
  readonly q:V;readonly v:V;readonly tau:V;readonly qdd:V;readonly M:V;readonly bias:V;
  readonly position=vec(3);readonly orientation=Float64Array.of(0,0,0,1);readonly gravity=Float64Array.of(0,0,-9.81);
  readonly bodies:BodyWork[];readonly external:V[];
  private E=vec(9);private R=vec(9);private r=vec(3);private t=vec(6);private s=vec(6);private jv=vec(6);private a0=vec(6);private mat=vec(36);private mat2=vec(36);private L=vec(36);private baseRhs=vec(6);
  private massL:V;private rhs:V;private zeros:V;
  constructor(readonly model:Model){const n=model.nv;this.q=vec(n);this.v=vec(n);this.tau=vec(n);this.qdd=vec(n);this.M=vec(n*n);this.bias=vec(n);this.massL=vec(n*n);this.rhs=vec(n);this.zeros=vec(n);this.bodies=model.links.map(()=>new BodyWork());this.external=model.links.map(()=>vec(6));}
  kinematics(){
    const {model}=this;
    for(let i=0;i<model.links.length;i++){
      const l=model.links[i],b=this.bodies[i],p=l.spec.parent;
      if(model.floating&&i===0){
        rotation(this.orientation,b.R);b.p.set(this.position);
        for(let r=0;r<3;r++)for(let c=0;c<3;c++)this.E[r*3+c]=b.R[c*3+r];transform(this.E,this.position,b.X);for(let k=0;k<6;k++)b.v[k]=this.v[k];b.c.fill(0);
      }else{
        axisRotation(l.axis,this.q[l.dof],this.E);transform(this.E,l.offset,b.X);
        if(p>=0){const pb=this.bodies[p];mm(pb.R,this.E,b.R,3,false,true);mv(pb.R,l.offset,this.r,3);for(let k=0;k<3;k++)b.p[k]=pb.p[k]+this.r[k];mv(b.X,pb.v,b.v);}
        else{for(let r=0;r<3;r++)for(let c=0;c<3;c++)b.R[r*3+c]=this.E[c*3+r];b.p.set(l.offset);b.v.fill(0);}
        for(let k=0;k<6;k++){this.jv[k]=l.S[k]*this.v[l.dof];b.v[k]+=this.jv[k];}crm(b.v,this.jv,b.c);
      }
    }
  }
  rnea(acc:V,out:V){
    this.kinematics();this.a0.fill(0);for(let k=0;k<3;k++)this.a0[k+3]=-this.gravity[k];
    for(let i=0;i<this.bodies.length;i++){
      const l=this.model.links[i],b=this.bodies[i];mv(b.X,l.spec.parent<0?this.a0:this.bodies[l.spec.parent].a,b.a);
      for(let k=0;k<6;k++)b.a[k]+=b.c[k]+(this.model.floating&&i===0?acc[k]:l.S[k]*acc[l.dof]);
      mv(l.I,b.a,b.f);mv(l.I,b.v,this.t);crf(b.v,this.t,this.s);for(let k=0;k<6;k++)b.f[k]+=this.s[k]-this.external[i][k];
    }
    for(let i=this.bodies.length-1;i>=0;i--){const l=this.model.links[i],b=this.bodies[i];if(this.model.floating&&i===0){for(let k=0;k<6;k++)out[k]=b.f[k];}else out[l.dof]=dot(l.S,b.f)+l.rotor*acc[l.dof];if(l.spec.parent>=0){mv(b.X,b.f,this.t,6,true);const pf=this.bodies[l.spec.parent].f;for(let k=0;k<6;k++)pf[k]+=this.t[k];}}
  }
  crba(){
    this.kinematics();this.M.fill(0);const n=this.model.nv;
    for(let i=0;i<this.bodies.length;i++)this.bodies[i].IA.set(this.model.links[i].I);
    for(let i=this.bodies.length-1;i>=0;i--){
      const l=this.model.links[i],b=this.bodies[i];
      if(this.model.floating&&i===0){for(let r=0;r<6;r++)for(let c=0;c<6;c++)this.M[r*n+c]=b.IA[r*6+c];}
      else{
        mv(b.IA,l.S,this.t);this.M[l.dof*n+l.dof]=dot(l.S,this.t)+l.rotor;
        let j=i;while(this.model.links[j].spec.parent>=0){mv(this.bodies[j].X,this.t,this.s,6,true);this.t.set(this.s);j=this.model.links[j].spec.parent;
          if(this.model.floating&&j===0){for(let k=0;k<6;k++)this.M[l.dof*n+k]=this.M[k*n+l.dof]=this.t[k];}
          else{const d=this.model.links[j].dof;this.M[l.dof*n+d]=this.M[d*n+l.dof]=dot(this.model.links[j].S,this.t);}
        }
      }
      if(l.spec.parent>=0){mm(b.IA,b.X,this.mat);mm(b.X,this.mat,this.mat2,6,true);const parent=this.bodies[l.spec.parent].IA;for(let k=0;k<36;k++)parent[k]+=this.mat2[k];}
    }return this.M;
  }
  /** Featherstone articulated-body recursion with scalar geared-joint armature. */
  aba(){
    this.kinematics();
    for(let i=0;i<this.bodies.length;i++){const l=this.model.links[i],b=this.bodies[i];b.IA.set(l.I);mv(l.I,b.v,this.t);crf(b.v,this.t,b.pA);for(let k=0;k<6;k++)b.pA[k]-=this.external[i][k];}
    for(let i=this.bodies.length-1;i>=0;i--){const l=this.model.links[i],b=this.bodies[i];if(this.model.floating&&i===0)continue;
      mv(b.IA,l.S,b.U);b.d=dot(l.S,b.U)+l.rotor;if(!(b.d>0))throw Error('Invalid articulated inertia');b.u=this.tau[l.dof]-dot(l.S,b.pA);
      if(l.spec.parent>=0){
        for(let r=0;r<6;r++)for(let c=0;c<6;c++)this.mat[r*6+c]=b.IA[r*6+c]-b.U[r]*b.U[c]/b.d;
        mv(this.mat,b.c,this.t);for(let k=0;k<6;k++)this.t[k]+=b.pA[k]+b.U[k]*b.u/b.d;
        mv(b.X,this.t,this.s,6,true);const parent=this.bodies[l.spec.parent];for(let k=0;k<6;k++)parent.pA[k]+=this.s[k];
        mm(this.mat,b.X,this.mat2);mm(b.X,this.mat2,this.mat,6,true);for(let k=0;k<36;k++)parent.IA[k]+=this.mat[k];
      }
    }
    this.a0.fill(0);for(let k=0;k<3;k++)this.a0[k+3]=-this.gravity[k];
    for(let i=0;i<this.bodies.length;i++){const l=this.model.links[i],b=this.bodies[i];
      mv(b.X,l.spec.parent<0?this.a0:this.bodies[l.spec.parent].a,b.a);for(let k=0;k<6;k++)b.a[k]+=b.c[k];
      if(this.model.floating&&i===0){mv(b.IA,b.a,this.t);for(let k=0;k<6;k++)this.baseRhs[k]=this.tau[k]-b.pA[k]-this.t[k];factor(b.IA,this.L,6);solve(this.L,this.baseRhs,this.s,6);for(let k=0;k<6;k++){this.qdd[k]=this.s[k];b.a[k]+=this.s[k];}}
      else{this.qdd[l.dof]=(b.u-dot(b.U,b.a))/b.d;for(let k=0;k<6;k++)b.a[k]+=l.S[k]*this.qdd[l.dof];}
    }return this.qdd;
  }
  denseForward(out:V){this.crba();this.rnea(this.zeros,this.bias);for(let i=0;i<this.model.nv;i++)this.rhs[i]=this.tau[i]-this.bias[i];factor(this.M,this.massL,this.model.nv);solve(this.massL,this.rhs,out,this.model.nv);}
  integrate(h:number){
    if(this.model.floating){
      // Convert the body-coordinate derivative to physical acceleration before the kick.
      // Transport the updated velocity into the new body frame after the rotation.
      rotation(this.orientation,this.R);
      for(let k=0;k<3;k++){const j=(k+1)%3,l=(k+2)%3;this.r[k]=this.v[k+3]+h*(this.qdd[k+3]+this.v[j]*this.v[l+3]-this.v[l]*this.v[j+3]);}
      mv(this.R,this.r,this.t,3);
      for(let k=0;k<3;k++){this.v[k]+=h*this.qdd[k];this.position[k]+=h*this.t[k];}
      expQuaternion(this.orientation,this.v[0],this.v[1],this.v[2],h);
      rotation(this.orientation,this.R);mv(this.R,this.t,this.r,3,true);
      for(let k=0;k<3;k++)this.v[k+3]=this.r[k];
    }
    for(let i=this.model.floating?6:0;i<this.model.nv;i++){this.v[i]+=h*this.qdd[i];this.q[i]+=h*this.v[i];}
  }
  step(dt=0.001,substeps=1){const h=dt/substeps;for(let s=0;s<substeps;s++){this.aba();this.integrate(h);}}
  energy(){this.kinematics();let e=0;for(let i=0;i<this.bodies.length;i++){const b=this.bodies[i],l=this.model.links[i];mv(l.I,b.v,this.t);e+=dot(b.v,this.t)/2;mv(b.R,l.com,this.r,3);for(let k=0;k<3;k++)e-=l.spec.mass*this.gravity[k]*(b.p[k]+this.r[k]);if(l.dof>=0)e+=l.rotor*this.v[l.dof]**2/2;}return e;}
  momentum(out:V){this.kinematics();out.fill(0);for(let i=0;i<this.bodies.length;i++){const b=this.bodies[i];mv(this.model.links[i].I,b.v,this.t);for(let k=0;k<3;k++){let angular=0,linear=0;for(let j=0;j<3;j++){angular+=b.R[k*3+j]*this.t[j];linear+=b.R[k*3+j]*this.t[j+3];}this.s[k]=angular;this.s[k+3]=linear;}for(let k=0;k<3;k++){const j=(k+1)%3,l=(k+2)%3;out[k]+=this.s[k]+b.p[j]*this.s[l+3]-b.p[l]*this.s[j+3];out[k+3]+=this.s[k+3];}}}
}
