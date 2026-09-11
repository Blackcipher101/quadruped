import { vec,factor,solve,type V } from './spatial';
/** Euclidean projection onto K_mu = {(n,t): n>=0, ||t||<=mu*n}. */
export function projectCone(n:number,x:number,y:number,mu:number,out:V){
 if(!(mu>=0))throw Error('Negative friction');const r=Math.hypot(x,y);
 if(n>=0&&r<=mu*n){out[0]=n;out[1]=x;out[2]=y;return;}
 const normal=(n+mu*r)/(1+mu*mu);if(normal<=0){out[0]=out[1]=out[2]=0;return;}
 out[0]=normal;const s=r>0?mu*normal/r:0;out[1]=x*s;out[2]=y*s;
}
/** Preallocated block projected Gauss-Seidel for the convex contact relaxation.
 * J rows are [normal,tangent1,tangent2]. lambda is impulse (N s).
 * W stores M^-1 J^T. A scalar block Lipschitz bound makes Euclidean SOC projection valid.
 */
export class Constraints {
 readonly J:V;readonly W:V;readonly lambda:V;readonly target:V;readonly cfm:V;readonly mu:V;
 readonly velocity:V;readonly lower:V;readonly upper:V;
 count=0;iterations=0;residual=0;pyramid=false;
 private L:V;private rhs:V;private column:V;private cone=vec(3);private lipschitz:V;private old=vec(3);private candidate=vec(3);
 constructor(readonly nv:number,readonly capacity:number){this.J=vec(3*capacity*nv);this.W=vec(3*capacity*nv);this.lambda=vec(3*capacity);this.target=vec(3*capacity);this.cfm=vec(3*capacity);this.mu=vec(capacity);this.velocity=vec(nv);this.L=vec(nv*nv);this.rhs=vec(nv);this.column=vec(nv);this.lipschitz=vec(capacity);this.lower=vec(capacity);this.upper=vec(capacity);this.upper.fill(Infinity);}
 prepare(M:V,freeVelocity:V){
  const n=this.nv;factor(M,this.L,n);this.velocity.set(freeVelocity);
  for(let row=0;row<this.count*3;row++){for(let j=0;j<n;j++)this.rhs[j]=this.J[row*n+j];solve(this.L,this.rhs,this.column,n);for(let j=0;j<n;j++)this.W[row*n+j]=this.column[j];}
  for(let c=0;c<this.count;c++){
   let bound=0;for(let r=0;r<3;r++){let sum=0;for(let k=0;k<3;k++){let a=0;for(let j=0;j<n;j++)a+=this.J[(3*c+r)*n+j]*this.W[(3*c+k)*n+j];if(r===k)a+=this.cfm[3*c+r];sum+=Math.abs(a);}bound=Math.max(bound,sum);}this.lipschitz[c]=bound>0?bound:1;
   projectCone(this.lambda[3*c],this.lambda[3*c+1],this.lambda[3*c+2],this.mu[c],this.cone);for(let r=0;r<3;r++){this.lambda[3*c+r]=this.cone[r];for(let j=0;j<n;j++)this.velocity[j]+=this.W[(3*c+r)*n+j]*this.cone[r];}
  }
 }
 solve(maxIterations=100,tolerance=1e-9){const n=this.nv;
  for(let iteration=0;iteration<maxIterations;iteration++){
   let maxResidual=0;
   for(let c=0;c<this.count;c++){
    const row=3*c;for(let r=0;r<3;r++){this.old[r]=this.lambda[row+r];let gradient=this.cfm[row+r]*this.old[r]-this.target[row+r];for(let j=0;j<n;j++)gradient+=this.J[(row+r)*n+j]*this.velocity[j];this.candidate[r]=this.old[r]-gradient/this.lipschitz[c];}
    if(this.pyramid){const normal=Math.max(0,this.candidate[0]),limit=this.mu[c]*normal;this.cone[0]=normal;this.cone[1]=Math.max(-limit,Math.min(limit,this.candidate[1]));this.cone[2]=Math.max(-limit,Math.min(limit,this.candidate[2]));}
    else projectCone(this.candidate[0],this.candidate[1],this.candidate[2],this.mu[c],this.cone);
    this.cone[0]=Math.max(this.lower[c],Math.min(this.upper[c],this.cone[0]));
    for(let r=0;r<3;r++){const delta=this.cone[r]-this.old[r];maxResidual=Math.max(maxResidual,Math.abs(delta)*this.lipschitz[c]);this.lambda[row+r]=this.cone[r];for(let j=0;j<n;j++)this.velocity[j]+=this.W[(row+r)*n+j]*delta;}
   }
   this.iterations=iteration+1;this.residual=maxResidual;if(maxResidual<tolerance)break;
  }
 }
}
