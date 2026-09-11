import { Dynamics } from './dynamics';
import { Constraints } from './constraints';
import { PointKinematics } from './kinematics';
import { Terrain } from './terrain';
import { vec,type V } from './spatial';
export interface ContactSite {body:number;point:V;radius:number;mu:number;restitution:number;}
/** Persistent analytic point/sphere-heightfield manifold for an articulated system. */
export class GroundSystem {
 readonly constraints:Constraints;readonly kinematics:PointKinematics;readonly forces:V;readonly active:Uint8Array;readonly depths:V;
 stiffness=Infinity;damping=0;maxPenetration=0;private free:V;private cache:V;private rowSite:Int32Array;private normal=vec(3);private tangent=vec(3);private bitangent=vec(3);private previousActive:Uint8Array;
 constructor(readonly dynamics:Dynamics,readonly sites:ContactSite[],readonly terrain=new Terrain()){
  this.constraints=new Constraints(dynamics.model.nv,sites.length+dynamics.model.nj);this.kinematics=new PointKinematics(dynamics);this.free=vec(dynamics.model.nv);this.cache=vec(3*sites.length);this.rowSite=new Int32Array(sites.length+dynamics.model.nj);this.forces=vec(3*sites.length);this.active=new Uint8Array(sites.length);this.previousActive=new Uint8Array(sites.length);this.depths=vec(sites.length);
 }
 reset(){this.cache.fill(0);this.previousActive.fill(0);this.active.fill(0);this.forces.fill(0);this.depths.fill(0);this.constraints.lambda.fill(0);}
 step(dt=.001,substeps=1){const h=dt/substeps,d=this.dynamics,n=d.model.nv,s=this.constraints;this.forces.fill(0);this.maxPenetration=0;
  for(let sub=0;sub<substeps;sub++){
   d.aba();for(let i=0;i<n;i++)this.free[i]=d.v[i]+h*d.qdd[i];d.crba();s.count=0;s.J.fill(0);s.target.fill(0);s.cfm.fill(0);s.lambda.fill(0);s.mu.fill(0);this.active.fill(0);
   for(let site=0;site<this.sites.length;site++){
    const contact=this.sites[site];this.kinematics.compute(contact.body,contact.point);const p=this.kinematics.position,ground=this.terrain.sample(p[0],p[1]);this.normal.set(this.terrain.normal);const gap=(p[2]-ground)*this.normal[2]-contact.radius;this.depths[site]=Math.max(0,-gap);this.maxPenetration=Math.max(this.maxPenetration,this.depths[site]);let vn=0;for(let k=0;k<3;k++)vn+=this.normal[k]*this.kinematics.velocity[k];
    if(gap>Math.max(.002,-vn*h)){this.cache.fill(0,site*3,site*3+3);continue;}
    this.active[site]=1;const c=s.count++,row=c*3;this.rowSite[c]=site;const norm=Math.hypot(this.normal[0],this.normal[2]);this.tangent[0]=this.normal[2]/norm;this.tangent[1]=0;this.tangent[2]=-this.normal[0]/norm;for(let k=0;k<3;k++){const j=(k+1)%3,l=(k+2)%3;this.bitangent[k]=this.normal[j]*this.tangent[l]-this.normal[l]*this.tangent[j];}
    for(let j=0;j<n;j++)for(let k=0;k<3;k++){s.J[row*n+j]+=this.normal[k]*this.kinematics.J[k*n+j];s.J[(row+1)*n+j]+=this.tangent[k]*this.kinematics.J[k*n+j];s.J[(row+2)*n+j]+=this.bitangent[k]*this.kinematics.J[k*n+j];}
    s.mu[c]=Math.min(contact.mu,this.terrain.friction(p[0],p[1]));const compliant=Number.isFinite(this.stiffness),erp=compliant?h*this.stiffness/(this.damping+h*this.stiffness):1;s.cfm[row]=compliant?1/(h*(this.damping+h*this.stiffness)):0;s.target[row]=-erp*gap/h;
    // Isolated Poisson impact: restitution impulse e times compression impulse.
    // For a single normal row this equals target -e*vn; no restitution at resting speed.
    if(!this.previousActive[site]&&vn<-.2)s.target[row]=Math.max(s.target[row],-contact.restitution*vn);
    for(let k=0;k<3;k++)s.lambda[row+k]=this.cache[site*3+k];
   }
   for(const link of d.model.links){if(link.dof<0)continue;const j=link.dof;let sign=0,gap=0;if(d.q[j]+h*this.free[j]<link.lower){sign=1;gap=d.q[j]-link.lower;}else if(d.q[j]+h*this.free[j]>link.upper){sign=-1;gap=link.upper-d.q[j];}if(sign){const c=s.count++;this.rowSite[c]=-1;s.J[c*3*n+j]=sign;s.target[c*3]=-gap/h;}}
   if(s.count){s.prepare(d.M,this.free);s.solve(100,1e-9);this.free.set(s.velocity);}
   for(let c=0;c<s.count;c++){const site=this.rowSite[c];if(site<0)continue;for(let k=0;k<3;k++){this.cache[site*3+k]=s.lambda[c*3+k];this.forces[site*3+k]+=s.lambda[c*3+k]/dt;}}
   this.previousActive.set(this.active);
   // Convert solved velocity impulse to generalized acceleration, then use the same integrator.
   for(let i=0;i<n;i++)d.qdd[i]=(this.free[i]-d.v[i])/h;d.integrate(h);
  }
 }
}
