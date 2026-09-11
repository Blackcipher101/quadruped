import {Fluid} from '../physics/fluid';
import {type Dynamics} from '../physics/dynamics';
import {type ContactSite} from '../physics/ground';
/** Subgrid sphere boundary coupling; reaction wrench is applied to the rigid
 * body at the same point. Coarse particles do not resolve foot-pad geometry. */
export class Water {
 readonly fluid=new Fluid();readonly reaction=new Float64Array(3);private centers:Float64Array;private velocities:Float64Array;
 constructor(readonly robot:Dynamics,readonly sites:ContactSite[]){this.centers=new Float64Array(sites.length*3);this.velocities=new Float64Array(sites.length*3);}
 reset(){this.fluid.reset();for(const wrench of this.robot.external)wrench.fill(0);}
 step(dt=.001){const d=this.robot;d.kinematics();for(const wrench of d.external)wrench.fill(0);this.reaction.fill(0);
 for(let j=0;j<this.sites.length;j++){const s=this.sites[j],b=d.bodies[s.body],p=s.point;for(let k=0;k<3;k++){this.centers[j*3+k]=b.p[k]+b.R[k*3]*p[0]+b.R[k*3+1]*p[1]+b.R[k*3+2]*p[2];let v=0;for(let l=0;l<3;l++){const a=(l+1)%3,c=(l+2)%3;v+=b.R[k*3+l]*(b.v[l+3]+b.v[a]*p[c]-b.v[c]*p[a]);}this.velocities[j*3+k]=v;}}
 for(let sub=0;sub<2;sub++){this.fluid.computeForces();this.couple(.5);this.fluid.integrate(dt/2);}}
 couple(weight=1){const f=this.fluid,d=this.robot;
 for(let i=0;i<f.count;i++)for(let j=0;j<this.sites.length;j++){const site=this.sites[j];if(site.radius<=0)continue;const p=i*3,c=j*3,dx=f.position[p]-this.centers[c],dy=f.position[p+1]-this.centers[c+1],dz=f.position[p+2]-this.centers[c+2],r=Math.hypot(dx,dy,dz),overlap=site.radius+.012-r;if(overlap<=0||r<1e-10)continue;const nx=dx/r,ny=dy/r,nz=dz/r,relative=(f.velocity[p]-this.velocities[c])*nx+(f.velocity[p+1]-this.velocities[c+1])*ny+(f.velocity[p+2]-this.velocities[c+2])*nz,magnitude=Math.max(0,800*overlap-2*Math.sqrt(800*f.mass)*relative),fx=nx*magnitude,fy=ny*magnitude,fz=nz*magnitude;
 f.force[p]+=fx;f.force[p+1]+=fy;f.force[p+2]+=fz;this.reaction[0]-=weight*fx;this.reaction[1]-=weight*fy;this.reaction[2]-=weight*fz;
 const body=d.bodies[site.body],R=body.R,w=d.external[site.body];const lx=-weight*(R[0]*fx+R[3]*fy+R[6]*fz),ly=-weight*(R[1]*fx+R[4]*fy+R[7]*fz),lz=-weight*(R[2]*fx+R[5]*fy+R[8]*fz);
 // Contact lies on the radial sphere normal; shifting along it leaves moment unchanged.
 const point=site.point;w[0]+=point[1]*lz-point[2]*ly;w[1]+=point[2]*lx-point[0]*lz;w[2]+=point[0]*ly-point[1]*lx;w[3]+=lx;w[4]+=ly;w[5]+=lz;
 }}
}
