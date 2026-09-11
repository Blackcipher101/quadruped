import {Dynamics} from './dynamics';
import {PointKinematics} from './kinematics';
import {Terrain} from './terrain';
import {GJK,EPA} from './convex';
import {SAT} from './sat';
import {SphereConvex} from './sphere-convex';
import {type Hull} from './shapes';
import {SweepAndPrune} from './sap';
import {vec,factor,solve,type V} from './spatial';
import {projectCone} from './constraints';
import {type ContactSite} from './ground';
const STRIDE=18,ROWS=6;
class SystemWork {inverse:V;L:V;rhs:V;column:V;free:V;point:PointKinematics;constructor(readonly d:Dynamics){const n=d.model.nv;this.inverse=vec(n*n);this.L=vec(n*n);this.rhs=vec(n);this.column=vec(n);this.free=vec(n);this.point=new PointKinematics(d);}}
/** Sparse coupled articulated-body contact solve. A row touches at most two trees.
 * Contact coordinates: normal, two tangent impulses, spin and two roll impulses.
 * All storage, Jacobians, manifold caches, and work buffers are allocated at construction.
 */
export class World {
 readonly systems:SystemWork[];readonly terrain=new Terrain();readonly broadphase:SweepAndPrune;
 readonly lambda:V;readonly points:V;readonly normals:V;readonly forceBySite:V;
 readonly a:Int32Array;readonly b:Int32Array;readonly source:Int32Array;
 readonly ja:V;readonly jb:V;readonly wa:V;readonly wb:V;
 readonly gap:V;readonly mu:V;readonly target:V;readonly compliance:V;readonly lipschitz:V;readonly key:Int32Array;
 profiling=false;readonly profile=vec(4);count=0;iterations=0;residual=0;maxPenetration=0;stiffness=Infinity;damping=0;pyramid=false;
 private activeSystems:Uint8Array;private nextActive:Uint8Array;private sphere=new SphereConvex();private sphereCenter=vec(3);private pairAge:Int32Array;private cache:V;private cacheAge:Int32Array;private frame=0;private normal=vec(3);private basis=vec(18);private local=vec(3);private point=vec(3);private tangent=vec(3);private cone=vec(3);private gradient=vec(6);private candidate=vec(6);private sat=new SAT();private gjk=new GJK();private epa=new EPA();private bound:V;private restitution:V;private fresh:Uint8Array;private spin:V;private rolling:V;
 constructor(dynamics:Dynamics[],readonly hulls:Hull[],readonly sites:ContactSite[]=[],readonly capacity=768){
  this.activeSystems=new Uint8Array(dynamics.length);this.nextActive=new Uint8Array(dynamics.length);this.pairAge=new Int32Array(hulls.length*hulls.length);this.pairAge.fill(-2);this.systems=dynamics.map(d=>new SystemWork(d));this.broadphase=new SweepAndPrune(hulls.length);this.broadphase.count=hulls.length;
  this.lambda=vec(capacity*ROWS);this.points=vec(capacity*3);this.normals=vec(capacity*3);this.a=new Int32Array(capacity);this.b=new Int32Array(capacity);this.source=new Int32Array(capacity);this.key=new Int32Array(capacity);this.ja=vec(capacity*ROWS*STRIDE);this.jb=vec(capacity*ROWS*STRIDE);this.wa=vec(capacity*ROWS*STRIDE);this.wb=vec(capacity*ROWS*STRIDE);this.gap=vec(capacity);this.mu=vec(capacity);this.target=vec(capacity);this.compliance=vec(capacity);this.lipschitz=vec(capacity);this.bound=vec(capacity*ROWS);this.restitution=vec(capacity);this.fresh=new Uint8Array(capacity);this.spin=vec(capacity);this.rolling=vec(capacity);this.forceBySite=vec(sites.length*3);
  // Stable keys reserve 64 vertex slots per hull/pair, plus sample and joint-limit slots.
  const keys=(hulls.length*hulls.length+hulls.length+2)*64+sites.length+sites.length*hulls.length+256;this.cache=vec(keys*ROWS);this.cacheAge=new Int32Array(keys);this.cacheAge.fill(-2);
 }
 reset(){this.pairAge.fill(-2);this.cache.fill(0);this.cacheAge.fill(-2);this.frame=0;this.count=0;this.forceBySite.fill(0);}
 private rowVelocity(c:number,r:number){let v=0;const start=(c*ROWS+r)*STRIDE,A=this.systems[this.a[c]],B=this.b[c]>=0?this.systems[this.b[c]]:undefined;for(let j=0;j<A.d.model.nv;j++)v+=this.ja[start+j]*A.free[j];if(B)for(let j=0;j<B.d.model.nv;j++)v+=this.jb[start+j]*B.free[j];return v;}
 private apply(c:number,r:number,impulse:number){const start=(c*ROWS+r)*STRIDE,A=this.systems[this.a[c]],B=this.b[c]>=0?this.systems[this.b[c]]:undefined;for(let j=0;j<A.d.model.nv;j++)A.free[j]+=this.wa[start+j]*impulse;if(B)for(let j=0;j<B.d.model.nv;j++)B.free[j]+=this.wb[start+j]*impulse;}
 private jacobian(c:number,side:number,body:number){const system=this.systems[side===0?this.a[c]:this.b[c]],d=system.d,b=d.bodies[body],J=side===0?this.ja:this.jb,sign=side===0?1:-1,n=d.model.nv;
  for(let k=0;k<3;k++)this.local[k]=b.R[k]*(this.point[0]-b.p[0])+b.R[3+k]*(this.point[1]-b.p[1])+b.R[6+k]*(this.point[2]-b.p[2]);system.point.compute(body,this.local);
  const start=c*ROWS*STRIDE;for(let r=0;r<ROWS;r++)for(let j=0;j<n;j++)J[start+r*STRIDE+j]=0;
  for(let r=0;r<3;r++)for(let j=0;j<n;j++)for(let k=0;k<3;k++)J[start+r*STRIDE+j]+=sign*this.basis[r*3+k]*system.point.J[k*n+j];
  for(let i=body;i>=0;i=d.model.links[i].spec.parent){const link=d.model.links[i],work=d.bodies[i],free=i===0&&d.model.floating;for(let column=0;column<(free?3:1);column++){const j=free?column:link.dof;if(j<0)continue;for(let k=0;k<3;k++)this.tangent[k]=free?work.R[k*3+column]:work.R[k*3]*link.axis[0]+work.R[k*3+1]*link.axis[1]+work.R[k*3+2]*link.axis[2];for(let r=0;r<3;r++)for(let k=0;k<3;k++)J[start+(r+3)*STRIDE+j]+=sign*this.basis[r*3+k]*this.tangent[k];}}
 }
 private add(a:number,bodyA:number,b:number,bodyB:number,gap:number,mu:number,key:number,source=-1,restitution=.05,rolling=.003,spinning=.001){
  if(this.count===this.capacity)throw Error('Contact capacity exceeded');const c=this.count++;this.a[c]=a;this.b[c]=b;this.source[c]=source;this.gap[c]=gap;this.mu[c]=mu;this.key[c]=key;this.restitution[c]=restitution;this.rolling[c]=rolling;this.spin[c]=spinning;this.points.set(this.point,c*3);this.normals.set(this.normal,c*3);this.maxPenetration=Math.max(this.maxPenetration,-gap);
  this.basis.set(this.normal);const x=this.normal[0],y=this.normal[1],z=this.normal[2],norm=Math.hypot(x,z);if(norm>1e-8){this.basis[3]=z/norm;this.basis[4]=0;this.basis[5]=-x/norm;}else{this.basis[3]=1;this.basis[4]=this.basis[5]=0;}this.basis[6]=y*this.basis[5]-z*this.basis[4];this.basis[7]=z*this.basis[3]-x*this.basis[5];this.basis[8]=x*this.basis[4]-y*this.basis[3];this.jacobian(c,0,bodyA);if(b>=0)this.jacobian(c,1,bodyB);
  this.fresh[c]=this.cacheAge[key]===this.frame-1?0:1;
  for(let r=0;r<ROWS;r++)this.lambda[c*ROWS+r]=this.fresh[c]?0:this.cache[key*ROWS+r];
 }
 private inside(shape:Hull,x:number,y:number,z:number,tolerance=.003){const R=shape.rotation,p=shape.position,dx=x-p[0],dy=y-p[1],dz=z-p[2],lx=R[0]*dx+R[3]*dy+R[6]*dz,ly=R[1]*dx+R[4]*dy+R[7]*dz,lz=R[2]*dx+R[5]*dy+R[8]*dz;for(let i=0;i<shape.planes.length;i+=4)if(shape.planes[i]*lx+shape.planes[i+1]*ly+shape.planes[i+2]*lz-shape.planes[i+3]>tolerance)return false;return true;}
 private detect(h:number){this.count=0;const nh=this.hulls.length;
  for(let i=0;i<nh;i++){const hull=this.hulls[i],v=hull.vertices,R=hull.rotation,p=hull.position,b=this.broadphase.bounds;for(let k=0;k<3;k++){b[i*6+k]=Infinity;b[i*6+k+3]=-Infinity;}
   for(let j=0;j<v.length;j+=3){for(let k=0;k<3;k++){const x=p[k]+R[k*3]*v[j]+R[k*3+1]*v[j+1]+R[k*3+2]*v[j+2];hull.world[j+k]=x;b[i*6+k]=Math.min(b[i*6+k],x-.003);b[i*6+k+3]=Math.max(b[i*6+k+3],x+.003);}const x=hull.world[j],y=hull.world[j+1],z=hull.world[j+2],height=this.terrain.sample(x,y),gap=(z-height)*this.terrain.normal[2];if(!hull.terrainContact||gap>.003)continue;this.point[0]=x;this.point[1]=y;this.point[2]=z;this.normal.set(this.terrain.normal);this.add(hull.system,hull.body,-1,0,gap,Math.min(hull.mu,this.terrain.friction(x,y)),i*64+j/3,-1,hull.restitution,hull.rolling,hull.spinning);}
  }
  this.broadphase.update();for(let pair=0;pair<this.broadphase.pairCount;pair++){const ai=this.broadphase.pairs[pair*2],bi=this.broadphase.pairs[pair*2+1],A=this.hulls[ai],B=this.hulls[bi];if(A.system===B.system)continue;const hit=this.gjk.query(A,B);let depth=0;if(!hit){if(this.gjk.distance>.003)continue;if(this.gjk.distance<1e-6){if(!this.sat.query(A,B))continue;this.normal.set(this.sat.normal);depth=-this.sat.gap;}else{this.normal.set(this.gjk.normal);depth=-this.gjk.distance;}}else if(this.pairAge[ai*nh+bi]===this.frame-1&&this.sat.query(A,B)){this.normal.set(this.sat.normal);depth=-this.sat.gap;}else if(this.epa.query(A,B)){this.normal.set(this.epa.normal);depth=this.epa.depth;}else{if(!this.sat.query(A,B))continue;this.normal.set(this.sat.normal);depth=-this.sat.gap;
  }
   this.pairAge[ai*nh+bi]=this.frame;let minA=Infinity,maxB=-Infinity;for(let i=0;i<A.world.length;i+=3)minA=Math.min(minA,this.normal[0]*A.world[i]+this.normal[1]*A.world[i+1]+this.normal[2]*A.world[i+2]);for(let i=0;i<B.world.length;i+=3)maxB=Math.max(maxB,this.normal[0]*B.world[i]+this.normal[1]*B.world[i+1]+this.normal[2]*B.world[i+2]);const start=this.count,key=(nh+ai*nh+bi)*64;
   for(let side=0;side<2;side++){const shape=side===0?A:B,other=side===0?B:A;for(let i=0;i<shape.world.length;i+=3){const x=shape.world[i],y=shape.world[i+1],z=shape.world[i+2],projection=this.normal[0]*x+this.normal[1]*y+this.normal[2]*z;if(side===0?projection>minA+.001:projection<maxB-.001)continue;if(!this.inside(other,x,y,z,.003+Math.max(0,depth)))continue;let duplicate=false;for(let c=start;c<this.count;c++)if((this.points[c*3]-x)**2+(this.points[c*3+1]-y)**2+(this.points[c*3+2]-z)**2<1e-8)duplicate=true;if(duplicate)continue;this.point[0]=x;this.point[1]=y;this.point[2]=z;this.add(A.system,A.body,B.system,B.body,minA-maxB,Math.min(A.mu,B.mu),key+side*24+i/3,-1,Math.min(A.restitution,B.restitution));}}
   if(this.count===start){for(let k=0;k<3;k++)this.point[k]=hit?(this.epa.pointA[k]+this.epa.pointB[k])*.5:(this.gjk.pointA[k]+this.gjk.pointB[k])*.5;this.add(A.system,A.body,B.system,B.body,-depth,Math.min(A.mu,B.mu),key+63,-1,Math.min(A.restitution,B.restitution));}
  }
  const offset=(nh*nh+nh+1)*64;
  for(let i=0;i<this.sites.length;i++){const site=this.sites[i],s=this.systems[0];s.point.compute(site.body,site.point);const p=s.point.position,height=this.terrain.sample(p[0],p[1]),gap=(p[2]-height)*this.terrain.normal[2]-site.radius;if(gap<.003){this.point.set(p);this.normal.set(this.terrain.normal);for(let k=0;k<3;k++)this.point[k]-=this.normal[k]*site.radius;this.add(0,site.body,-1,0,gap,Math.min(site.mu,this.terrain.friction(p[0],p[1])),offset+i,i,site.restitution);}}
  for(let i=0;i<this.sites.length;i++){const site=this.sites[i];if(site.radius<=0)continue;this.systems[0].point.compute(site.body,site.point);this.sphereCenter.set(this.systems[0].point.position);const p=this.sphereCenter,r=site.radius+.003;
   for(let hi=0;hi<nh;hi++){const shape=this.hulls[hi];if(shape.system===0)continue;const bounds=this.broadphase.bounds;if(p[0]+r<bounds[hi*6]||p[0]-r>bounds[hi*6+3]||p[1]+r<bounds[hi*6+1]||p[1]-r>bounds[hi*6+4]||p[2]+r<bounds[hi*6+2]||p[2]-r>bounds[hi*6+5])continue;const gap=this.sphere.query(shape,p)-site.radius;if(gap>.003)continue;this.normal.set(this.sphere.normal);for(let k=0;k<3;k++)this.point[k]=p[k]-this.normal[k]*site.radius;this.add(0,site.body,shape.system,shape.body,gap,Math.min(site.mu,shape.mu),offset+this.sites.length+128+i*nh+hi,i,Math.min(site.restitution,shape.restitution));}
  }
  // Hard joint limits share the same impulse solve. Tangential/angular rows are zero.
  for(let si=0;si<this.systems.length;si++){const s=this.systems[si],d=s.d;for(let j=0;j<d.model.links.length;j++){const link=d.model.links[j],q=link.dof;if(q<0)continue;let sign=0,gap=0;if(d.q[q]+h*s.free[q]<link.lower){sign=1;gap=d.q[q]-link.lower;}else if(d.q[q]+h*s.free[q]>link.upper){sign=-1;gap=link.upper-d.q[q];}if(sign){const c=this.count++;if(c>=this.capacity)throw Error('Contact capacity exceeded');this.a[c]=si;this.b[c]=-1;this.source[c]=-1;this.gap[c]=gap;this.mu[c]=0;this.key[c]=offset+this.sites.length+q;this.restitution[c]=0;this.rolling[c]=this.spin[c]=0;this.fresh[c]=1;this.lambda.fill(0,c*ROWS,c*ROWS+ROWS);this.ja.fill(0,c*ROWS*STRIDE,(c+1)*ROWS*STRIDE);this.ja[c*ROWS*STRIDE+q]=sign;}}}
 }
 private prepare(h:number){for(let c=0;c<this.count;c++){const A=this.systems[this.a[c]],B=this.b[c]>=0?this.systems[this.b[c]]:undefined;const erp=Number.isFinite(this.stiffness)?h*this.stiffness/(this.damping+h*this.stiffness):1;this.compliance[c]=Number.isFinite(this.stiffness)?1/(h*(this.damping+h*this.stiffness)):0;this.target[c]=-erp*this.gap[c]/h;
  const vn=this.rowVelocity(c,0);if(this.fresh[c]&&vn<-.2)this.target[c]=Math.max(this.target[c],-this.restitution[c]*vn);
  for(let side=0;side<2;side++){const system=side===0?A:B;if(!system)continue;const J=side===0?this.ja:this.jb,W=side===0?this.wa:this.wb,n=system.d.model.nv;for(let r=0;r<ROWS;r++){const start=(c*ROWS+r)*STRIDE;for(let i=0;i<n;i++){let sum=0;for(let j=0;j<n;j++)sum+=system.inverse[i*n+j]*J[start+j];W[start+i]=sum;}}}
  let bound=0;for(let r=0;r<ROWS;r++){let sum=0;for(let k=0;k<ROWS;k++){let x=0;for(let j=0;j<A.d.model.nv;j++)x+=this.ja[(c*ROWS+r)*STRIDE+j]*this.wa[(c*ROWS+k)*STRIDE+j];if(B)for(let j=0;j<B.d.model.nv;j++)x+=this.jb[(c*ROWS+r)*STRIDE+j]*this.wb[(c*ROWS+k)*STRIDE+j];if(r===k){this.bound[c*ROWS+r]=x; x+=this.compliance[c];}if(r<3&&k<3)sum+=Math.abs(x);}if(r<3)bound=Math.max(bound,sum);}this.lipschitz[c]=Math.max(bound,1e-12);
 }
 for(let c=0;c<this.count;c++)for(let r=0;r<ROWS;r++)this.apply(c,r,this.lambda[c*ROWS+r]);}
 private solve(){this.activeSystems.fill(1);for(let iteration=0;iteration<120;iteration++){this.nextActive.fill(0);let residual=0;for(let c=0;c<this.count;c++){if(!this.activeSystems[this.a[c]]&&(this.b[c]<0||!this.activeSystems[this.b[c]]))continue;let contactResidual=0;const base=c*ROWS,L=this.lipschitz[c];for(let r=0;r<3;r++){this.gradient[r]=this.rowVelocity(c,r)+this.compliance[c]*this.lambda[base+r]-(r===0?this.target[c]:0);this.candidate[r]=this.lambda[base+r]-this.gradient[r]/L;}if(this.pyramid){const n=Math.max(0,this.candidate[0]),lim=this.mu[c]*n;this.cone[0]=n;this.cone[1]=Math.max(-lim,Math.min(lim,this.candidate[1]));this.cone[2]=Math.max(-lim,Math.min(lim,this.candidate[2]));}else projectCone(this.candidate[0],this.candidate[1],this.candidate[2],this.mu[c],this.cone);
  for(let r=0;r<3;r++){const delta=this.cone[r]-this.lambda[base+r];this.lambda[base+r]=this.cone[r];this.apply(c,r,delta);contactResidual=Math.max(contactResidual,Math.abs(delta)*L);}
  // Angular impulse dimensions N m s. Friction lengths are expressed in metres.
  for(let r=3;r<6;r++){const den=this.bound[base+r]+this.compliance[c];this.candidate[r]=den>1e-12?this.lambda[base+r]-(this.rowVelocity(c,r)+this.compliance[c]*this.lambda[base+r])/den:0;}
  const spinLimit=this.spin[c]*this.lambda[base],rollLimit=this.rolling[c]*this.lambda[base],length=Math.hypot(this.candidate[4],this.candidate[5]),scale=length>rollLimit?rollLimit/length:1;this.candidate[3]=Math.max(-spinLimit,Math.min(spinLimit,this.candidate[3]));this.candidate[4]*=scale;this.candidate[5]*=scale;
  for(let r=3;r<6;r++){const delta=this.candidate[r]-this.lambda[base+r];this.lambda[base+r]=this.candidate[r];this.apply(c,r,delta);contactResidual=Math.max(contactResidual,Math.abs(delta)*this.bound[base+r]);}
 residual=Math.max(residual,contactResidual);if(contactResidual>=1e-7){this.nextActive[this.a[c]]=1;if(this.b[c]>=0)this.nextActive[this.b[c]]=1;} }this.activeSystems.set(this.nextActive);this.iterations=iteration+1;this.residual=residual;if(residual<1e-7)break;}}
 step(dt=.001,substeps=2){const h=dt/substeps;this.maxPenetration=0;this.forceBySite.fill(0);for(let sub=0;sub<substeps;sub++){this.frame++;let clock=this.profiling?performance.now():0;
  for(const s of this.systems){const d=s.d,n=d.model.nv;d.aba();d.crba();factor(d.M,s.L,n);for(let col=0;col<n;col++){s.rhs.fill(0);s.rhs[col]=1;solve(s.L,s.rhs,s.column,n);for(let row=0;row<n;row++)s.inverse[row*n+col]=s.column[row];}for(let j=0;j<n;j++)s.free[j]=d.v[j]+h*d.qdd[j];if(d.model.floating)for(let k=0;k<3;k++){const j=(k+1)%3,l=(k+2)%3;s.free[k+3]+=h*(d.v[j]*d.v[l+3]-d.v[l]*d.v[j+3]);}}
  if(this.profiling){this.profile[0]+=performance.now()-clock;clock=performance.now();}this.detect(h);if(this.profiling){this.profile[1]+=performance.now()-clock;clock=performance.now();}this.prepare(h);if(this.profiling){this.profile[2]+=performance.now()-clock;clock=performance.now();}this.solve();if(this.profiling)this.profile[3]+=performance.now()-clock;for(let c=0;c<this.count;c++){const key=this.key[c];this.cacheAge[key]=this.frame;for(let r=0;r<ROWS;r++)this.cache[key*ROWS+r]=this.lambda[c*ROWS+r];if(this.source[c]>=0)for(let r=0;r<3;r++)this.forceBySite[this.source[c]*3+r]+=this.lambda[c*ROWS+r]/dt;}
  for(const s of this.systems){const d=s.d;for(let j=0;j<d.model.nv;j++)d.qdd[j]=(s.free[j]-d.v[j])/h;if(d.model.floating)for(let k=0;k<3;k++){const j=(k+1)%3,l=(k+2)%3;d.qdd[k+3]-=d.v[j]*d.v[l+3]-d.v[l]*d.v[j+3];}d.integrate(h);}
 }}
}
