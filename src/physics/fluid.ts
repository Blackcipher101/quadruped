/** Weakly compressible 3D SPH. SI units, pairwise conservative pressure and
 * viscosity forces. This is a coarse free-surface approximation, not an
 * incompressible pressure-projection solver or a resolved turbulence model. */
export class Fluid {
 readonly position:Float64Array;readonly velocity:Float64Array;readonly force:Float64Array;readonly density:Float64Array;readonly pressure:Float64Array;
 readonly gravity=Float64Array.of(0,0,-9.81);readonly lower=Float64Array.of(.28,-.32,0);readonly upper=Float64Array.of(1.4,.32,1);
 readonly restDensity=1000;readonly spacing=.04;readonly h=.08;readonly mass=1000*.04**3;viscosity=.001;soundSpeed=12;walls=true;
 private heads=new Int32Array(4096);private next:Int32Array;private cells:Int32Array;
 constructor(readonly count=1120){this.position=new Float64Array(count*3);this.velocity=new Float64Array(count*3);this.force=new Float64Array(count*3);this.density=new Float64Array(count);this.pressure=new Float64Array(count);this.next=new Int32Array(count);this.cells=new Int32Array(count*3);this.reset();}
 reset(){this.velocity.fill(0);this.force.fill(0);for(let i=0;i<this.count;i++){this.position[i*3]=.3+(i%28)*this.spacing;this.position[i*3+1]=-.18+(Math.floor(i/28)%10)*this.spacing;this.position[i*3+2]=.025+Math.floor(i/280)*this.spacing;}}
 private hash(x:number,y:number,z:number){return ((x*73856093)^(y*19349663)^(z*83492791))&4095;}
 private grid(){this.heads.fill(-1);for(let i=0;i<this.count;i++){const p=i*3,x=Math.floor(this.position[p]/this.h),y=Math.floor(this.position[p+1]/this.h),z=Math.floor(this.position[p+2]/this.h);this.cells[p]=x;this.cells[p+1]=y;this.cells[p+2]=z;const key=this.hash(x,y,z);this.next[i]=this.heads[key];this.heads[key]=i;}}
 computeForces(){this.grid();const h=this.h,h2=h*h,m=this.mass,poly=315/(64*Math.PI*h**9),spiky=45/(Math.PI*h**6),B=this.restDensity*this.soundSpeed**2/7;
 for(let i=0;i<this.count;i++){let rho=0;const p=i*3;for(let x=-1;x<=1;x++)for(let y=-1;y<=1;y++)for(let z=-1;z<=1;z++){const cx=this.cells[p]+x,cy=this.cells[p+1]+y,cz=this.cells[p+2]+z;for(let j=this.heads[this.hash(cx,cy,cz)];j>=0;j=this.next[j]){const q=j*3;if(this.cells[q]!==cx||this.cells[q+1]!==cy||this.cells[q+2]!==cz)continue;const r2=(this.position[p]-this.position[q])**2+(this.position[p+1]-this.position[q+1])**2+(this.position[p+2]-this.position[q+2])**2;if(r2<h2)rho+=m*poly*(h2-r2)**3;}}this.density[i]=rho;this.pressure[i]=Math.max(0,B*((rho/this.restDensity)**7-1));for(let k=0;k<3;k++)this.force[p+k]=m*this.gravity[k];}
 for(let i=0;i<this.count;i++){const p=i*3;for(let x=-1;x<=1;x++)for(let y=-1;y<=1;y++)for(let z=-1;z<=1;z++){const cx=this.cells[p]+x,cy=this.cells[p+1]+y,cz=this.cells[p+2]+z;for(let j=this.heads[this.hash(cx,cy,cz)];j>=0;j=this.next[j]){if(j<=i)continue;const q=j*3;if(this.cells[q]!==cx||this.cells[q+1]!==cy||this.cells[q+2]!==cz)continue;const dx=this.position[p]-this.position[q],dy=this.position[p+1]-this.position[q+1],dz=this.position[p+2]-this.position[q+2],r=Math.hypot(dx,dy,dz);if(r>=h||r<1e-12)continue;const vx=this.velocity[p]-this.velocity[q],vy=this.velocity[p+1]-this.velocity[q+1],vz=this.velocity[p+2]-this.velocity[q+2],approach=vx*dx+vy*dy+vz*dz;
 // Monaghan compression viscosity alpha=.02; explicit numerical dissipation.
 const artificial=approach<0?-.02*this.soundSpeed*h*approach/((r*r+.01*h2)*.5*(this.density[i]+this.density[j])):0;
 const pressure=m*m*(this.pressure[i]/this.density[i]**2+this.pressure[j]/this.density[j]**2+artificial)*spiky*(h-r)**2/r;
 const visc=m*m*this.viscosity/(this.density[i]*this.density[j])*spiky*(h-r);
 for(let k=0;k<3;k++){const f=pressure*(this.position[p+k]-this.position[q+k])+visc*(this.velocity[q+k]-this.velocity[p+k]);this.force[p+k]+=f;this.force[q+k]-=f;}}}
 if(this.walls)for(let k=0;k<3;k++){const coordinate=this.position[p+k],v=this.velocity[p+k],radius=.012;const low=this.lower[k]+radius-coordinate,high=coordinate-(this.upper[k]-radius);if(low>0)this.force[p+k]+=Math.max(0,4000*low-2*Math.sqrt(4000*m)*v);if(high>0)this.force[p+k]-=Math.max(0,4000*high+2*Math.sqrt(4000*m)*v);}}
 }
 integrate(dt:number){for(let i=0;i<this.position.length;i++){this.velocity[i]+=dt*this.force[i]/this.mass;this.position[i]+=dt*this.velocity[i];}}
 step(dt=.001){this.computeForces();this.integrate(dt);}
}
