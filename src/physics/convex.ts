import {vec,type V} from './spatial';
export interface Convex { vertices:V; rotation:V; position:V; }
/** Signed tetrahedral volume integration. Input must be a closed outward-wound triangle mesh. */
export function meshInertia(vertices:V,triangles:Uint32Array,density:number){
 let volume=0;const first=vec(3),second=vec(9),com=vec(3),inertia=vec(9);
 for(let f=0;f<triangles.length;f+=3){const a=triangles[f]*3,b=triangles[f+1]*3,c=triangles[f+2]*3;
  const v=(vertices[a]*(vertices[b+1]*vertices[c+2]-vertices[b+2]*vertices[c+1])+vertices[a+1]*(vertices[b+2]*vertices[c]-vertices[b]*vertices[c+2])+vertices[a+2]*(vertices[b]*vertices[c+1]-vertices[b+1]*vertices[c]))/6;volume+=v;
  for(let i=0;i<3;i++){const si=vertices[a+i]+vertices[b+i]+vertices[c+i];first[i]+=v*si/4;for(let j=0;j<3;j++){const sj=vertices[a+j]+vertices[b+j]+vertices[c+j];second[i*3+j]+=v*(si*sj+vertices[a+i]*vertices[a+j]+vertices[b+i]*vertices[b+j]+vertices[c+i]*vertices[c+j])/20;}}
 }
 if(!(volume>0)||!(density>0))throw Error('Mesh must have positive signed volume and density');const mass=density*volume;for(let i=0;i<3;i++)com[i]=first[i]/volume;const trace=second[0]+second[4]+second[8],c2=com[0]**2+com[1]**2+com[2]**2;
 for(let i=0;i<3;i++)for(let j=0;j<3;j++)inertia[i*3+j]=density*((i===j?trace:0)-second[i*3+j])-mass*((i===j?c2:0)-com[i]*com[j]);return {mass,volume,com,inertia};
}
function support(shape:Convex,x:number,y:number,z:number,out:V,offset:number){const R=shape.rotation,v=shape.vertices;const dx=R[0]*x+R[3]*y+R[6]*z,dy=R[1]*x+R[4]*y+R[7]*z,dz=R[2]*x+R[5]*y+R[8]*z;let best=-Infinity,index=0;for(let i=0;i<v.length;i+=3){const score=v[i]*dx+v[i+1]*dy+v[i+2]*dz;if(score>best){best=score;index=i;}}for(let k=0;k<3;k++)out[offset+k]=shape.position[k]+R[k*3]*v[index]+R[k*3+1]*v[index+1]+R[k*3+2]*v[index+2];}
/** Deterministic support-mapped GJK distance. Exhaustive simplex subsets avoid winding cases.
 * EPA is deliberately a separate deliverable; this routine does not invent penetration depths.
 */
export class GJK {
 readonly normal=vec(3);readonly pointA=vec(3);readonly pointB=vec(3);distance=0;iterations=0;intersect=false;
 private simplex=vec(36);private compact=vec(36);private candidate=vec(9);private weights=vec(4);private bestWeights=vec(4);private indices=new Int32Array(4);private matrix=vec(25);private rhs=vec(5);private closest=vec(3);private count=0;
 private minkowski(a:Convex,b:Convex,x:number,y:number,z:number){support(a,x,y,z,this.candidate,3);support(b,-x,-y,-z,this.candidate,6);for(let k=0;k<3;k++)this.candidate[k]=this.candidate[k+3]-this.candidate[k+6];}
 private reduce(){
  let best=Infinity,bestMask=0;this.bestWeights.fill(0);
  for(let mask=1;mask<(1<<this.count);mask++){
   let n=0;for(let i=0;i<this.count;i++)if(mask&(1<<i))this.indices[n++]=i;
   const size=n+1;this.matrix.fill(0);this.rhs.fill(0);this.rhs[n]=1;
   for(let i=0;i<n;i++){for(let j=0;j<n;j++){let s=0;for(let k=0;k<3;k++)s+=this.simplex[this.indices[i]*9+k]*this.simplex[this.indices[j]*9+k];this.matrix[i*size+j]=s;}this.matrix[i*size+n]=1;this.matrix[n*size+i]=1;}
   let valid=true;
   for(let i=0;i<size;i++){let pivot=i;for(let j=i+1;j<size;j++)if(Math.abs(this.matrix[j*size+i])>Math.abs(this.matrix[pivot*size+i]))pivot=j;if(Math.abs(this.matrix[pivot*size+i])<1e-15){valid=false;break;}for(let k=i;k<size;k++){const t=this.matrix[i*size+k];this.matrix[i*size+k]=this.matrix[pivot*size+k];this.matrix[pivot*size+k]=t;}const t=this.rhs[i];this.rhs[i]=this.rhs[pivot];this.rhs[pivot]=t;const d=this.matrix[i*size+i];for(let k=i;k<size;k++)this.matrix[i*size+k]/=d;this.rhs[i]/=d;for(let j=0;j<size;j++)if(j!==i){const f=this.matrix[j*size+i];for(let k=i;k<size;k++)this.matrix[j*size+k]-=f*this.matrix[i*size+k];this.rhs[j]-=f*this.rhs[i];}}
   if(!valid)continue;for(let i=0;i<n;i++)if(this.rhs[i]<-1e-10)valid=false;if(!valid)continue;this.closest.fill(0);for(let i=0;i<n;i++)for(let k=0;k<3;k++)this.closest[k]+=this.rhs[i]*this.simplex[this.indices[i]*9+k];const dist=this.closest[0]**2+this.closest[1]**2+this.closest[2]**2;
   if(dist<best){best=dist;bestMask=mask;this.bestWeights.fill(0);for(let i=0;i<n;i++)this.bestWeights[this.indices[i]]=this.rhs[i];}
  }
  this.pointA.fill(0);this.pointB.fill(0);let count=0;
  for(let i=0;i<this.count;i++)if(bestMask&(1<<i)){for(let k=0;k<3;k++){this.pointA[k]+=this.bestWeights[i]*this.simplex[i*9+k+3];this.pointB[k]+=this.bestWeights[i]*this.simplex[i*9+k+6];}for(let k=0;k<9;k++)this.compact[count*9+k]=this.simplex[i*9+k];count++;}this.count=count;this.simplex.set(this.compact);for(let k=0;k<3;k++)this.closest[k]=this.pointA[k]-this.pointB[k];return best;
 }
 query(a:Convex,b:Convex){this.count=1;this.minkowski(a,b,1,0,0);this.simplex.set(this.candidate);let dist=this.reduce();this.intersect=false;
  for(let iteration=0;iteration<64;iteration++){this.iterations=iteration+1;if(dist<1e-20){this.intersect=true;break;}this.minkowski(a,b,-this.closest[0],-this.closest[1],-this.closest[2]);let improvement=dist;for(let k=0;k<3;k++)improvement-=this.closest[k]*this.candidate[k];if(improvement<1e-12*Math.max(1,dist))break;if(this.count>=4)throw Error('GJK simplex failed to reduce');for(let k=0;k<9;k++)this.simplex[this.count*9+k]=this.candidate[k];this.count++;dist=this.reduce();}
  this.distance=Math.sqrt(dist);for(let k=0;k<3;k++)this.normal[k]=this.distance>1e-10?this.closest[k]/this.distance:0;return this.intersect;
 }
}
/** Expanding polytope penetration depth; normal points from B toward A. */
export class EPA {
 readonly normal=vec(3);readonly pointA=vec(3);readonly pointB=vec(3);depth=0;iterations=0;
 private vertices=vec(9*128);private initial=vec(9*14);private faces=new Int32Array(512*3);private planes=vec(512*4);private active=new Uint8Array(512);private edges=new Int32Array(1536);private candidate=vec(9);private nv=0;private nf=0;private ne=0;
 private support(a:Convex,b:Convex,x:number,y:number,z:number){support(a,x,y,z,this.candidate,3);support(b,-x,-y,-z,this.candidate,6);for(let k=0;k<3;k++)this.candidate[k]=this.candidate[k+3]-this.candidate[k+6];}
 private face(a:number,b:number,c:number){if(this.nf===512)throw Error('EPA face capacity exceeded');const v=this.vertices,ax=v[b*9]-v[a*9],ay=v[b*9+1]-v[a*9+1],az=v[b*9+2]-v[a*9+2],bx=v[c*9]-v[a*9],by=v[c*9+1]-v[a*9+1],bz=v[c*9+2]-v[a*9+2];let x=ay*bz-az*by,y=az*bx-ax*bz,z=ax*by-ay*bx;const norm=Math.hypot(x,y,z);if(norm<1e-14)return;x/=norm;y/=norm;z/=norm;let d=x*v[a*9]+y*v[a*9+1]+z*v[a*9+2];if(d<0){const t=b;b=c;c=t;x=-x;y=-y;z=-z;d=-d;}const f=this.nf++;this.faces[f*3]=a;this.faces[f*3+1]=b;this.faces[f*3+2]=c;this.planes[f*4]=x;this.planes[f*4+1]=y;this.planes[f*4+2]=z;this.planes[f*4+3]=d;this.active[f]=1;}
 private edge(a:number,b:number){for(let i=0;i<this.ne;i++)if(this.edges[i*2]===b&&this.edges[i*2+1]===a){this.ne--;this.edges[i*2]=this.edges[this.ne*2];this.edges[i*2+1]=this.edges[this.ne*2+1];return;}if(this.ne>=768)throw Error('EPA edge capacity exceeded');this.edges[this.ne*2]=a;this.edges[this.ne*2+1]=b;this.ne++;}
 private determinant(ax:number,ay:number,az:number,bx:number,by:number,bz:number,cx:number,cy:number,cz:number){return ax*(by*cz-bz*cy)-ay*(bx*cz-bz*cx)+az*(bx*cy-by*cx);}
 query(a:Convex,b:Convex){
  this.depth=0;this.nv=0;this.nf=0;this.active.fill(0);let n=0;
  for(let axis=0;axis<3;axis++)for(let sign=-1;sign<=1;sign+=2){this.support(a,b,axis===0?sign:0,axis===1?sign:0,axis===2?sign:0);for(let k=0;k<9;k++)this.initial[n*9+k]=this.candidate[k];n++;}
  for(let mask=0;mask<8;mask++){this.support(a,b,mask&1?1:-1,mask&2?1:-1,mask&4?1:-1);for(let k=0;k<9;k++)this.initial[n*9+k]=this.candidate[k];n++;}
  let found=false;const v=this.initial;
  search:for(let i=0;i<n-3;i++)for(let j=i+1;j<n-2;j++)for(let k=j+1;k<n-1;k++)for(let l=k+1;l<n;l++){
   const ax=v[i*9]-v[l*9],ay=v[i*9+1]-v[l*9+1],az=v[i*9+2]-v[l*9+2],bx=v[j*9]-v[l*9],by=v[j*9+1]-v[l*9+1],bz=v[j*9+2]-v[l*9+2],cx=v[k*9]-v[l*9],cy=v[k*9+1]-v[l*9+1],cz=v[k*9+2]-v[l*9+2],x=-v[l*9],y=-v[l*9+1],z=-v[l*9+2];const det=this.determinant(ax,ay,az,bx,by,bz,cx,cy,cz);if(Math.abs(det)<1e-14)continue;
   const wa=this.determinant(x,y,z,bx,by,bz,cx,cy,cz)/det,wb=this.determinant(ax,ay,az,x,y,z,cx,cy,cz)/det,wc=this.determinant(ax,ay,az,bx,by,bz,x,y,z)/det;
   if(Math.min(wa,wb,wc,1-wa-wb-wc)>1e-9){for(let t=0;t<9;t++){this.vertices[t]=v[i*9+t];this.vertices[9+t]=v[j*9+t];this.vertices[18+t]=v[k*9+t];this.vertices[27+t]=v[l*9+t];}found=true;break search;}
  }
  if(!found)return false;this.nv=4;this.face(0,1,2);this.face(0,3,1);this.face(0,2,3);this.face(1,3,2);
  for(let iteration=0;iteration<96;iteration++){
   this.iterations=iteration+1;let best=-1,min=Infinity;for(let f=0;f<this.nf;f++)if(this.active[f]&&this.planes[f*4+3]<min){min=this.planes[f*4+3];best=f;}if(best<0)return false;const p=best*4,x=this.planes[p],y=this.planes[p+1],z=this.planes[p+2];this.support(a,b,x,y,z);const distance=x*this.candidate[0]+y*this.candidate[1]+z*this.candidate[2];
   if(distance-min<1e-8){this.depth=min;this.normal[0]=-x;this.normal[1]=-y;this.normal[2]=-z;
    const ia=this.faces[best*3]*9,ib=this.faces[best*3+1]*9,ic=this.faces[best*3+2]*9;const w=this.vertices;
    const ux=w[ib]-w[ia],uy=w[ib+1]-w[ia+1],uz=w[ib+2]-w[ia+2],vx=w[ic]-w[ia],vy=w[ic+1]-w[ia+1],vz=w[ic+2]-w[ia+2],px=x*min-w[ia],py=y*min-w[ia+1],pz=z*min-w[ia+2];const uu=ux*ux+uy*uy+uz*uz,uv=ux*vx+uy*vy+uz*vz,vv=vx*vx+vy*vy+vz*vz,pu=px*ux+py*uy+pz*uz,pv=px*vx+py*vy+pz*vz,den=uu*vv-uv*uv,wb=(vv*pu-uv*pv)/den,wc=(uu*pv-uv*pu)/den,wa=1-wb-wc;
    for(let k=0;k<3;k++){this.pointA[k]=wa*w[ia+3+k]+wb*w[ib+3+k]+wc*w[ic+3+k];this.pointB[k]=wa*w[ia+6+k]+wb*w[ib+6+k]+wc*w[ic+6+k];}return true;
   }
   const next=this.nv++;for(let k=0;k<9;k++)this.vertices[next*9+k]=this.candidate[k];this.ne=0;
   for(let f=0;f<this.nf;f++)if(this.active[f]&&this.planes[f*4]*this.candidate[0]+this.planes[f*4+1]*this.candidate[1]+this.planes[f*4+2]*this.candidate[2]-this.planes[f*4+3]>1e-10){this.active[f]=0;this.edge(this.faces[f*3],this.faces[f*3+1]);this.edge(this.faces[f*3+1],this.faces[f*3+2]);this.edge(this.faces[f*3+2],this.faces[f*3]);}
   for(let e=0;e<this.ne;e++)this.face(this.edges[e*2],this.edges[e*2+1],next);
  }return false;
 }
}
