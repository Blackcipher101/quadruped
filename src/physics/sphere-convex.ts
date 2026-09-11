import {type Hull} from './shapes';
import {vec,type V} from './spatial';
/** Closest triangle point in local coordinates, with signed inside/outside distance. */
export class SphereConvex {
 readonly normal=vec(3);readonly point=vec(3);distance=Infinity;private p=vec(3);private closest=vec(3);
 query(shape:Hull,center:V){const R=shape.rotation,dx=center[0]-shape.position[0],dy=center[1]-shape.position[1],dz=center[2]-shape.position[2];for(let k=0;k<3;k++)this.p[k]=R[k]*dx+R[k+3]*dy+R[k+6]*dz;let inside=true;for(let f=0;f<shape.planes.length;f+=4)if(shape.planes[f]*this.p[0]+shape.planes[f+1]*this.p[1]+shape.planes[f+2]*this.p[2]>shape.planes[f+3]+1e-12)inside=false;let best=Infinity;const v=shape.vertices;
 for(let f=0;f<shape.triangles.length;f+=3){const a=shape.triangles[f]*3,b=shape.triangles[f+1]*3,c=shape.triangles[f+2]*3,abx=v[b]-v[a],aby=v[b+1]-v[a+1],abz=v[b+2]-v[a+2],acx=v[c]-v[a],acy=v[c+1]-v[a+1],acz=v[c+2]-v[a+2],apx=this.p[0]-v[a],apy=this.p[1]-v[a+1],apz=this.p[2]-v[a+2],d1=abx*apx+aby*apy+abz*apz,d2=acx*apx+acy*apy+acz*apz,bpx=this.p[0]-v[b],bpy=this.p[1]-v[b+1],bpz=this.p[2]-v[b+2],d3=abx*bpx+aby*bpy+abz*bpz,d4=acx*bpx+acy*bpy+acz*bpz,cpx=this.p[0]-v[c],cpy=this.p[1]-v[c+1],cpz=this.p[2]-v[c+2],d5=abx*cpx+aby*cpy+abz*cpz,d6=acx*cpx+acy*cpy+acz*cpz,vc=d1*d4-d3*d2,vb=d5*d2-d1*d6,va=d3*d6-d5*d4;let u=0,w=0;
 if(d1<=0&&d2<=0){}else if(d3>=0&&d4<=d3)u=1;else if(vc<=0&&d1>=0&&d3<=0)u=d1/(d1-d3);else if(d6>=0&&d5<=d6)w=1;else if(vb<=0&&d2>=0&&d6<=0)w=d2/(d2-d6);else if(va<=0&&d4-d3>=0&&d5-d6>=0){w=(d4-d3)/(d4-d3+d5-d6);u=1-w;}else{const den=1/(va+vb+vc);u=vb*den;w=vc*den;}
 const x=v[a]+u*abx+w*acx,y=v[a+1]+u*aby+w*acy,z=v[a+2]+u*abz+w*acz,dist=(x-this.p[0])**2+(y-this.p[1])**2+(z-this.p[2])**2;if(dist<best){best=dist;this.closest[0]=x;this.closest[1]=y;this.closest[2]=z;}}
 const length=Math.sqrt(best);this.distance=inside?-length:length;for(let k=0;k<3;k++){this.point[k]=shape.position[k]+R[k*3]*this.closest[0]+R[k*3+1]*this.closest[1]+R[k*3+2]*this.closest[2];this.normal[k]=(center[k]-this.point[k])/(length||1)*(inside?-1:1);}return this.distance;
 }
}
