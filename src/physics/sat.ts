import {type Hull} from './shapes';
import {vec} from './spatial';
/** Separating-axis fallback for degenerate/touching GJK/EPA simplices.
 * Tests face normals and edge cross-products; never invents a centre-to-centre normal.
 */
export class SAT {
 readonly normal=vec(3);gap=-Infinity;private ax=0;private ay=0;private az=0;
 private axis(A:Hull,B:Hull,x:number,y:number,z:number){const length=Math.hypot(x,y,z);if(length<1e-8)return;x/=length;y/=length;z/=length;let amin=Infinity,amax=-Infinity,bmin=Infinity,bmax=-Infinity;for(let i=0;i<A.world.length;i+=3){const d=x*A.world[i]+y*A.world[i+1]+z*A.world[i+2];amin=Math.min(amin,d);amax=Math.max(amax,d);}for(let i=0;i<B.world.length;i+=3){const d=x*B.world[i]+y*B.world[i+1]+z*B.world[i+2];bmin=Math.min(bmin,d);bmax=Math.max(bmax,d);}let gap=amin-bmax;if(bmin-amax>gap){gap=bmin-amax;x=-x;y=-y;z=-z;}if(gap>this.gap){this.gap=gap;this.normal[0]=x;this.normal[1]=y;this.normal[2]=z;}}
 query(A:Hull,B:Hull){this.gap=-Infinity;for(let side=0;side<2;side++){const shape=side===0?A:B,R=shape.rotation;for(let f=0;f<shape.planes.length;f+=4){const p=shape.planes;this.axis(A,B,R[0]*p[f]+R[1]*p[f+1]+R[2]*p[f+2],R[3]*p[f]+R[4]*p[f+1]+R[5]*p[f+2],R[6]*p[f]+R[7]*p[f+1]+R[8]*p[f+2]);if(this.gap>.003)return false;}}
 for(let i=0;i<A.edges.length;i+=3){const ar=A.rotation,ae=A.edges,ax=ar[0]*ae[i]+ar[1]*ae[i+1]+ar[2]*ae[i+2],ay=ar[3]*ae[i]+ar[4]*ae[i+1]+ar[5]*ae[i+2],az=ar[6]*ae[i]+ar[7]*ae[i+1]+ar[8]*ae[i+2];for(let j=0;j<B.edges.length;j+=3){const br=B.rotation,be=B.edges,bx=br[0]*be[j]+br[1]*be[j+1]+br[2]*be[j+2],by=br[3]*be[j]+br[4]*be[j+1]+br[5]*be[j+2],bz=br[6]*be[j]+br[7]*be[j+1]+br[8]*be[j+2];this.axis(A,B,ay*bz-az*by,az*bx-ax*bz,ax*by-ay*bx);if(this.gap>.003)return false;}}
 return true;}
}
