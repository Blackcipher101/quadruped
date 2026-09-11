/** SI units. Row-major matrices; spatial motion [angular; linear], force [moment; force]. */
export type V = Float64Array;
export const vec = (n: number) => new Float64Array(n);
export function identity(a: V, n: number) { a.fill(0); for (let i=0;i<n;i++) a[i*n+i]=1; }
export function mv(a: V, x: V, out: V, n=6, transpose=false) {
  for(let i=0;i<n;i++){let s=0;for(let j=0;j<n;j++)s+=a[transpose?j*n+i:i*n+j]*x[j];out[i]=s;}
}
export function mm(a: V,b: V,out: V,n=6,ta=false,tb=false) {
  for(let i=0;i<n;i++)for(let j=0;j<n;j++){let s=0;for(let k=0;k<n;k++)s+=a[ta?k*n+i:i*n+k]*b[tb?j*n+k:k*n+j];out[i*n+j]=s;}
}
export function dot(a:V,b:V){let s=0;for(let i=0;i<a.length;i++)s+=a[i]*b[i];return s;}
export function cross(a:V,b:V,o:V){const x=a[1]*b[2]-a[2]*b[1],y=a[2]*b[0]-a[0]*b[2],z=a[0]*b[1]-a[1]*b[0];o[0]=x;o[1]=y;o[2]=z;}
export function skew(x:number,y:number,z:number,o:V){o[0]=0;o[1]=-z;o[2]=y;o[3]=z;o[4]=0;o[5]=-x;o[6]=-y;o[7]=x;o[8]=0;}
export function crm(v:V,m:V,o:V){
  for(let i=0;i<3;i++){const j=(i+1)%3,k=(i+2)%3;o[i]=v[j]*m[k]-v[k]*m[j];o[i+3]=v[j]*m[k+3]-v[k]*m[j+3]+v[j+3]*m[k]-v[k+3]*m[j];}
}
export function crf(v:V,f:V,o:V){
  for(let i=0;i<3;i++){const j=(i+1)%3,k=(i+2)%3;o[i]=v[j]*f[k]-v[k]*f[j]+v[j+3]*f[k+3]-v[k+3]*f[j+3];o[i+3]=v[j]*f[k+3]-v[k]*f[j+3];}
}
/** E maps parent coordinates to child; r is child origin in parent coordinates. */
export function transform(E:V,r:V,X:V){
  X.fill(0);
  for(let i=0;i<3;i++)for(let j=0;j<3;j++){
    X[i*6+j]=X[(i+3)*6+j+3]=E[i*3+j];
    const k=(j+1)%3,l=(j+2)%3;X[(i+3)*6+j]=-E[i*3+k]*r[l]+E[i*3+l]*r[k];
  }
}
/** Active rotation, body to parent, quaternion [x,y,z,w]. */
export function rotation(q:V,R:V){const x=q[0],y=q[1],z=q[2],w=q[3];R[0]=1-2*(y*y+z*z);R[1]=2*(x*y-z*w);R[2]=2*(x*z+y*w);R[3]=2*(x*y+z*w);R[4]=1-2*(x*x+z*z);R[5]=2*(y*z-x*w);R[6]=2*(x*z-y*w);R[7]=2*(y*z+x*w);R[8]=1-2*(x*x+y*y);}
export function expQuaternion(q:V,wx:number,wy:number,wz:number,dt:number){
  const h=Math.hypot(wx,wy,wz)*dt/2,s=h<1e-8?dt/2*(1-h*h/6):Math.sin(h)*dt/(2*h);
  const x=wx*s,y=wy*s,z=wz*s,w=Math.cos(h),a=q[0],b=q[1],c=q[2],d=q[3];
  q[0]=d*x+a*w+b*z-c*y;q[1]=d*y-a*z+b*w+c*x;q[2]=d*z+a*y-b*x+c*w;q[3]=d*w-a*x-b*y-c*z;
  const norm=Math.hypot(q[0],q[1],q[2],q[3]);for(let i=0;i<4;i++)q[i]/=norm;
}
export function axisRotation(axis:V,angle:number,E:V){const c=Math.cos(angle),s=-Math.sin(angle),t=1-c,x=axis[0],y=axis[1],z=axis[2];E[0]=c+x*x*t;E[1]=x*y*t-z*s;E[2]=x*z*t+y*s;E[3]=y*x*t+z*s;E[4]=c+y*y*t;E[5]=y*z*t-x*s;E[6]=z*x*t-y*s;E[7]=z*y*t+x*s;E[8]=c+z*z*t;}
export function inertia(m:number,c:V,Ic:V,out:V){out.fill(0);const c2=dot(c,c);for(let i=0;i<3;i++)for(let j=0;j<3;j++){out[i*6+j]=Ic[i*3+j]+m*((i===j?c2:0)-c[i]*c[j]);const k=(i+1)%3,l=(i+2)%3;const cij=j===k?-c[l]:j===l?c[k]:0;out[i*6+j+3]=m*cij;out[(j+3)*6+i]=m*cij;out[(i+3)*6+j+3]=i===j?m:0;}}
/** Cholesky. Throws on invalid inertia rather than masking a singular model. */
export function factor(a:V,L:V,n:number){L.fill(0);for(let i=0;i<n;i++)for(let j=0;j<=i;j++){let s=a[i*n+j];for(let k=0;k<j;k++)s-=L[i*n+k]*L[j*n+k];if(i===j){if(!(s>0))throw Error('Non-positive mass matrix');L[i*n+j]=Math.sqrt(s);}else L[i*n+j]=s/L[j*n+j];}}
export function solve(L:V,b:V,x:V,n:number){for(let i=0;i<n;i++){let s=b[i];for(let j=0;j<i;j++)s-=L[i*n+j]*x[j];x[i]=s/L[i*n+i];}for(let i=n-1;i>=0;i--){let s=x[i];for(let j=i+1;j<n;j++)s-=L[j*n+i]*x[j];x[i]=s/L[i*n+i];}}
export function matrixQuaternion(R:V,q:V){const trace=R[0]+R[4]+R[8];if(trace>0){const s=Math.sqrt(trace+1)*2;q[3]=.25*s;q[0]=(R[7]-R[5])/s;q[1]=(R[2]-R[6])/s;q[2]=(R[3]-R[1])/s;}else if(R[0]>R[4]&&R[0]>R[8]){const s=Math.sqrt(1+R[0]-R[4]-R[8])*2;q[3]=(R[7]-R[5])/s;q[0]=.25*s;q[1]=(R[1]+R[3])/s;q[2]=(R[2]+R[6])/s;}else if(R[4]>R[8]){const s=Math.sqrt(1+R[4]-R[0]-R[8])*2;q[3]=(R[2]-R[6])/s;q[0]=(R[1]+R[3])/s;q[1]=.25*s;q[2]=(R[5]+R[7])/s;}else{const s=Math.sqrt(1+R[8]-R[0]-R[4])*2;q[3]=(R[3]-R[1])/s;q[0]=(R[2]+R[6])/s;q[1]=(R[5]+R[7])/s;q[2]=.25*s;}}
