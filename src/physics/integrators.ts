import { Dynamics } from './dynamics';
import { vec } from './spatial';
/** Reference integrator for pinned smooth systems only, never contact dynamics. */
export class RK4 {
 private q;private v;private kq;private kv;
 constructor(readonly d:Dynamics){if(d.model.floating)throw Error('RK4 reference supports pinned systems');const n=d.model.nv;this.q=vec(n);this.v=vec(n);this.kq=[vec(n),vec(n),vec(n),vec(n)];this.kv=[vec(n),vec(n),vec(n),vec(n)];}
 step(h:number){const d=this.d,n=d.model.nv;this.q.set(d.q);this.v.set(d.v);for(let s=0;s<4;s++){const f=s===3?1:.5;if(s>0)for(let i=0;i<n;i++){d.q[i]=this.q[i]+h*f*this.kq[s-1][i];d.v[i]=this.v[i]+h*f*this.kv[s-1][i];}d.aba();this.kq[s].set(d.v);this.kv[s].set(d.qdd);}for(let i=0;i<n;i++){d.q[i]=this.q[i]+h*(this.kq[0][i]+2*this.kq[1][i]+2*this.kq[2][i]+this.kq[3][i])/6;d.v[i]=this.v[i]+h*(this.kv[0][i]+2*this.kv[1][i]+2*this.kv[2][i]+this.kv[3][i])/6;}}
}
