import {Dynamics} from '../physics/dynamics';
import {GroundSystem} from '../physics/ground';
import {vec,rotation,mv} from '../physics/spatial';
export class SeededNoise {private state:number;constructor(seed=42){this.state=seed>>>0||1;}uniform(){let x=this.state;x^=x<<13;x^=x>>>17;x^=x<<5;this.state=x>>>0;return (this.state+.5)/4294967296;}gaussian(){return Math.sqrt(-2*Math.log(this.uniform()))*Math.cos(2*Math.PI*this.uniform());}}
/** The only simulation-side adapter allowed to access truth for the controller. */
export class Sensors {
 readonly q=vec(12);readonly qd=vec(12);readonly gyro=vec(3);readonly accelerometer=vec(3);readonly footForce=vec(4);readonly contact=new Uint8Array(4);time=0;
 private previous=vec(12);private gyroBias=vec(3);private accelBias=vec(3);private R=vec(9);private g=vec(3);private rng:SeededNoise;private initialized=false;
 noiseScale=1;
 constructor(seed=42){this.rng=new SeededNoise(seed);}
 reset(seed=42){this.rng=new SeededNoise(seed);this.q.fill(0);this.qd.fill(0);this.previous.fill(0);this.gyroBias.fill(0);this.accelBias.fill(0);this.gyro.fill(0);this.accelerometer.fill(0);this.footForce.fill(0);this.contact.fill(0);this.initialized=false;this.time=0;}
 sample(d:Dynamics,ground:GroundSystem,dt=.001){const quantum=2*Math.PI/16384;for(let j=0;j<12;j++){this.q[j]=Math.round(d.q[j+6]/quantum)*quantum;const derivative=this.initialized?(this.q[j]-this.previous[j])/dt:0;this.qd[j]+=.2*(derivative-this.qd[j]);this.previous[j]=this.q[j];}
 rotation(d.orientation,this.R);mv(this.R,d.gravity,this.g,3,true);
 for(let k=0;k<3;k++){const j=(k+1)%3,l=(k+2)%3;this.gyroBias[k]+=this.noiseScale*.00002*Math.sqrt(dt)*this.rng.gaussian();this.accelBias[k]+=this.noiseScale*.0002*Math.sqrt(dt)*this.rng.gaussian();this.gyro[k]=d.v[k]+this.gyroBias[k]+this.noiseScale*.001*this.rng.gaussian();this.accelerometer[k]=d.qdd[k+3]+d.v[j]*d.v[l+3]-d.v[l]*d.v[j+3]-this.g[k]+this.accelBias[k]+this.noiseScale*.015*this.rng.gaussian();}
 for(let f=0;f<4;f++){this.footForce[f]=ground.forces[f*3];this.contact[f]=this.footForce[f]>2?1:0;}this.initialized=true;this.time+=dt;
 }
}
