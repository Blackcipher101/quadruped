import {TRAINED_PARAMETERS} from './trained-policy';
import {type Sensors} from '../robot/sensors';
import {type JointController} from './joint';
import {Dynamics} from '../physics/dynamics';
import {parseURDF} from '../robot/urdf';
import {PointKinematics} from '../physics/kinematics';
import {vec,rotation,expQuaternion} from '../physics/spatial';
export type Gait='stand'|'trot'|'crawl'|'pace'|'bound';
export interface GaitParameters {period:number;duty:number;height:number;clearance:number;strideGain:number;balance:number;velocityGain:number;feedforward:number;kp:number;kd:number;footOffset:number;}
export const DEFAULT_GAIT:GaitParameters={...TRAINED_PARAMETERS};
/** Sensor-only CPG/IK policy. IK produces actuator targets, never simulation poses.
 * A proprioceptive complementary observer integrates gyro and stance-foot odometry.
 * This is a trainable parameter policy, not a whole-body MPC or an EKF.
 */
export class Locomotion {
 gait:Gait='stand';readonly command=vec(3);readonly velocity=vec(3);readonly orientation=Float64Array.of(0,0,0,1);readonly footTargets=vec(12);readonly schedule=new Uint8Array(4);readonly parameters:GaitParameters={...DEFAULT_GAIT};readonly kinematicModel:Dynamics;
 private point:PointKinematics;private local=Float64Array.of(0,0,-.22);private R=vec(9);private previous=vec(12);private measuredVelocity=vec(3);private initialized=false;private smoothed=vec(3);private phase=0;
 constructor(urdf:string){this.kinematicModel=new Dynamics(parseURDF(urdf));this.point=new PointKinematics(this.kinematicModel);}
 reset(){this.orientation.set([0,0,0,1]);this.velocity.fill(0);this.smoothed.fill(0);this.previous.fill(0);this.initialized=false;this.phase=0;}
 private observe(s:Sensors,dt:number){expQuaternion(this.orientation,s.gyro[0],s.gyro[1],s.gyro[2],dt);rotation(this.orientation,this.R);let contacts=0;for(let f=0;f<4;f++)contacts+=s.contact[f];const length=Math.hypot(...s.accelerometer);if(contacts>=3&&Math.abs(length-9.81)<1.5){const ax=s.accelerometer[0]/length,ay=s.accelerometer[1]/length,az=s.accelerometer[2]/length,gx=this.R[6],gy=this.R[7],gz=this.R[8];expQuaternion(this.orientation,ay*gz-az*gy,az*gx-ax*gz,ax*gy-ay*gx,dt*.8);rotation(this.orientation,this.R);}
 const model=this.kinematicModel;model.orientation.set(this.orientation);model.position.fill(0);for(let j=0;j<3;j++)model.v[j]=s.gyro[j];for(let j=0;j<12;j++){model.q[j+6]=s.q[j];model.v[j+6]=s.qd[j];}model.kinematics();this.measuredVelocity.fill(0);if(contacts){for(let f=0;f<4;f++)if(s.contact[f]){this.point.compute(3+f*3,this.local);for(let k=0;k<3;k++)this.measuredVelocity[k]-=this.point.velocity[k]/contacts;}for(let k=0;k<3;k++)this.velocity[k]+=.04*(this.measuredVelocity[k]-this.velocity[k]);}}
 update(s:Sensors,joints:JointController,dt=.001){this.observe(s,dt);if(this.gait==='stand'){joints.feedforward.fill(0);joints.velocityTarget.fill(0);for(let f=0;f<4;f++){joints.target[f*3]=0;joints.target[f*3+1]=.7;joints.target[f*3+2]=-1.4;}this.initialized=false;return;}
 const p=this.parameters;this.phase=(this.phase+dt/p.period)%1;for(let k=0;k<3;k++)this.smoothed[k]+=.004*(this.command[k]-this.smoothed[k]);joints.kp=p.kp;joints.kd=p.kd;const duty=this.gait==='crawl'?Math.max(.8,p.duty):p.duty;let stance=0;
 for(let f=0;f<4;f++){const offset=this.gait==='trot'?(f===0||f===3?0:.5):this.gait==='pace'?(f%2===0?0:.5):this.gait==='bound'?(f<2?0:.5):f===0?0:f===1?.5:f===2?.75:.25;this.schedule[f]=((this.phase+offset)%1)<duty?1:0;stance+=this.schedule[f];}
 for(let f=0;f<4;f++){const side=f%2===0?1:-1,hx=f<2?.2:-.2,hy=side*.1,offset=this.gait==='trot'?(f===0||f===3?0:.5):this.gait==='pace'?(f%2===0?0:.5):this.gait==='bound'?(f<2?0:.5):f===0?0:f===1?.5:f===2?.75:.25,phase=(this.phase+offset)%1,swing=phase>=duty,u=swing?(phase-duty)/(1-duty):phase/duty,blend=swing?u*u*(3-2*u):u;
  const vx=this.smoothed[0]-this.smoothed[2]*(hy+side*.06),vy=this.smoothed[1]+this.smoothed[2]*hx,strideX=vx*p.period*duty*p.strideGain,strideY=vy*p.period*duty*p.strideGain,cycle=swing?blend-.5:.5-blend;
  const measuredX=this.R[0]*this.velocity[0]+this.R[3]*this.velocity[1],measuredY=this.R[1]*this.velocity[0]+this.R[4]*this.velocity[1];let x=p.footOffset+strideX*cycle+p.velocityGain*(measuredX-this.smoothed[0]),y=side*.06+strideY*cycle+p.velocityGain*(measuredY-this.smoothed[1]),z=-p.height+(swing?p.clearance*16*u*u*(1-u)*(1-u):0);
  // Keep desired feet level using estimated roll/pitch; no ground truth pose is read.
  // Body-frame gravity direction removes yaw from tilt feedback.
  z+=p.balance*(this.R[6]*(hx+x)+this.R[7]*(hy+y));
  this.footTargets[f*3]=x;this.footTargets[f*3+1]=y;this.footTargets[f*3+2]=z;
  const vertical=Math.sqrt(Math.max(.02**2,y*y+z*z-.06**2)),radius=Math.hypot(x,vertical),bend=Math.acos(Math.max(-.999,Math.min(.999,radius/.44))),abduction=Math.atan2(y,-z)-Math.atan2(side*.06,vertical),hip=Math.atan2(-x,vertical)+bend,knee=-2*bend;
  joints.target[f*3]=abduction;joints.target[f*3+1]=hip;joints.target[f*3+2]=knee;this.point.compute(3+f*3,this.local);
  for(let j=0;j<3;j++){const index=f*3+j;const derivative=this.initialized?(joints.target[index]-this.previous[index])/dt:0;joints.velocityTarget[index]+=.15*(derivative-joints.velocityTarget[index]);this.previous[index]=joints.target[index];joints.feedforward[index]=swing?0:-p.feedforward*this.point.J[2*18+6+index]*13.9*9.81/Math.max(1,stance);}
 }this.initialized=true;
 }
}
