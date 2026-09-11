import {Sensors} from '../robot/sensors';
import {Actuator} from '../robot/actuator';
import {vec} from '../physics/spatial';
/** No Dynamics reference: controller accepts only measured sensor output. */
export class JointController {
 readonly target=vec(12);readonly velocityTarget=vec(12);readonly feedforward=vec(12);readonly torque=vec(12);readonly derated=vec(12);readonly actuators=Array.from({length:12},()=>new Actuator());kp=80;kd=2.5;
 constructor(){for(let f=0;f<4;f++){this.target[f*3]=0;this.target[f*3+1]=.7;this.target[f*3+2]=-1.4;}}
 update(s:Sensors){for(let i=0;i<12;i++){this.torque[i]=this.actuators[i].command(this.kp*(this.target[i]-s.q[i])+this.kd*(this.velocityTarget[i]-s.qd[i])+this.feedforward[i],s.qd[i]);this.derated[i]=this.actuators[i].deratedTorque;}}
}
