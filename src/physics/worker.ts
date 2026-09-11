import urdf from '../robot/q14.urdf?raw';
import {RobotSimulation} from '../robot/simulation';
import {Exchange,POSE_PREVIOUS,POSE_CURRENT,TELEMETRY,WORLD_META,ROCK_PREVIOUS,ROCK_CURRENT,FLUID_POSITIONS,FLUID_PREVIOUS} from './exchange';
import {matrixQuaternion,vec} from './spatial';
import {Impulse} from './impulse';
import {type Gait} from '../control/locomotion';
import {type SceneName} from './terrain';
const simulation=new RobotSimulation(urdf),impulse=new Impulse(simulation.dynamics),quaternion=vec(4),previous=vec(91),current=vec(91),point=vec(3),force=vec(3);
const previousFluid=vec(3360);
const previousRocks=vec(280),currentRocks=vec(280);
function rockPoses(){if(!simulation.gravel)return;for(let i=0;i<40;i++){const d=simulation.gravel.rocks[i];for(let k=0;k<3;k++)currentRocks[i*7+k]=d.position[k];for(let k=0;k<4;k++)currentRocks[i*7+3+k]=d.orientation[k];}}
let exchange:Exchange|undefined,paused=false,speed=1,last=performance.now(),accumulator=0,stepTime=0,scene:SceneName='flat';
function poses(out:Float64Array){simulation.dynamics.kinematics();for(let b=0;b<13;b++){const body=simulation.dynamics.bodies[b];matrixQuaternion(body.R,quaternion);for(let k=0;k<3;k++)out[b*7+k]=body.p[k];for(let k=0;k<4;k++)out[b*7+3+k]=quaternion[k];}}
poses(current);previous.set(current);
function tick(){if(scene==='water'&&simulation.water)previousFluid.set(simulation.water.fluid.position);previous.set(current);previousRocks.set(currentRocks);const start=performance.now();simulation.step();stepTime=.95*stepTime+.05*(performance.now()-start);poses(current);rockPoses();}
function publish(){if(!exchange)return;const data=exchange.data;data.fill(0);data[0]=simulation.steps;data[1]=accumulator/.001;data[2]=performance.timeOrigin+performance.now();data[3]=stepTime;data[4]=simulation.ground.constraints.iterations;data[5]=simulation.ground.constraints.residual;data[6]=simulation.ground.maxPenetration;data[7]=paused?1:0;data.set(previous,POSE_PREVIOUS);data.set(current,POSE_CURRENT);data[WORLD_META]=(scene==='gravel'||scene==='rocks')?40:0;data.set(previousRocks,ROCK_PREVIOUS);data.set(currentRocks,ROCK_CURRENT);data[WORLD_META+1]=simulation.locomotion.gait==='stand'?0:1;data[WORLD_META+2]=simulation.locomotion.command[0];if(scene==='water'&&simulation.water){data[WORLD_META+3]=simulation.water.fluid.count;data.set(simulation.water.fluid.position,FLUID_POSITIONS);data.set(previousFluid,FLUID_PREVIOUS);}
 for(let i=0;i<12;i++){data[TELEMETRY+i]=simulation.controller.torque[i];data[TELEMETRY+12+i]=simulation.controller.derated[i];data[TELEMETRY+24+i]=simulation.sensors.q[i];}
 for(let i=0;i<4;i++){data[TELEMETRY+36+i]=simulation.sensors.footForce[i];data[TELEMETRY+40+i]=simulation.sensors.contact[i];}
 for(let k=0;k<6;k++)data[TELEMETRY+44+k]=simulation.dynamics.v[k];exchange.publish();}
self.onmessage=(event:MessageEvent)=>{const m=event.data;switch(m.type){case 'init':exchange=new Exchange(m.buffer,true);scene=m.scene??'flat';simulation.reset(scene);poses(current);previous.set(current);rockPoses();previousRocks.set(currentRocks);last=performance.now();publish();break;case 'tumble':if(scene==='gravel'||scene==='rocks')simulation.gravel?.tumble();publish();break;case 'pause':paused=m.value;accumulator=0;break;case 'step':if(paused)tick();publish();break;case 'reset':scene=m.scene??scene;simulation.reset(scene);if(simulation.water)previousFluid.set(simulation.water.fluid.position);poses(current);previous.set(current);rockPoses();previousRocks.set(currentRocks);accumulator=0;stepTime=0;publish();break;case 'gait':simulation.locomotion.gait=m.value as Gait;if(m.value==='stand'){simulation.locomotion.update(simulation.sensors,simulation.controller);simulation.controller.kp=80;simulation.controller.kd=2.5;}break;case 'command':simulation.locomotion.command.set(m.value);break;case 'speed':speed=m.value;break;case 'kp':simulation.controller.kp=m.value;break;case 'kd':simulation.controller.kd=m.value;break;case 'mu':for(const site of simulation.ground.sites)site.mu=m.value;break;case 'roughness':simulation.ground.terrain.roughness=m.value;break;case 'push':point.set(m.point??[0,0,0]);force.set(m.impulse??[0,2,0]);impulse.apply(m.body??0,point,force);break;}}
function pump(){try{const now=performance.now();if(!paused&&exchange)accumulator+=(now-last)*.001*speed;last=now;
 // Retain backlog; never drop elapsed physics time or scale gravity on slow frames.
 let ticks=0;while(accumulator>=.001&&ticks<32){tick();accumulator-=.001;ticks++;}publish();setTimeout(pump,2);
 }catch(error){paused=true;self.postMessage({type:'error',message:error instanceof Error?error.message:String(error)});}}
pump();
