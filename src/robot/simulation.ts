import {GRAVEL_PARAMETERS} from '../control/gravel-policy';
import {DEFAULT_GAIT} from '../control/locomotion';
import {Locomotion} from '../control/locomotion';
import {Water} from './water';
import {Gravel} from './gravel';
import {parseURDF} from './urdf';
import {Dynamics} from '../physics/dynamics';
import {GroundSystem,type ContactSite} from '../physics/ground';
import {Sensors} from './sensors';
import {JointController} from '../control/joint';
import {type SceneName} from '../physics/terrain';
export class RobotSimulation {
 readonly locomotion:Locomotion;readonly dynamics:Dynamics;readonly ground:GroundSystem;readonly sensors=new Sensors(42);readonly controller=new JointController();steps=0;substeps=2;gravel:Gravel|undefined;water:Water|undefined;scene:SceneName='flat';
 constructor(urdf:string){this.locomotion=new Locomotion(urdf);this.dynamics=new Dynamics(parseURDF(urdf));const sites:ContactSite[]=[];for(let f=0;f<4;f++)sites.push({body:f*3+3,point:Float64Array.of(0,0,-.22),radius:.022,mu:.9,restitution:.05});
 // Body and link collision samples make falls physical; these are not full convex collision.
 for(let x=-1;x<=1;x+=2)for(let y=-1;y<=1;y+=2)for(let z=-1;z<=1;z+=2)sites.push({body:0,point:Float64Array.of(x*.22,y*.1,z*.06),radius:0,mu:.5,restitution:.05});
 for(let f=0;f<4;f++)for(let link=1;link<=3;link++)sites.push({body:f*3+link,point:Float64Array.of(0,0,link===1?0:-.11),radius:.025,mu:.5,restitution:.05});this.ground=new GroundSystem(this.dynamics,sites);this.reset();}
 reset(scene:SceneName='flat'){this.scene=scene;for(const wrench of this.dynamics.external)wrench.fill(0);Object.assign(this.locomotion.parameters,scene==='gravel'||scene==='rocks'?GRAVEL_PARAMETERS:DEFAULT_GAIT);this.locomotion.reset();this.controller.feedforward.fill(0);this.controller.velocityTarget.fill(0);for(let f=0;f<4;f++){this.controller.target[f*3]=0;this.controller.target[f*3+1]=.7;this.controller.target[f*3+2]=-1.4;}const d=this.dynamics;d.q.fill(0);d.v.fill(0);d.tau.fill(0);d.position.set([0,0,.41]);d.orientation.set([0,0,0,1]);for(let i=0;i<12;i++)d.q[i+6]=this.controller.target[i];d.qdd.fill(0);this.ground.terrain.scene=scene;this.ground.reset();this.sensors.reset();this.steps=0;if(scene==='water'){if(!this.water)this.water=new Water(d,this.ground.sites);this.water.reset();}if(scene==='gravel'||scene==='rocks'){if(!this.gravel)this.gravel=new Gravel(d,this.ground.sites);this.gravel.reset();this.gravel.world.terrain.scene=scene;}}
 step(){if(this.scene==='water')this.water?.step();this.sensors.sample(this.dynamics,this.ground);if(this.locomotion.gait!=='stand')this.locomotion.update(this.sensors,this.controller);this.controller.update(this.sensors);for(let i=0;i<12;i++)this.dynamics.tau[i+6]=this.controller.torque[i];if((this.scene==='gravel'||this.scene==='rocks')&&this.gravel){const w=this.gravel.world;w.terrain.roughness=this.ground.terrain.roughness;w.step(.001,this.substeps);this.ground.forces.set(w.forceBySite);this.ground.maxPenetration=w.maxPenetration;this.ground.constraints.iterations=w.iterations;this.ground.constraints.residual=w.residual;}else this.ground.step(.001,this.substeps);this.steps++;}
}
