import {Impulse} from '../physics/impulse';
import {Dynamics} from '../physics/dynamics';
import {World} from '../physics/world';
import {hull,rockMesh,boxVertices,BOX_TRIANGLES,rigidHull,type Hull} from '../physics/shapes';
import {type ContactSite} from '../physics/ground';
import {expQuaternion} from '../physics/spatial';
export const ROCK_COUNT=40;
export function gravelShape(i:number){return rockMesh(i%8,.031+(i%4)*.005);}
export class Gravel {
 private kick:Impulse|undefined;private kickPoint=Float64Array.of(.025,0,.02);private kickForce=Float64Array.of(.08,.035,.18);readonly rocks:Dynamics[]=[];readonly world:World;readonly hulls:Hull[]=[];
 constructor(robot:Dynamics,sites:ContactSite[]){
  for(let i=0;i<13;i++){if(i>0&&(i-1)%3===0)continue;const vertices=i===0?boxVertices(.22,.10,.06):boxVertices(.018,.019,.11);if(i>0)for(let v=2;v<vertices.length;v+=3)vertices[v]-=.11;const h=hull(robot,0,i,vertices,BOX_TRIANGLES,.5);h.terrainContact=false;this.hulls.push(h);}
  for(let i=0;i<ROCK_COUNT;i++){const shape=gravelShape(i),d=rigidHull(shape.vertices,shape.triangles,2600);this.rocks.push(d);const h=hull(d,i+1,0,shape.vertices,shape.triangles,.72);h.rolling=.0015;h.spinning=.0008;h.restitution=.04;this.hulls.push(h);}this.world=new World([robot,...this.rocks],this.hulls,sites);this.reset();
 }
 tumble(){if(!this.kick)this.kick=new Impulse(this.rocks[3]);this.kick.apply(0,this.kickPoint,this.kickForce);}
 reset(seed=42){for(let i=0;i<this.rocks.length;i++){const d=this.rocks[i],row=Math.floor(i/10),col=i%10;d.position.set([.38+col*.14+.015*Math.sin(seed+i*11),-.25+row*.165+.012*Math.cos(seed+i*7),.09+.006*(i%3)]);d.v.fill(0);d.tau.fill(0);d.qdd.fill(0);d.orientation.set([0,0,0,1]);expQuaternion(d.orientation,.24*Math.sin(i+seed),.27*Math.cos(i*5),i*.731,1);d.kinematics();}this.world.reset();}
}
