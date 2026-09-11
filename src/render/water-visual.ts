import * as T from 'three';
import {MarchingCubes} from 'three/addons/objects/MarchingCubes.js';
/** Isosurface reconstructed exclusively from simulated particle positions. */
export class WaterVisual {
 readonly group=new T.Group();private surface:MarchingCubes;private lastStep=-1;
 constructor(scene:T.Scene){const material=new T.MeshPhysicalMaterial({color:0x287e91,metalness:0,roughness:.12,transmission:.55,thickness:.12,ior:1.333,clearcoat:1,clearcoatRoughness:.08,attenuationColor:new T.Color(0x298294),attenuationDistance:.5,side:T.DoubleSide});this.surface=new MarchingCubes(64,material,false,false,60000);this.surface.position.set(.84,0,.7);this.surface.scale.setScalar(.84);this.surface.enableUvs=false;this.surface.frustumCulled=false;this.group.add(this.surface);
 // A wire boundary identifies the fluid computational domain; it is not a rigid tank.
 const boundary=new T.LineSegments(new T.EdgesGeometry(new T.BoxGeometry(1.12,.64,.18)),new T.LineBasicMaterial({color:0x70bac3,transparent:true,opacity:.35}));boundary.position.set(.84,0,.09);this.group.add(boundary);scene.add(this.group);this.group.visible=false;}
 update(data:Float64Array,previous:number,current:number,alpha:number,count:number){this.group.visible=count>0;if(!count||data[0]===this.lastStep)return;this.lastStep=data[0];this.surface.reset();for(let i=0;i<count;i++){const a=previous+3*i,b=current+3*i,x=data[a]+alpha*(data[b]-data[a]),y=data[a+1]+alpha*(data[b+1]-data[a+1]),z=data[a+2]+alpha*(data[b+2]-data[a+2]);this.surface.addBall((x-.84)/1.68+.5,y/1.68+.5,(z-.7)/1.68+.5,.034,12);}this.surface.update();}
 dispose(){this.group.traverse(o=>{if(o instanceof T.Mesh||o instanceof T.LineSegments){o.geometry.dispose();const m=o.material as T.Material;m.dispose();}});this.group.removeFromParent();}
}
