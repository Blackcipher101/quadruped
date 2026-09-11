import * as T from 'three';
import {RoomEnvironment} from 'three/addons/environments/RoomEnvironment.js';
import {RectAreaLightUniformsLib} from 'three/addons/lights/RectAreaLightUniformsLib.js';
import {CSM} from 'three/addons/csm/CSM.js';
import {Terrain,type SceneName} from '../physics/terrain';
import {surfaceTexture,labelTexture} from './materials';
export type Lighting='studio'|'daylight'|'sunset';
export class Environment {
 readonly terrain=new Terrain();readonly ground:T.Mesh<T.PlaneGeometry,T.MeshStandardMaterial>;readonly csm:CSM;
 private gridEnabled=true;private hemisphere:T.HemisphereLight;private rim:T.RectAreaLight;private fill:T.RectAreaLight;private environmentTarget:T.WebGLRenderTarget;private labels:T.Mesh[]=[];private pmrem:T.PMREMGenerator;private texture:T.Texture;
 readonly uniforms={gridStrength:{value:.34},markings:{value:1},accent:{value:new T.Color(0xb78754)}};
 constructor(readonly scene:T.Scene,renderer:T.WebGLRenderer,camera:T.PerspectiveCamera){
  RectAreaLightUniformsLib.init();this.pmrem=new T.PMREMGenerator(renderer);const room=new RoomEnvironment();this.environmentTarget=this.pmrem.fromScene(room,.035);room.dispose();scene.environment=this.environmentTarget.texture;scene.environmentIntensity=.55;
  this.hemisphere=new T.HemisphereLight(0xcad9e1,0x323739,.65);scene.add(this.hemisphere);this.csm=new CSM({maxFar:12,cascades:3,shadowMapSize:2048,lightDirection:new T.Vector3(-1.1,1.2,-2.4).normalize(),camera,parent:scene,lightIntensity:3.2,lightMargin:2,shadowBias:-.00002});this.csm.fade=true;for(const light of this.csm.lights){light.color.set(0xe1edff);light.shadow.radius=3;light.shadow.normalBias=.001;}
  this.rim=new T.RectAreaLight(0xf0a06c,5,2,1.5);this.rim.position.set(-1.5,1.5,1.8);this.rim.lookAt(0,0,.3);scene.add(this.rim);this.fill=new T.RectAreaLight(0xe9f0f7,5,2,2);this.fill.position.set(1.5,-1.5,2.4);this.fill.lookAt(0,0,.2);scene.add(this.fill);
  this.texture=surfaceTexture('concrete');this.texture.repeat.set(32,32);this.texture.colorSpace=T.NoColorSpace;
  const material=new T.MeshStandardMaterial({color:0x333b3c,roughness:.78,metalness:.18,bumpMap:this.texture,bumpScale:.0012});
  material.onBeforeCompile=shader=>{Object.assign(shader.uniforms,this.uniforms);shader.vertexShader='varying vec3 groundWorld;\n'+shader.vertexShader;shader.vertexShader=shader.vertexShader.replace('#include <worldpos_vertex>','#include <worldpos_vertex>\ngroundWorld=(modelMatrix*vec4(transformed,1.0)).xyz;');shader.fragmentShader='varying vec3 groundWorld;uniform float gridStrength;uniform float markings;uniform vec3 accent;\n'+shader.fragmentShader;shader.fragmentShader=shader.fragmentShader.replace('#include <color_fragment>',`#include <color_fragment>
 vec2 g=groundWorld.xy;vec2 d=max(fwidth(g),vec2(.00001));
 vec2 minor=abs(fract(g*4.-.5)-.5)/d/4.;float grid=1.-min(min(minor.x,minor.y),1.);
 float fade=1.-smoothstep(1.5,5.,length(g));diffuseColor.rgb=mix(diffuseColor.rgb,vec3(.027,.035,.04),grid*gridStrength*fade);
 vec2 edge=abs(abs(g)-vec2(1.3,1.05));float line=1.-smoothstep(.002,.004,min(edge.x,edge.y));line*=step(abs(g.x),1.304)*step(abs(g.y),1.054);
 float corners=step(.90,abs(g.x))*step(.70,abs(g.y));float dashed=step(.45,fract((g.x+g.y)*12.));
 diffuseColor.rgb=mix(diffuseColor.rgb,vec3(.075,.092,.1),line*markings*mix(dashed,1.,corners)*.7);
 float hatch=step(1.17,abs(g.x))*step(abs(g.x),1.265)*step(.92,abs(g.y))*step(abs(g.y),1.018)*step(.50,fract((g.x+g.y)*40.));
 diffuseColor.rgb=mix(diffuseColor.rgb,accent,hatch*markings*.65);
 `);};const decorate=material.onBeforeCompile;this.csm.setupMaterial(material);const cascade=material.onBeforeCompile;material.onBeforeCompile=(shader,renderer)=>{cascade.call(material,shader,renderer);decorate.call(material,shader,renderer);};this.ground=new T.Mesh(new T.PlaneGeometry(32,32,256,256),material);this.ground.receiveShadow=true;scene.add(this.ground);
  this.label('Q14','DYNAMICS LAB / TEST VOLUME',.56,-.68,-.87);this.label('01','LIVE EXPERIMENT',.18,1.08,.90);this.setLighting('studio');
 }
 private label(title:string,sub:string,width:number,x:number,y:number){const map=labelTexture(title,sub,'#aab6b6');map.colorSpace=T.SRGBColorSpace;const mesh=new T.Mesh(new T.PlaneGeometry(width,width/4),new T.MeshBasicMaterial({map,transparent:true,opacity:.3,depthWrite:false,polygonOffset:true,polygonOffsetFactor:-1}));mesh.position.set(x,y,.002);this.scene.add(mesh);this.labels.push(mesh);}
 setLighting(lighting:Lighting){const night=lighting==='studio',sunset=lighting==='sunset';this.scene.background=new T.Color(night?0x141b1e:sunset?0xb6a596:0xc4cacf);this.scene.fog=new T.FogExp2(night?0x141b1e:sunset?0xb6a596:0xc4cacf,.055);this.scene.environmentIntensity=night?.65:sunset?.65:.95;this.ground.material.color.set(night?0x111a20:sunset?0x857564:0x989f9e);this.hemisphere.intensity=night?.35:.95;this.hemisphere.color.set(night?0xc6d7e7:0xe6edf3);this.rim.intensity=night?3:2;this.rim.color.set(sunset?0xffa467:0xc9d6df);this.fill.intensity=night?3:2;for(const light of this.csm.lights){light.intensity=night?2:sunset?4:3;light.color.set(sunset?0xffbe82:0xe9f2ff);}this.uniforms.gridStrength.value=this.gridEnabled?(night?.3:.22):0;}
 setTerrain(scene:SceneName,roughness=0){this.terrain.scene=scene;this.terrain.roughness=roughness;const pos=this.ground.geometry.getAttribute('position');for(let i=0;i<pos.count;i++)pos.setZ(i,this.terrain.height(pos.getX(i),pos.getY(i)));pos.needsUpdate=true;this.ground.geometry.computeVertexNormals();for(const label of this.labels){label.visible=this.gridEnabled&&scene==='flat'&&roughness===0;}this.uniforms.markings.value=this.gridEnabled&&scene==='flat'?1:0;this.ground.material.roughness=scene==='ice'?.22:.78;this.ground.material.metalness=scene==='ice'?.3:.18;}
 setGrid(enabled:boolean){this.gridEnabled=enabled;this.uniforms.gridStrength.value=enabled?.4:0;this.uniforms.markings.value=enabled&&this.terrain.scene==='flat'?1:0;for(const label of this.labels)label.visible=enabled&&this.terrain.scene==='flat'&&this.terrain.roughness===0;}
 dispose(){this.csm.remove();this.csm.dispose();this.environmentTarget.dispose();this.pmrem.dispose();this.texture.dispose();this.ground.geometry.dispose();this.ground.material.dispose();for(const label of this.labels){label.geometry.dispose();(label.material as T.MeshBasicMaterial).map?.dispose();(label.material as T.Material).dispose();}}
}
