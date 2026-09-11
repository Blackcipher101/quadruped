import * as T from 'three';
import {EffectComposer} from 'three/addons/postprocessing/EffectComposer.js';
import {RenderPass} from 'three/addons/postprocessing/RenderPass.js';
import {GTAOPass} from 'three/addons/postprocessing/GTAOPass.js';
import {UnrealBloomPass} from 'three/addons/postprocessing/UnrealBloomPass.js';
import {OutputPass} from 'three/addons/postprocessing/OutputPass.js';
import {ShaderPass} from 'three/addons/postprocessing/ShaderPass.js';
import {FXAAShader} from 'three/addons/shaders/FXAAShader.js';
export type Quality='ultra'|'balanced'|'performance';
export class PostProcessing {
 readonly composer:EffectComposer;readonly ao:GTAOPass;readonly bloom:UnrealBloomPass;readonly fxaa:ShaderPass;readonly grade:ShaderPass;
 quality:Quality='balanced';aoEnabled=true;bloomEnabled=true;private width=1;private height=1;
 constructor(readonly renderer:T.WebGLRenderer,scene:T.Scene,camera:T.PerspectiveCamera){
  const target=new T.WebGLRenderTarget(1,1,{type:T.HalfFloatType,depthBuffer:true});target.samples=2;
  this.composer=new EffectComposer(renderer,target);this.composer.addPass(new RenderPass(scene,camera));this.ao=new GTAOPass(scene,camera,1,1);this.ao.blendIntensity=.65;this.ao.updateGtaoMaterial({radius:.14,thickness:.08,distanceFallOff:1,scale:1,samples:8});this.ao.updatePdMaterial({radius:5,samples:8});this.composer.addPass(this.ao);
  this.bloom=new UnrealBloomPass(new T.Vector2(1,1),.16,.4,1.6);this.composer.addPass(this.bloom);
  this.grade=new ShaderPass({uniforms:{tDiffuse:{value:null},vignette:{value:.2}},vertexShader:'varying vec2 vUv; void main(){vUv=uv;gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.0);}',fragmentShader:'uniform sampler2D tDiffuse;uniform float vignette;varying vec2 vUv;void main(){vec4 c=texture2D(tDiffuse,vUv);vec2 p=vUv*2.0-1.0;float v=1.0-vignette*dot(p,p)*.5;gl_FragColor=vec4(c.rgb*v,c.a);}'});this.composer.addPass(this.grade);this.composer.addPass(new OutputPass());this.fxaa=new ShaderPass(FXAAShader);this.composer.addPass(this.fxaa);
 }
 setQuality(quality:Quality){this.quality=quality;this.ao.enabled=this.aoEnabled&&quality!=='performance';this.bloom.enabled=this.bloomEnabled&&quality!=='performance';this.ao.updateGtaoMaterial({samples:quality==='ultra'?16:8});this.renderer.setPixelRatio(Math.min(devicePixelRatio,quality==='ultra'?2:quality==='balanced'?1.5:1));this.resize(this.width,this.height);}
 resize(width:number,height:number){this.width=width;this.height=height;const ratio=this.renderer.getPixelRatio();this.composer.setPixelRatio(ratio);this.composer.setSize(width,height);if(this.quality!=='ultra')this.ao.setSize(Math.max(1,Math.floor(width*ratio*.65)),Math.max(1,Math.floor(height*ratio*.65)));this.fxaa.material.uniforms.resolution.value.set(1/(width*ratio),1/(height*ratio));}
 render(){this.composer.render();}
 dispose(){for(const pass of this.composer.passes)pass.dispose();this.composer.dispose();}
}
