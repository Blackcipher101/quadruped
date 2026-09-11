import {vec} from './spatial';
export type SceneName='flat'|'slope15'|'slope25'|'stairs'|'rocks'|'beam'|'ice'|'gravel'|'water';
export const materials={rubber:{mu:.9,restitution:.05},rock:{mu:.7,restitution:.1},dirt:{mu:.8,restitution:0},steel:{mu:.5,restitution:.1},ice:{mu:.15,restitution:0}};
export class Terrain {
 readonly normal=vec(3);roughness=0;scene:SceneName='flat';
 height(x:number,y:number){
  let h=0;if(this.scene==='slope15')h=x*Math.tan(15*Math.PI/180);if(this.scene==='slope25')h=x*Math.tan(25*Math.PI/180);
  if(this.scene==='stairs')h=Math.max(0,Math.min(5,Math.floor((x+1)/.35)))*.1;
  if(this.scene==='beam')h=Math.abs(y)<.10&&Math.abs(x)<2?.16:0;
  return h+this.roughness*(Math.sin(x*7.13+y*1.37)*Math.cos(y*8.31-x*.79));
 }
 sample(x:number,y:number){const h=this.height(x,y),epsilon=1e-5;const dx=(this.height(x+epsilon,y)-this.height(x-epsilon,y))/(2*epsilon),dy=(this.height(x,y+epsilon)-this.height(x,y-epsilon))/(2*epsilon),n=Math.hypot(dx,dy,1);this.normal[0]=-dx/n;this.normal[1]=-dy/n;this.normal[2]=1/n;return h;}
 friction(x:number,y:number){return this.scene==='ice'&&Math.abs(x)<1&&Math.abs(y)<1?.15:.8;}
}
