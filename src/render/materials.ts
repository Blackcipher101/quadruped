import * as T from 'three';
/** Procedural renderer-only surface maps. They never displace collision geometry. */
export function surfaceTexture(kind:'carbon'|'concrete'|'brushed',size=512){
 const canvas=document.createElement('canvas');canvas.width=canvas.height=size;const ctx=canvas.getContext('2d')!,image=ctx.createImageData(size,size);let seed=73917;
 const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
 for(let y=0;y<size;y++)for(let x=0;x<size;x++){const i=(y*size+x)*4;let n=0;if(kind==='carbon'){const weave=(Math.floor(x/8)+Math.floor(y/8))%2,thread=weave?Math.sin(x*1.6):Math.sin(y*1.6);n=48+weave*13+thread*11+random()*5;}else if(kind==='brushed'){n=180+Math.sin(y*3.4)*15+random()*18;}else{n=153+random()*38+5*Math.sin(x*.15)*Math.cos(y*.17)+3*Math.sin(x*.8+y*.37);}image.data[i]=image.data[i+1]=image.data[i+2]=n;image.data[i+3]=255;}
 ctx.putImageData(image,0,0);const texture=new T.CanvasTexture(canvas);texture.wrapS=texture.wrapT=T.RepeatWrapping;texture.colorSpace=T.SRGBColorSpace;texture.anisotropy=8;return texture;
}
export function labelTexture(title:string,subtitle:string,color='#c5c9c7',background='transparent'){
 const canvas=document.createElement('canvas');canvas.width=1024;canvas.height=256;const c=canvas.getContext('2d')!;if(background!=='transparent'){c.fillStyle=background;c.fillRect(0,0,1024,256);}c.fillStyle=color;c.font='700 122px "Manrope Variable", sans-serif';c.fillText(title,32,135);c.font='26px "IBM Plex Mono", monospace';c.fillText(subtitle,39,203);return new T.CanvasTexture(canvas);
}
export function robotMaterials(){const carbon=surfaceTexture('carbon',128),brushed=surfaceTexture('brushed',256);carbon.repeat.set(3,10);brushed.repeat.set(1,3);return {
 shell:new T.MeshPhysicalMaterial({color:0xc8ccc7,metalness:.55,roughness:.32,clearcoat:.38,clearcoatRoughness:.3}),
 dark:new T.MeshStandardMaterial({color:0x252b2c,metalness:.72,roughness:.32}),
 carbon:new T.MeshPhysicalMaterial({color:0x596465,map:carbon,roughness:.4,metalness:.35,clearcoat:.6,clearcoatRoughness:.2}),
 metal:new T.MeshStandardMaterial({color:0x8d9a9e,metalness:.92,roughness:.27,roughnessMap:brushed}),
 orange:new T.MeshPhysicalMaterial({color:0xd95f29,metalness:.25,roughness:.35,clearcoat:.5}),
 rubber:new T.MeshStandardMaterial({color:0x151b1b,roughness:.94}),
 glass:new T.MeshPhysicalMaterial({color:0x172c30,metalness:.92,roughness:.055,clearcoat:1}),
 led:new T.MeshStandardMaterial({color:0xf4995e,emissive:0xff8842,emissiveIntensity:3.2,roughness:.2}),
 white:new T.MeshStandardMaterial({color:0xc1d4cc,emissive:0x99c4b5,emissiveIntensity:1.1,roughness:.2})
};}
