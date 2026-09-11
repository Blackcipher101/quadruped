/** Single producer/single consumer triple buffer. Each side owns an immutable slot.
 * Header[0] is the middle slot with dirty bit 4. Atomics.exchange transfers ownership.
 * No float is read concurrently with a write; no torn-pose seqlock assumptions.
 */
export const SNAPSHOT_SIZE=8192;
export const HEADER_BYTES=16;
export const POSE_PREVIOUS=8;
export const POSE_CURRENT=POSE_PREVIOUS+13*7;
export const TELEMETRY=POSE_CURRENT+13*7;
export const WORLD_META=256;
export const ROCK_PREVIOUS=272;
export const ROCK_CURRENT=ROCK_PREVIOUS+40*7;
export const FLUID_POSITIONS=ROCK_CURRENT+40*7;
export const FLUID_PREVIOUS=FLUID_POSITIONS+1120*3;
export function createExchange(){const buffer=new SharedArrayBuffer(HEADER_BYTES+3*SNAPSHOT_SIZE*8);new Int32Array(buffer,0,4)[0]=1;return buffer;}
export class Exchange {
 readonly header:Int32Array;readonly slots:Float64Array[];private owned:number;
 constructor(buffer:SharedArrayBuffer,readonly writer:boolean){this.header=new Int32Array(buffer,0,4);this.slots=[0,1,2].map(i=>new Float64Array(buffer,HEADER_BYTES+i*SNAPSHOT_SIZE*8,SNAPSHOT_SIZE));this.owned=writer?2:0;}
 get data(){return this.slots[this.owned];}
 publish(){if(!this.writer)throw Error('Consumer cannot publish');this.owned=Atomics.exchange(this.header,0,this.owned|4)&3;}
 consume(){if(this.writer)throw Error('Producer cannot consume');if(Atomics.load(this.header,0)&4){this.owned=Atomics.exchange(this.header,0,this.owned)&3;return true;}return false;}
}
