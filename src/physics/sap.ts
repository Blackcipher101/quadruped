/** Coherent insertion-sorted sweep and prune, deterministic ties by collider id. */
export class SweepAndPrune {
 readonly bounds:Float64Array;readonly order:Int32Array;readonly pairs:Int32Array;count=0;pairCount=0;
 constructor(readonly capacity:number){this.bounds=new Float64Array(capacity*6);this.order=new Int32Array(capacity);this.pairs=new Int32Array(capacity*capacity*2);for(let i=0;i<capacity;i++)this.order[i]=i;}
 update(){for(let i=1;i<this.count;i++){const id=this.order[i],min=this.bounds[id*6];let j=i-1;while(j>=0&&(this.bounds[this.order[j]*6]>min||(this.bounds[this.order[j]*6]===min&&this.order[j]>id))){this.order[j+1]=this.order[j];j--;}this.order[j+1]=id;}this.pairCount=0;
 for(let i=0;i<this.count;i++){const a=this.order[i];for(let j=i+1;j<this.count;j++){const b=this.order[j];if(this.bounds[b*6]>this.bounds[a*6+3])break;if(this.bounds[a*6+1]>this.bounds[b*6+4]||this.bounds[b*6+1]>this.bounds[a*6+4]||this.bounds[a*6+2]>this.bounds[b*6+5]||this.bounds[b*6+2]>this.bounds[a*6+5])continue;this.pairs[this.pairCount*2]=Math.min(a,b);this.pairs[this.pairCount*2+1]=Math.max(a,b);this.pairCount++;}}
 // Canonical pair order independent of the temporal sort history.
 for(let i=1;i<this.pairCount;i++){const a=this.pairs[i*2],b=this.pairs[i*2+1];let j=i-1;while(j>=0&&(this.pairs[j*2]>a||(this.pairs[j*2]===a&&this.pairs[j*2+1]>b))){this.pairs[(j+1)*2]=this.pairs[j*2];this.pairs[(j+1)*2+1]=this.pairs[j*2+1];j--;}this.pairs[(j+1)*2]=a;this.pairs[(j+1)*2+1]=b;}
 }
}
