import {it,expect} from 'vitest';
import {GJK,EPA,meshInertia,type Convex} from '../src/physics/convex';
import {vec,identity} from '../src/physics/spatial';
const vertices=Float64Array.of(-1,-1,-1,1,-1,-1,1,1,-1,-1,1,-1,-1,-1,1,1,-1,1,1,1,1,-1,1,1);
const faces=Uint32Array.of(0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7);
it('divergence-theorem cube mass and full inertia match analytic values',()=>{const m=meshInertia(vertices,faces,3);expect(m.mass).toBeCloseTo(24,12);for(let i=0;i<3;i++){expect(m.com[i]).toBeCloseTo(0,12);for(let j=0;j<3;j++)expect(m.inertia[i*3+j]).toBeCloseTo(i===j?16:0,12);}});
it('GJK returns exact separated cube distance and detects overlap',()=>{const R=vec(9);identity(R,3);const a:Convex={vertices,rotation:R,position:vec(3)},b:Convex={vertices,rotation:R,position:Float64Array.of(3,0,0)},g=new GJK();expect(g.query(a,b)).toBe(false);expect(g.distance).toBeCloseTo(1,10);b.position.set([1,.3,.5]);expect(g.query(a,b)).toBe(true);});

it('EPA cube penetration depth and witness separation are analytic',()=>{const R=vec(9);identity(R,3);const a:Convex={vertices,rotation:R,position:Float64Array.of(0,0,1.7)},b:Convex={vertices,rotation:R,position:vec(3)},e=new EPA();expect(e.query(a,b)).toBe(true);expect(e.depth).toBeCloseTo(.3,8);expect(e.normal[2]).toBeCloseTo(1,8);expect(e.pointB[2]-e.pointA[2]).toBeCloseTo(.3,8);});
