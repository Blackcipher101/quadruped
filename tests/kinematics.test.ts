import {it,expect} from 'vitest';
import {Dynamics} from '../src/physics/dynamics';
import {pendulum} from '../src/physics/model';
import {PointKinematics} from '../src/physics/kinematics';
it('point Jacobian and analytic Jdot match central finite differences',()=>{const d=new Dynamics(pendulum(3)),p=new PointKinematics(d),local=Float64Array.of(.1,.03,-.8),h=1e-6;d.q.set([.3,-.4,.7]);d.v.set([.4,.2,-.5]);d.kinematics();p.compute(2,local);const J=p.J.slice(),Jdot=p.Jdot.slice(),q=d.q.slice();for(let c=0;c<3;c++){d.q[c]+=h;d.kinematics();p.compute(2,local);const plus=p.position.slice();d.q[c]-=2*h;d.kinematics();p.compute(2,local);for(let r=0;r<3;r++)expect(Math.abs((plus[r]-p.position[r])/(2*h)-J[r*3+c])).toBeLessThan(1e-9);d.q.set(q);}
 for(let c=0;c<3;c++)d.q[c]=q[c]+h*d.v[c];d.kinematics();p.compute(2,local);const plus=p.J.slice();for(let c=0;c<3;c++)d.q[c]=q[c]-h*d.v[c];d.kinematics();p.compute(2,local);for(let i=0;i<9;i++)expect(Math.abs((plus[i]-p.J[i])/(2*h)-Jdot[i])).toBeLessThan(1e-9);
});
