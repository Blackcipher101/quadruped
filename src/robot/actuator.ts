/** Geared PMSM output-shaft envelope, SI. Parameterized representative actuator, not vendor identification. */
export class Actuator {
 gear=9;efficiency=.85;torqueConstant=.095;currentLimit=33;voltage=48;resistance=.25;viscous=.03;coulomb=.08;deratedTorque=0;current=0;
 command(torque:number,speed:number){const motorSpeed=speed*this.gear,voltageCurrent=Math.max(0,(this.voltage-Math.abs(motorSpeed)*this.torqueConstant)/this.resistance);const limit=Math.min(this.currentLimit,voltageCurrent);this.deratedTorque=this.gear*this.efficiency*this.torqueConstant*limit;const electromagnetic=Math.max(-this.deratedTorque,Math.min(this.deratedTorque,torque));this.current=electromagnetic/(this.gear*this.efficiency*this.torqueConstant);return electromagnetic-this.viscous*speed-this.coulomb*Math.sign(speed);}
}
