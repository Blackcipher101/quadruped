import {it,expect} from 'vitest';
import {createExchange,Exchange} from '../src/physics/exchange';
it('triple buffer never writes the consumer-owned snapshot',()=>{const buffer=createExchange(),w=new Exchange(buffer,true),r=new Exchange(buffer,false);w.data[0]=1;w.publish();expect(r.consume()).toBe(true);const owned=r.data;for(let i=2;i<100;i++){w.data[0]=i;w.publish();expect(owned[0]).toBe(1);}expect(r.consume()).toBe(true);expect(r.data[0]).toBe(99);expect(r.consume()).toBe(false);});
