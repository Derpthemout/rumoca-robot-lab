import {it,expect} from 'vitest';
import {OccupancyMap,type PlanarScan} from '../src/robotics/mapping';
const scan=():PlanarScan=>({kind:'lidar',frame:{position:[0,0,.2],rotation:[1,0,0,0,1,0,0,0,1]},ranges:Array(128).fill(null),maxRange:2});
it('marks hit endpoints occupied, ray interiors free, and unseen space unknown',()=>{
 const map=new OccupancyMap(),s=scan();s.ranges[0]=1;map.integrate(s);
 const value=(x:number,y:number)=>{const c=map.cell([x,y]);return map.evidence[c[1]*map.size+c[0]];};
 expect(value(1,0)).toBeGreaterThan(0);expect(value(.5,0)).toBeLessThan(0);expect(value(2.5,2.5)).toBe(0);expect(map.scans).toBe(1);
 map.clear();expect(map.scans).toBe(0);expect(map.evidence.every(v=>v===0)).toBe(true);
});
it('rejects tilted and malformed scans without changing the map',()=>{
 const map=new OccupancyMap(),s=scan();s.frame.rotation[6]=.2;expect(()=>map.integrate(s)).toThrow(/level/);expect(map.scans).toBe(0);
 s.frame.rotation[6]=0;s.ranges[3]=NaN;expect(()=>map.integrate(s)).toThrow();expect(map.evidence.every(v=>v===0)).toBe(true);
});
it('routes around obstacles and refuses unknown space and insufficient footprint clearance',()=>{
 const map=new OccupancyMap();map.evidence.fill(-2);
 for(let y=40;y<80;y++)map.evidence[y*120+60]=6;
 const route=map.plan([-.5,0],[.5,0],.1);expect(route.length).toBeGreaterThan(40);
 for(const p of route){const [x,y]=map.cell(p);expect(map.evidence[y*120+x]).toBeLessThan(0);}
 expect(()=>map.plan([0,0],[.5,0],.1)).toThrow(/clearance/);
 map.clear();expect(()=>map.plan([0,0],[.5,0])).toThrow(/clearance/);
 map.evidence.fill(6);for(let x=20;x<100;x++)map.evidence[60*120+x]=-2;
 expect(map.plan([-.5,.025],[.5,.025],0).length).toBeGreaterThan(1);
 expect(()=>map.plan([-.5,.025],[.5,.025],.1)).toThrow();
});
it('uses world yaw for rays and yields repeatable maps',()=>{
 const a=new OccupancyMap(),b=new OccupancyMap(),s=scan();s.frame.rotation=[0,-1,0,1,0,0,0,0,1];s.ranges[0]=1;
 a.integrate(s);b.integrate(s);const c=a.cell([0,1]);expect(a.evidence[c[1]*120+c[0]]).toBeGreaterThan(0);expect(a.export()).toEqual(b.export());
});
it('keeps scans at one height and can start a new slice after clear',()=>{
 const map=new OccupancyMap(),s=scan();map.integrate(s);const before=map.export();s.frame.position[2]+=.02;
 expect(()=>map.integrate(s)).toThrow(/height/);expect(map.export()).toEqual(before);map.clear();map.integrate(s);expect(map.scans).toBe(1);
});
