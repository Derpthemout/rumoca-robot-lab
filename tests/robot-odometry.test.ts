import {it,expect} from 'vitest';
import {LidarOdometry,type Pose2} from '../src/robotics/odometry';
// Analytic intersections with an asymmetric room and internal wall segment.
function scan(p:Pose2){
 const segments=[[-1,-1,1.4,-1],[1.4,-1,1.4,1.2],[1.4,1.2,-1,1.2],[-1,1.2,-1,-1],[.5,.2,.5,.7]];
 const ranges=Array.from({length:128},(_,i)=>{const a=p.yaw+2*Math.PI*i/128,dx=Math.cos(a),dy=Math.sin(a);let best=3;
 for(const [x,y,u,v] of segments){const ex=u-x,ey=v-y,det=dx*ey-dy*ex;if(Math.abs(det)<1e-9)continue;const t=((x-p.x)*ey-(y-p.y)*ex)/det,s=((x-p.x)*dy-(y-p.y)*dx)/det;if(t>0&&s>=0&&s<=1)best=Math.min(best,t);}
 return best<3?best:null;});return {ranges,maxRange:3};
}
it('tracks small translations and yaw from ranges alone',()=>{
 const tracker=new LidarOdometry();tracker.initialize(scan({x:0,y:0,yaw:0}),{x:0,y:0,yaw:0});
 let result;for(let i=1;i<=10;i++)result=tracker.update(scan({x:i*.005,y:i*.002,yaw:i*.002}));
 expect(Math.hypot(result!.pose.x-.05,result!.pose.y-.02)).toBeLessThan(.025);expect(Math.abs(result!.pose.yaw-.02)).toBeLessThan(.025);expect(result!.rmse).toBeLessThan(.04);
});
it('rejects missing or invalid geometry without poisoning the reference',()=>{
 const t=new LidarOdometry(),initial=scan({x:0,y:0,yaw:0});t.initialize(initial,{x:0,y:0,yaw:0});
 expect(()=>t.update({ranges:Array(128).fill(null),maxRange:3})).toThrow();expect(()=>t.update({ranges:Array(128).fill(NaN),maxRange:3})).toThrow();
 const result=t.update(initial);expect(result.pose.x).toBeCloseTo(0,8);expect(result.rmse).toBeLessThan(1e-8);
 t.reset();expect(()=>t.update(initial)).toThrow(/Initialize/);
});
it('anchors once and keeps identical scans stationary in an arbitrary map frame',()=>{
 const s=scan({x:0,y:0,yaw:0}),t=new LidarOdometry(),anchor={x:.7,y:-.2,yaw:.4};t.initialize(s,anchor);
 expect(t.update(s).pose.x).toBeCloseTo(anchor.x,8);expect(t.update(s).pose.yaw).toBeCloseTo(anchor.yaw,8);
});
import {MappingSession} from '../src/robotics/localization';
import {OccupancyMap} from '../src/robotics/mapping';
function planar(p:Pose2){const c=Math.cos(p.yaw),s=Math.sin(p.yaw);return {...scan(p),kind:'lidar',frame:{position:[p.x,p.y,.22] as [number,number,number],rotation:[c,-s,0,s,c,0,0,0,1]}};}
it('uses truth only for the first anchor and diagnostics, not subsequent estimates',()=>{
 const a=new MappingSession(new OccupancyMap()),b=new MappingSession(new OccupancyMap());a.reset('estimated');b.reset('estimated');
 const first=planar({x:0,y:0,yaw:0});a.integrate(first);b.integrate(first);
 const next=planar({x:.01,y:.005,yaw:.004}),fake={...next,frame:{...next.frame,position:[.8,-.7,.22] as [number,number,number]}};
 const x=a.integrate(next)!,y=b.integrate(fake)!;expect(y.estimate).toEqual(x.estimate);expect(y.positionError).toBeGreaterThan(.9);expect(b.map.evidence).toEqual(a.map.evidence);
});
it('stops fusion on tracking failure until explicitly reset',()=>{
 const session=new MappingSession(new OccupancyMap());session.reset('estimated');const s=planar({x:0,y:0,yaw:0});session.integrate(s);const before=Array.from(session.map.evidence);
 expect(()=>session.integrate({...s,ranges:Array(128).fill(null)})).toThrow();expect(session.map.scans).toBe(1);expect(Array.from(session.map.evidence)).toEqual(before);
 expect(()=>session.integrate(s)).toThrow(/stopped/);expect(session.export().trackingStatus).toBe('lost');session.reset('estimated');session.integrate(s);expect(session.map.scans).toBe(1);
});
