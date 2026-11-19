import {Matrix4, Quaternion, Vector3} from 'three';
export type Vec3 = [number,number,number];
export interface Joint {name:string;axis:Vec3;position:Vec3;limits:[number,number]}
export interface Link {name:string;parent:string|null;position:Vec3;quaternion:[number,number,number,number];joint?:string}
export interface Site {name:string;link:string;position:Vec3;quaternion:[number,number,number,number]}
export interface Visual {link:string;mesh:string;position:Vec3;quaternion:[number,number,number,number];rgba:[number,number,number,number]}
export interface RobotProfile {visuals?:Visual[];version:1;name:string;description:string;links:Link[];joints:Joint[];sites:Site[];source?:string}
export function validateProfile(value:unknown):RobotProfile {
 const p=value as RobotProfile;
 if(!p||p.version!==1||typeof p.name!=='string'||!Array.isArray(p.links)||!Array.isArray(p.joints)||!Array.isArray(p.sites)||!p.links.length||p.links.length>80||p.joints.length>40||p.sites.length>40)throw Error('Invalid robot profile or model too large');
 const vector=(v:unknown,n:number)=>Array.isArray(v)&&v.length===n&&v.every(x=>typeof x==='number'&&Number.isFinite(x));
 const names=new Set<string>(),joints=new Set<string>(),used=new Set<string>();let roots=0;
 for(const j of p.joints){if(!j||typeof j.name!=='string'||joints.has(j.name)||!vector(j.axis,3)||Math.hypot(...j.axis)<1e-9||!vector(j.position,3)||!vector(j.limits,2)||j.limits[0]>=j.limits[1])throw Error('Invalid or duplicate joint');joints.add(j.name);}
 for(const l of p.links){
  if(!l||typeof l.name!=='string'||names.has(l.name)||!vector(l.position,3)||!vector(l.quaternion,4)||Math.hypot(...l.quaternion)<1e-9||!(l.parent===null||names.has(l.parent)))throw Error('Links must form an ordered tree with finite transforms');
  if(l.parent===null)roots++;
  if(l.joint){if(!joints.has(l.joint)||used.has(l.joint))throw Error('Unknown or reused joint');used.add(l.joint);}names.add(l.name);
 }
 if(roots!==1||used.size!==joints.size)throw Error('Require one root and exactly one link for each joint');
 const sites=new Set<string>();for(const s of p.sites){if(!s||typeof s.name!=='string'||sites.has(s.name)||!names.has(s.link)||!vector(s.position,3)||!vector(s.quaternion,4)||Math.hypot(...s.quaternion)<1e-9)throw Error('Invalid site');sites.add(s.name);}
 if(p.visuals){if(!Array.isArray(p.visuals)||p.visuals.length>200)throw Error('Invalid visuals');for(const v of p.visuals){if(!v||!names.has(v.link)||typeof v.mesh!=='string'||! /^[a-zA-Z0-9_]+\.stl$/.test(v.mesh)||!vector(v.position,3)||!vector(v.quaternion,4)||Math.hypot(...v.quaternion)<1e-9||!vector(v.rgba,4)||v.rgba.some(x=>x<0||x>1))throw Error('Invalid visual geometry');}}
 return p;
}
function rotation(q:Link['quaternion']){
 const [w,x,y,z]=q;const m=new Matrix4().makeRotationFromQuaternion(new Quaternion(x,y,z,w).normalize()).elements;
 return [[m[0],m[4],m[8]],[m[1],m[5],m[9]],[m[2],m[6],m[10]]];
}
const num=(n:number)=>'('+String(Object.is(n,-0)?0:n)+')';
/** Emit declarative FK equations. Rumoca owns evaluation; no JS FK runtime. */
export function modelicaSource(profile:RobotProfile):string {
 const p=validateProfile(profile),decl=['Real clock(start=0, fixed=true);'],eq=['der(clock) = 1;'];
 p.joints.forEach((_,i)=>decl.push(`input Real q${i};`));
 const all=[...p.links.map(l=>({...l,parentIndex:p.links.findIndex(x=>x.name===l.parent)})),...p.sites.map(s=>({...s,parentIndex:p.links.findIndex(l=>l.name===s.link),joint:undefined}))];
 all.forEach((l,i)=>{
  decl.push(`output Real p${i}[3];`,`output Real r${i}[3,3];`);
  const fixed=rotation(l.quaternion),joint=p.joints.findIndex(j=>j.name===l.joint),j=p.joints[joint];
  let local=fixed.map(row=>row.map(num));let offset=l.position.map(num);
  if(j){
   const a=new Vector3(...j.axis).normalize().toArray(),c=`cos(q${joint})`,s=`sin(q${joint})`;
   const skew=[[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]];
   const rot=a.map((u,row)=>a.map((v,col)=>`(${row===col?c:'0'} + ${num(u*v)}*(1-${c}) + ${num(skew[row][col])}*${s})`));
   local=fixed.map(row=>a.map((_,col)=>row.map((f,k)=>`(${num(f)})*${rot[k][col]}`).join(' + ')));
   offset=l.position.map((v,row)=>`${num(v)} + `+j.position.map((v,k)=>`(${num(fixed[row][k])}-(${local[row][k]}))*(${num(v)})`).join(' + '));
  }
  for(let row=0;row<3;row++){
   eq.push(`p${i}[${row+1}] = `+(l.parentIndex<0?offset[row]:`p${l.parentIndex}[${row+1}] + `+offset.map((v,k)=>`r${l.parentIndex}[${row+1},${k+1}]*(${v})`).join(' + '))+';');
   for(let col=0;col<3;col++)eq.push(`r${i}[${row+1},${col+1}] = `+(l.parentIndex<0?local[row][col]:local.map((r,k)=>`r${l.parentIndex}[${row+1},${k+1}]*(${r[col]})`).join(' + '))+';');
  }
 });
 return `model RobotKinematics\n${decl.join('\n')}\nequation\n${eq.join('\n')}\nend RobotKinematics;\n`;
}
export function chain(profile:RobotProfile,siteName:string):number[]{
 const site=profile.sites.find(s=>s.name===siteName);if(!site)throw Error('Unknown end effector');
 let link=profile.links.find(l=>l.name===site.link);const result:number[]=[];
 while(link){if(link.joint)result.unshift(profile.joints.findIndex(j=>j.name===link!.joint));link=profile.links.find(l=>l.name===link!.parent);}return result;
}
export interface Frame {position:Vec3;rotation:number[]}
export interface KinematicSession {set_inputs(v:string):void;state_json():string;free():void}
export function evaluate(session:KinematicSession,profile:RobotProfile,q:number[]):Frame[]{
 if(q.length!==profile.joints.length||q.some((v,i)=>!Number.isFinite(v)||v<profile.joints[i].limits[0]-1e-9||v>profile.joints[i].limits[1]+1e-9))throw Error('Joint target outside limits');
 session.set_inputs(JSON.stringify(q.map((v,i)=>[`q${i}`,v])));
 const {values}=JSON.parse(session.state_json());
 const read=(name:string)=>{const v=values[name];if(typeof v!=='number'||!Number.isFinite(v))throw Error(`Invalid Rumoca output ${name}`);return v;};
 return [...profile.links,...profile.sites].map((_,i)=>({position:[1,2,3].map(k=>read(`p${i}[${k}]`)) as Vec3,rotation:[1,2,3].flatMap(r=>[1,2,3].map(c=>read(`r${i}[${r},${c}]`)))}));
}
