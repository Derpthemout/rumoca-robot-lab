import type {Frame} from './model';
export type Point2=[number,number];
export interface PlanarScan {kind:string;frame:Frame;ranges:(number|null)[];maxRange:number}
/** Horizontal range-slice mapping with known poses, not a localization estimator. */
export class OccupancyMap {
  readonly size=120;readonly resolution=.05;readonly origin:Point2=[-3,-3];
  readonly evidence=new Int8Array(this.size*this.size);scans=0;private height:number|undefined;
  clear(){this.evidence.fill(0);this.scans=0;this.height=undefined;}
  cell(p:Point2){return p.map((v,i)=>Math.floor((v-this.origin[i])/this.resolution)) as Point2;}
  inside(x:number,y:number){return x>=0&&y>=0&&x<this.size&&y<this.size;}
  center(x:number,y:number):Point2{return [this.origin[0]+(x+.5)*this.resolution,this.origin[1]+(y+.5)*this.resolution];}
  integrate(scan:PlanarScan){
    const {frame,ranges,maxRange}=scan,r=frame.rotation;
    if(scan.kind!=='lidar'||ranges.length!==128||!Number.isFinite(maxRange)||maxRange<=0||maxRange>10||frame.position.length!==3||!frame.position.every(Number.isFinite)||r.length!==9||!r.every(Number.isFinite)||ranges.some(v=>v!==null&&(!Number.isFinite(v)||v<0||v>maxRange)))throw Error('Invalid planar LiDAR scan');
    // Tilted rays cannot establish free space in a horizontal navigation slice.
    if(Math.abs(r[6])>1e-4||Math.abs(r[7])>1e-4||Math.abs(r[8]-1)>1e-4)throw Error('Mapping requires level LiDAR. Use the root frame with zero roll/pitch.');
    const [ox,oy]=frame.position,[cx,cy]=this.cell([ox,oy]);if(!this.inside(cx,cy))throw Error('Sensor lies outside the map');
    if(this.height!==undefined&&Math.abs(frame.position[2]-this.height)>.005)throw Error('Scan height changed. Clear map before changing the navigation slice.');
    this.height??=frame.position[2];
    const free=new Set<number>(),hits=new Set<number>();
    ranges.forEach((range,i)=>{
      const angle=2*Math.PI*i/ranges.length,dx=r[0]*Math.cos(angle)+r[1]*Math.sin(angle),dy=r[3]*Math.cos(angle)+r[4]*Math.sin(angle),distance=range??maxRange;
      const end=this.cell([ox+dx*distance,oy+dy*distance]);
      // Subcell sampling covers the range segment; each cell receives one vote per scan.
      for(let d=0;d<distance;d+=this.resolution/4){const [x,y]=this.cell([ox+dx*d,oy+dy*d]);if(!this.inside(x,y))break;if(range!==null&&x===end[0]&&y===end[1])break;free.add(y*this.size+x);}
      if(range!==null&&this.inside(...end))hits.add(end[1]*this.size+end[0]);
    });
    free.forEach(i=>{if(!hits.has(i))this.evidence[i]=Math.max(-20,this.evidence[i]-2);});
    hits.forEach(i=>{this.evidence[i]=Math.min(20,this.evidence[i]+6);});this.scans++;
  }
  /** Unknown and occupied cells are blocked, including the complete circular footprint. */
  plan(start:Point2,goal:Point2,radius=.1):Point2[]{
    if(![...start,...goal,radius].every(Number.isFinite)||radius<0||radius>.5)throw Error('Invalid route coordinates or radius');
    const a=this.cell(start),b=this.cell(goal);if(!this.inside(...a)||!this.inside(...b))throw Error('Route lies outside map');
    const blocked=new Uint8Array(this.evidence.length),n=Math.ceil(radius/this.resolution);
    for(let y=0;y<this.size;y++)for(let x=0;x<this.size;x++){
      for(let dy=-n;dy<=n&&!blocked[y*this.size+x];dy++)for(let dx=-n;dx<=n;dx++){
        // Include cells whose area intersects the footprint, not only their centers.
        if(Math.hypot(Math.max(0,Math.abs(dx)-.5),Math.max(0,Math.abs(dy)-.5))*this.resolution>radius)continue;
        if(!this.inside(x+dx,y+dy)||this.evidence[(y+dy)*this.size+x+dx]>=0){blocked[y*this.size+x]=1;break;}
      }
    }
    const from=a[1]*this.size+a[0],to=b[1]*this.size+b[0];
    if(blocked[from]||blocked[to])throw Error('Start or goal lacks observed clearance. Capture more scans or choose a free location.');
    const parent=new Int32Array(blocked.length).fill(-1),queue=[from];parent[from]=from;
    // Equal-cost four-neighbor search avoids diagonal corner cutting.
    for(let head=0;head<queue.length&&parent[to]<0;head++){
      const at=queue[head],x=at%this.size,y=Math.floor(at/this.size);
      for(const [dx,dy] of [[1,0],[-1,0],[0,1],[0,-1]]){const nx=x+dx,ny=y+dy,next=ny*this.size+nx;if(this.inside(nx,ny)&&!blocked[next]&&parent[next]<0){parent[next]=at;queue.push(next);}}
    }
    if(parent[to]<0)throw Error('No route through observed free space');
    const result:Point2[]=[];for(let at=to;;at=parent[at]){result.push(this.center(at%this.size,Math.floor(at/this.size)));if(at===from)break;}return result.reverse();
  }
  export(){return {format:'rumoca-known-pose-occupancy',version:1,poseSource:'Rumoca ground truth; not SLAM',size:this.size,resolution:this.resolution,origin:this.origin,axes:'world XY; row-major y*size+x',units:'metres',height:this.height,scans:this.scans,evidence:Array.from(this.evidence),encoding:'negative=free, zero=unknown, positive=occupied; bounded evidence scores, not probabilities'};}
}
