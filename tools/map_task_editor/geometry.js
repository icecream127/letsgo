const Geo = {
  toWorld(m,u,v) {
    const a=u*m.resolution,b=(m.height-v)*m.resolution,c=Math.cos(m.origin[2]),s=Math.sin(m.origin[2]);
    return {x:m.origin[0]+c*a-s*b,y:m.origin[1]+s*a+c*b};
  },
  toPixel(m,x,y) {
    const a=x-m.origin[0],b=y-m.origin[1],c=Math.cos(m.origin[2]),s=Math.sin(m.origin[2]);
    return {u:(c*a+s*b)/m.resolution,v:m.height-(-s*a+c*b)/m.resolution};
  },
  cell(m,pixels,x,y) {
    const p=this.toPixel(m,x,y),u=Math.floor(p.u),v=Math.floor(p.v);
    if(u<0||v<0||u>=m.width||v>=m.height)return 'outside';
    const f=pixels[v*m.width+u]/255,occ=m.negate?f:1-f;
    return occ>m.occupied_thresh?'occupied':occ<m.free_thresh?'free':'unknown';
  },
  check(m,pixels,p,radius) {
    const center=this.cell(m,pixels,p.x,p.y);
    if(center!=='free')return center;
    const loc=this.toPixel(m,p.x,p.y),rr=radius/m.resolution;
    // Conservative circle versus grid squares, including unknown/outside cells.
    for(let v=Math.floor(loc.v-rr);v<=Math.floor(loc.v+rr);v++)
      for(let u=Math.floor(loc.u-rr);u<=Math.floor(loc.u+rr);u++) {
        const dx=Math.max(u-loc.u,0,loc.u-u-1),dy=Math.max(v-loc.v,0,loc.v-v-1);
        if(dx*dx+dy*dy>rr*rr)continue;
        const w=this.toWorld(m,u+.5,v+.5);
        if(this.cell(m,pixels,w.x,w.y)!=='free')return 'clearance';
      }
    return 'free';
  },
  centerline(points) {
    if(!Array.isArray(points)||points.length!==4||!points.every(p=>p&&Number.isFinite(p.x)&&Number.isFinite(p.y)))throw Error('请依次标出两条边界的四个端点');
    const [a,b,c,d]=points, len=Math.hypot(b.x-a.x,b.y-a.y),len2=Math.hypot(d.x-c.x,d.y-c.y);
    if(len<.1||len2<.1)throw Error('边界太短，请至少标出 0.1 米');
    let ux=(b.x-a.x)/len,uy=(b.y-a.y)/len,vx=(d.x-c.x)/len2,vy=(d.y-c.y)/len2;
    if(ux*vx+uy*vy<0){vx=-vx;vy=-vy}
    const angle=Math.acos(Math.min(1,Math.max(-1,ux*vx+uy*vy)));
    if(angle>3*Math.PI/180)throw Error('两侧边界相差超过 3°，无法生成平行中线，请重新标注');
    const n=Math.hypot(ux+vx,uy+vy);ux=(ux+vx)/n;uy=(uy+vy)/n;
    const t=p=>(p.x-a.x)*ux+(p.y-a.y)*uy, h=p=>-(p.x-a.x)*uy+(p.y-a.y)*ux;
    const low=Math.max(Math.min(t(a),t(b)),Math.min(t(c),t(d))),high=Math.min(Math.max(t(a),t(b)),Math.max(t(c),t(d)));
    if(high-low<.1)throw Error('两条边界没有足够的重叠长度');
    const h1=(h(a)+h(b))/2,h2=(h(c)+h(d))/2,width=Math.abs(h2-h1),offset=(h1+h2)/2;
    if(width<.05)throw Error('两侧边界间距不足 5 厘米');
    const pose=t=>({x:a.x+ux*t-uy*offset,y:a.y+uy*t+ux*offset,yaw:Math.atan2(uy,ux)});
    return {entry:pose(low),exit:pose(high),width,angle_deg:angle*180/Math.PI};
  },
  projectToCenterline(points,p) {
    const line=this.centerline(points),c=Math.cos(line.entry.yaw),s=Math.sin(line.entry.yaw);
    const distance=(p.x-line.entry.x)*c+(p.y-line.entry.y)*s;
    return {x:line.entry.x+distance*c,y:line.entry.y+distance*s,yaw:line.entry.yaw};
  },
  validateProject(doc,m) {
    if(doc.version!==1||doc.frame_id!=='map'||doc.map_fingerprint!==m.fingerprint)throw Error('文件版本或地图不匹配，请使用同一张地图的任务工程。');
    if(!Number.isFinite(doc.radius)||doc.radius<0||doc.radius>5)throw Error('检查半径无效');
    if(!Array.isArray(doc.tasks)||doc.tasks.length>500)throw Error('任务列表无效');
    for(const t of doc.tasks){
      if(typeof t.name!=='string'||t.name.length>100||!['nav','ramp','bridge','step','crawl','jump','custom'].includes(t.type))throw Error('任务名称或类型无效');
      if(t.boundaries!==undefined)this.centerline(t.boundaries);
      for(const k of ['guide_enabled','snap_centerline'])if(t[k]!==undefined&&typeof t[k]!=='boolean')throw Error('中线设置无效');
      if((t.guide_enabled||t.snap_centerline)&&!t.boundaries)throw Error('中线缺少边界');
      for(const k of ['entry','exit'])if(t[k]!==null&&(!t[k]||!['x','y','yaw'].every(v=>Number.isFinite(t[k][v]))))throw Error('任务点坐标无效');
    }
    if(doc.start_area!==undefined){
      const a=doc.start_area;
      if(!a||![a.size_x,a.size_y].every(v=>Number.isFinite(v)&&v>=.05&&v<=50))throw Error('启动区尺寸无效');
      if(a.pose!==null&&(!a.pose||!['x','y','yaw'].every(k=>Number.isFinite(a.pose[k]))))throw Error('启动区坐标无效');
    }
    return doc;
  }
};
if(typeof module!=='undefined')module.exports=Geo;
