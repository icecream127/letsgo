const assert=require('node:assert/strict');
const G=require('../tools/map_task_editor/geometry.js');
const m={width:4,height:4,resolution:.5,origin:[-2,3,Math.PI/2],negate:false,free_thresh:.196,occupied_thresh:.65,fingerprint:'test'};
for(const [u,v] of [[.5,.5],[3.5,3.5],[0,4]]){const w=G.toWorld(m,u,v),p=G.toPixel(m,w.x,w.y);assert.ok(Math.abs(p.u-u)<1e-10&&Math.abs(p.v-v)<1e-10)}
const w=G.toWorld(m,.5,3.5);assert.ok(Math.abs(w.x+2.25)<1e-10&&Math.abs(w.y-3.25)<1e-10);
let pixels=new Uint8Array(16).fill(254);pixels[0]=0;pixels[1]=205;
for(const [u,v,s]of [[.5,.5,'occupied'],[1.5,.5,'unknown'],[2.5,2.5,'free'],[-1,2,'outside']]){const p=G.toWorld(m,u,v);assert.equal(G.cell(m,pixels,p.x,p.y),s)}
let p=G.toWorld(m,2.5,2.5);assert.equal(G.check(m,pixels,p,.1),'free');assert.equal(G.check(m,pixels,p,2),'clearance');
const doc={version:1,frame_id:'map',map_fingerprint:'test',radius:.3,tasks:[{name:'A',type:'nav',entry:{x:1,y:2,yaw:0},exit:null}]};assert.equal(G.validateProject(doc,m),doc);assert.throws(()=>G.validateProject({...doc,map_fingerprint:'wrong'},m));assert.throws(()=>G.validateProject({...doc,tasks:[{...doc.tasks[0],entry:{x:NaN,y:2,yaw:0}}]},m));
console.log('PASS: rotated coordinate roundtrip, known cell center, occupancy, clearance, project validation');
const startDoc={...doc,start_area:{pose:{x:1,y:2,yaw:Math.PI/2},size_x:2,size_y:1}};
assert.deepEqual(G.validateProject(JSON.parse(JSON.stringify(startDoc)),m).start_area,startDoc.start_area);
assert.throws(()=>G.validateProject({...doc,start_area:{pose:null,size_x:-1,size_y:1}},m));
assert.throws(()=>G.validateProject({...doc,start_area:{pose:{x:Infinity,y:0,yaw:0},size_x:1,size_y:1}},m));
console.log('PASS: start area roundtrip, invalid dimensions/pose rejected, old projects still accepted');
const line=G.centerline([{x:0,y:0},{x:4,y:0},{x:3,y:2},{x:1,y:2}]);
assert.deepEqual(line.entry,{x:1,y:1,yaw:0});assert.deepEqual(line.exit,{x:3,y:1,yaw:0});assert.equal(line.width,2);
const vertical=G.centerline([{x:2,y:0},{x:2,y:4},{x:0,y:1},{x:0,y:3}]);assert.equal(vertical.entry.x,1);assert.equal(vertical.entry.y,1);assert.equal(vertical.exit.y,3);assert.equal(vertical.entry.yaw,Math.PI/2);
assert.throws(()=>G.centerline([{x:0,y:0},{x:2,y:0},{x:0,y:1},{x:2,y:2}]));
assert.throws(()=>G.centerline([{x:0,y:0},{x:1,y:0},{x:2,y:1},{x:3,y:1}]));
console.log('PASS: centerline halfway/parallel, opposite edge ordering, overlap, vertical direction, invalid edges');
const edges=[{x:0,y:0},{x:4,y:0},{x:0,y:2},{x:4,y:2}];
assert.deepEqual(G.projectToCenterline(edges,{x:-2,y:7}),{x:-2,y:1,yaw:0});
assert.deepEqual(G.projectToCenterline(edges,{x:9,y:-4}),{x:9,y:1,yaw:0});
assert.deepEqual(G.projectToCenterline(edges,{x:2,y:1.2}),{x:2,y:1,yaw:0});
console.log('PASS: free selection on centerline and both outside extensions');
