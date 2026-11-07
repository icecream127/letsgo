from pathlib import Path
import numpy as np,yaml,json
from collections import deque
from PIL import Image
root=Path(__file__).resolve().parents[1]
report_dir=root/'validation/results/competition'
report_dir.mkdir(parents=True,exist_ok=True)
cfg=yaml.safe_load((root/'maps/competition/map.yaml').read_text());res=cfg['resolution'];origin=np.array(cfg['origin'][:2]);im=np.flipud(np.array(Image.open(root/'maps/competition/map.pgm')))
free=im==254;safe=free.copy();radius=.30; conservative=radius+res/np.sqrt(2)
# Occupied and unknown cell squares both exclude the circular footprint.
for dx in range(-7,8):
 for dy in range(-7,8):
  if np.hypot(dx,dy)*res>conservative:continue
  shifted=np.zeros_like(free)
  ys=slice(max(0,dy),min(len(free),len(free)+dy));xs=slice(max(0,dx),min(free.shape[1],free.shape[1]+dx))
  shifted[ys,xs]=free[slice(max(0,-dy),min(len(free),len(free)-dy)),slice(max(0,-dx),min(free.shape[1],free.shape[1]-dx))];safe &=shifted
Y,X=np.indices(im.shape);X=origin[0]+(X+.5)*res;Y=origin[1]+(Y+.5)*res;n=Y+.36*X+2.7
# Stay within this lower passage and its C approach, preventing a detour around it.
safe &=(X>.7)&(X<5.6)&(n>-.45)&(n<2.2)
def nearest(xy):
 d=(X-xy[0])**2+(Y-xy[1])**2;d[~safe]=np.inf
 idx=np.unravel_index(np.argmin(d),im.shape)
 if np.sqrt(d[idx])>.25:raise ValueError(('No footprint-safe position near',xy,np.sqrt(d[idx])))
 return idx
start=nearest([5.,-2.8]);goal=nearest([1.2,-2.95]);q=deque([start]);parent={start:None}
while q:
 a,b=q.popleft()
 if (a,b)==goal:break
 for da,db in [(1,0),(-1,0),(0,1),(0,-1)]:
  nxt=(a+da,b+db)
  if 0<=nxt[0]<len(im) and 0<=nxt[1]<im.shape[1] and safe[nxt] and nxt not in parent:parent[nxt]=(a,b);q.append(nxt)
assert goal in parent,'C entry -> A disconnected for 0.30 m radius'
path=[];v=goal
while v is not None:path.append([float(X[v]),float(Y[v])]);v=parent[v]
path=path[::-1]
report={'result':'PASS','robot_radius_m':radius,'cell_square_clearance_included':True,'source':'competition/cloud.pcd','start':path[0],'goal':path[-1],'path_length_m':(len(path)-1)*res,'path':path,'method':'offline circular-footprint grid search, unknown blocked, constrained to lower passage and C approach','map_sha256':__import__('hashlib').sha256((root/'maps/competition/map.pgm').read_bytes()).hexdigest()}
(report_dir/'route_check.json').write_text(json.dumps(report,indent=2)+'\n')
print({k:v for k,v in report.items() if k!='path'})
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,9));ax.imshow(np.flipud(im),cmap='gray',vmin=0,vmax=254,extent=[origin[0],origin[0]+im.shape[1]*res,origin[1],origin[1]+im.shape[0]*res]);p=np.array(path);ax.plot(p[:,0],p[:,1],color='#00a060',lw=2)
for label,xy in [('Approach C',path[0]),('A',path[-1])]:ax.text(xy[0]+.1,xy[1]+.1,label,color='red',fontsize=12)
ax.text(5., -4.2, 'C', color='red', fontsize=12)
ax.set_title('Confirmed slatted passage: C entry to A, radius 0.30 m');ax.set_aspect('equal');fig.tight_layout();fig.savefig(report_dir/'route_preview.png',dpi=130)
