"""Read-only legacy course illustration; no ROS or actuator access."""
import ast, csv, json, hashlib
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'validation/results/legacy_course'
OUT.mkdir(parents=True, exist_ok=True)
SRC = ROOT/'lidar_ws/src/ros2_to_stm32/ros2_to_stm32'
config = SRC/'map_zones_config.json'
controller = SRC/'task_to_do_unitree.py'
data = json.loads(config.read_text())
font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/msyh.ttc')
plt.rcParams.update({'font.family': font_manager.FontProperties(fname='/mnt/c/Windows/Fonts/msyh.ttc').get_name(), 'axes.unicode_minus':False, 'font.size':10})
methods = {n.name:n for n in ast.walk(ast.parse(controller.read_text())) if isinstance(n,ast.FunctionDef)}
colors = ['#007e91','#2361ad','#cc7a12','#8c58a6','#9b6535','#21875b','#cb5260','#637184']

def evaluate(n, env):
    if isinstance(n,ast.Constant): return n.value
    if isinstance(n,ast.Name): return env[n.id]
    if isinstance(n,ast.UnaryOp) and isinstance(n.op,ast.USub): return -evaluate(n.operand,env)
    if isinstance(n,ast.BinOp):
        a,b=evaluate(n.left,env),evaluate(n.right,env)
        if isinstance(n.op,ast.Add): return a+b
        if isinstance(n.op,ast.Sub): return a-b
    if isinstance(n,ast.Subscript): return evaluate(n.value,env)[evaluate(n.slice,env)]
    if isinstance(n,ast.Attribute) and isinstance(n.value,ast.Name) and n.value.id=='self' and n.attr=='zones': return data['zones']
    raise ValueError(ast.dump(n))

def owner(x,y):
    return next((name for name,z in data['zones'].items() if z['x_min']<=x<z['x_max'] and z['y_min']<=y<z['y_max']), '回退木桥A')

routes=[]
rows=[]
for i,(name,z) in enumerate(data['zones'].items()):
    assignment=next(n for n in ast.walk(methods[z['func_name']]) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='Waypoints' for t in n.targets))
    env=dict(x=0.,y=0.,yaw=0.,x_begin=z['begin_x'],y_begin=z['begin_y'])
    pts=[(z['begin_x'],z['begin_y'])]; dynamic=[True]
    for k,t in enumerate(assignment.value.elts,1):
        xy=t.elts[:2]
        dyn=any(isinstance(n,ast.Name) and n.id in ('x','y') for p in xy for n in ast.walk(p))
        x,y=[evaluate(p,env) for p in xy]
        gx,gy=x+z['begin_x'],y+z['begin_y']
        rows.append(dict(obstacle=name,step=k,x_expression=ast.unparse(xy[0]),y_expression=ast.unparse(xy[1]),yaw_expression=ast.unparse(t.elts[2]),mode=evaluate(t.elts[3],env),dynamic_xy=dyn,fixed_x='' if dyn else round(gx,3),fixed_y='' if dyn else round(gy,3),nominal_x=round(gx,3),nominal_y=round(gy,3),dispatch_at_fixed_target='' if dyn else owner(gx,gy)))
        pts.append((gx,gy));dynamic.append(dyn)
        env.update(x=x,y=y)
    routes.append((name,z,pts,dynamic,colors[i]))
with (OUT/'legacy_waypoints.csv').open('w',encoding='utf-8-sig',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def setup(ax,title,ylim=(2.4,-6.2)):
    ax.set_title(title,loc='left',fontsize=14,fontweight='bold',pad=13)
    ax.set_aspect('equal');ax.set_xlim(-9.2,6);ax.set_ylim(*ylim)
    ax.set_xlabel('旧赛道 x / 米（A→B 为正）');ax.set_ylabel('旧赛道 y / 米（右侧为正，图中向下）')
    ax.set_xticks(range(-9,7));ax.set_yticks(range(-6,6));ax.set_ylim(*ylim)
    ax.grid(True,color='#dfe5ea',linewidth=.6);ax.set_axisbelow(True)
    ax.axhline(0,color='#9ba9b2',lw=.8);ax.axvline(0,color='#9ba9b2',lw=.8)
    ax.scatter([0],[0],marker='*',s=140,c='#1c2833',zorder=8)
    ax.annotate('A 原点 / 台阶参考点',(0,0),xytext=(.25,1.4),arrowprops=dict(arrowstyle='-',color='#445566'),fontsize=9)
    ax.annotate('',xy=(2,1.65),xytext=(.3,1.65),arrowprops=dict(arrowstyle='->',color='#445566',lw=1.5))
    ax.text(.3,2.08,'A→B 方向；B 实际坐标未持久化',fontsize=8,color='#445566')

def regions(ax,labels=True):
    for i,(name,z,pts,dyn,c) in enumerate(routes):
        ax.add_patch(Rectangle((z['x_min'],z['y_min']),z['x_max']-z['x_min'],z['y_max']-z['y_min'],fc=c,ec=c,alpha=.12,lw=1.5))
        if labels:
            ax.text((z['x_min']+z['x_max'])/2,z['y_min']+.17,f'{i+1} {name}',ha='center',va='top',color=c,fontweight='bold')
        ax.scatter(z['begin_x'],z['begin_y'],c=c,s=48,zorder=6,edgecolors='white')

fig,axes=plt.subplots(1,2,figsize=(20,9))
setup(axes[0],'01  障碍调度范围与已保存采点');regions(axes[0])
for name,z,pts,dyn,c in routes:
    axes[0].annotate(f"({z['begin_x']:.2f}, {z['begin_y']:.2f})",(z['begin_x'],z['begin_y']),xytext=(5,9),textcoords='offset points',fontsize=8,color=c)
setup(axes[1],'02  各处理函数的目标点序列（示意）');regions(axes[1])
for name,z,pts,dyn,c in routes:
    for k,(a,b) in enumerate(zip(pts,pts[1:]),1):
        uncertain=dyn[k] or dyn[k-1]
        axes[1].plot([a[0],b[0]],[a[1],b[1]],color=c,lw=1.6,ls='--' if uncertain else '-')
        if abs(a[0]-b[0])+abs(a[1]-b[1])>.1:
            mid=((a[0]+b[0])/2,(a[1]+b[1])/2)
            axes[1].annotate('',xy=b,xytext=mid,arrowprops=dict(arrowstyle='->',color=c,lw=1.2))
        axes[1].scatter(*b,s=20,facecolors='white' if dyn[k] else c,edgecolors=c,zorder=6)
axes[1].legend(handles=[Line2D([0],[0],color='#334455',label='实线：固定目标间连线（非实际轨迹）'),Line2D([0],[0],ls='--',color='#334455',label='虚线：含动态坐标／参考点起始假设')],loc='lower left',fontsize=8)
fig.suptitle('旧方案场地对照图  |  A/B 赛道坐标，非 Nav2 map 坐标',fontsize=20,fontweight='bold',y=.96)
fig.text(.06,.12,'彩色矩形是程序触发区域，并非障碍实物尺寸。编号用于索引，不代表程序强制的全场执行顺序。',fontsize=11)
fig.text(.06,.08,'动态 x/y 按上一示意目标延续；跳跃位移、转向漂移及区域中途切换未模拟。当前默认 demo 仅运行两个测试点。',fontsize=10,color='#9c3f28')
fig.subplots_adjust(left=.06,right=.98,bottom=.20,top=.86,wspace=.15)
fig.savefig(OUT/'legacy_course_overview.png',dpi=180)
fig.savefig(OUT/'legacy_course_overview.pdf')
plt.close(fig)

fig,axes=plt.subplots(2,4,figsize=(20,11))
for ax,(name,z,pts,dyn,c) in zip(axes.flat,routes):
    ax.set_title(name,fontweight='bold',color=c)
    ax.add_patch(Rectangle((z['x_min'],z['y_min']),z['x_max']-z['x_min'],z['y_max']-z['y_min'],fc=c,ec=c,alpha=.12))
    for k,(a,b) in enumerate(zip(pts,pts[1:]),1):
        ax.plot([a[0],b[0]],[a[1],b[1]],color=c,ls='--' if dyn[k] or dyn[k-1] else '-')
        ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle='->',color=c))
        ax.scatter(*b,s=30,facecolors='white' if dyn[k] else c,edgecolors=c)
        ax.annotate(str(k),b,xytext=(5,5+(k%2)*7),textcoords='offset points',fontsize=9)
    ax.scatter(*pts[0],marker='*',s=100,c='black');ax.annotate('采点',pts[0],xytext=(5,-13),textcoords='offset points',fontsize=8)
    ax.invert_yaxis();ax.set_aspect('equal',adjustable='datalim');ax.margins(.2);ax.grid(alpha=.25);ax.set_xlabel('旧 x / 米');ax.set_ylabel('旧 y / 米')
fig.suptitle('各障碍航点细节  |  数字对应 CSV 的 step；重合序号可能为原地转向',fontsize=18)
fig.text(.05,.025,'实线仅表示固定目标间连线；虚线涉及动态坐标。不是实车轨迹或可直接执行的全场路线。',fontsize=11)
fig.tight_layout(rect=(0,.06,1,.95));fig.savefig(OUT/'legacy_course_details.png',dpi=170);plt.close(fig)

fig,ax=plt.subplots(figsize=(13,10));setup(ax,'03  主调度区域与 zon 辅助框差异',ylim=(5.5,-6.2));regions(ax)
for key,z in data['zon'].items():
    ax.add_patch(Rectangle((z['x_min'],z['y_min']),z['x_max']-z['x_min'],z['y_max']-z['y_min'],fill=False,ec='#555555',ls='--',lw=1))
    ax.text((z['x_min']+z['x_max'])/2,(z['y_min']+z['y_max'])/2,key,fontsize=9,ha='center')
fig.text(.12,.035,'灰色虚线为 zon 原始值，未替它修正正负号；当前 Quadruped_Obstacle 分派使用 zones / zone。',fontsize=10)
fig.tight_layout(rect=(0,.07,1,1));fig.savefig(OUT/'legacy_auxiliary_regions.png',dpi=160);plt.close(fig)

summary={'config_sha256':hashlib.sha256(config.read_bytes()).hexdigest(),'controller_sha256':hashlib.sha256(controller.read_bytes()).hexdigest(),'obstacles':len(routes),'waypoints':len(rows),'fixed_targets_in_other_dispatch':[r for r in rows if not r['dynamic_xy'] and r['dispatch_at_fixed_target']!=r['obstacle']]}
(OUT/'audit.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps({'obstacles':len(routes),'waypoints':len(rows),'cross_region_fixed_targets':len(summary['fixed_targets_in_other_dispatch']),'output':str(OUT)},ensure_ascii=False))
