#!/usr/bin/env python3
"""Independent ground coordinate survey. ROS subscriptions only; no flight I/O."""
import argparse
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import shutil
from statistics import median
import sys
import threading
import time
import uuid


POINT_ORDER = ('0','1.1','1.2','2.1','2.2','3','4.1','4.2','5.1','5.2')
DEFAULT_POINTS = {
    '0':(0.,0.), '1.1':(1.7,0.), '1.2':(2.2,0.),
    '2.1':(1.7,1.4), '2.2':(2.2,1.4), '3':(0.,1.4),
    '4.1':(1.2,1.4), '4.2':(2.2,1.4), '5.1':(1.2,2.7), '5.2':(2.2,2.7),
}
POINT_DESCRIPTIONS = {
    '0':'起飞原点', '1.1':'第一通道近端', '1.2':'第一通道远端',
    '2.1':'危险品区前近端', '2.2':'危险品区前远端', '3':'危险品区参考点',
    '4.1':'第二通道近端', '4.2':'第二通道远端（单独记录）',
    '5.1':'人员搜索通道近端', '5.2':'人员搜索通道远端',
}
POSE_TOPIC = '/mavros/local_position/pose'
DEFAULT_OUTPUT = Path.home()/'simple_flight_ws/calibration/coordinates.json'


class CalibrationError(ValueError):
    pass


class ReferenceChangedError(CalibrationError):
    pass


@dataclass(frozen=True)
class PoseSample:
    position: tuple
    yaw: float
    frame: str
    received: float
    stamp_ns: int


@dataclass(frozen=True)
class Capture:
    position: tuple
    yaw: float
    frame: str
    count: int
    span_s: float
    spread_m: float
    boot_id: str


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def valid_xy(xy):
    return isinstance(xy,(list,tuple)) and len(xy)==2 and all(finite(v) for v in xy) and math.hypot(*xy)<=100.


def relative_xy(origin, point):
    if origin.frame != point.frame or origin.boot_id != point.boot_id:
        raise ReferenceChangedError('定位参考已改变，本轮不能继续；请回原点重新开始。')
    dx, dy = (point.position[i]-origin.position[i] for i in (0,1))
    c, s = math.cos(origin.yaw), math.sin(origin.yaw)
    return c*dx+s*dy, -s*dx+c*dy


def aggregate_samples(rows, started, boot_id):
    if len(rows)<6 or rows[-1].received-rows[0].received<.4:
        raise CalibrationError('新定位数据不足，请保持静止后重新按Enter。')
    if any(p.received<started or not p.frame or not all(finite(v) for v in (*p.position,p.yaw,p.received)) for p in rows):
        raise CalibrationError('采样包含旧数据或无效坐标，请重试。')
    if len({p.frame for p in rows})!=1:
        raise ReferenceChangedError('采样期间坐标系改变，请回原点重新开始。')
    if any(b.stamp_ns<=a.stamp_ns for a,b in zip(rows,rows[1:])):
        raise CalibrationError('定位时间戳冻结或倒退，不能使用该次测量。')
    ranges=[max(p.position[i] for p in rows)-min(p.position[i] for p in rows) for i in range(3)]
    spread=math.hypot(*ranges[:2])
    yaw=math.atan2(sum(math.sin(p.yaw) for p in rows),sum(math.cos(p.yaw) for p in rows))
    if spread>.03 or ranges[2]>.04 or any(abs(math.remainder(p.yaw-yaw,2*math.pi))>math.radians(3) for p in rows):
        raise CalibrationError('采样时仍在移动或转动，请放稳后重新按Enter。')
    return Capture(tuple(median(p.position[i] for p in rows) for i in range(3)), yaw,
                   rows[0].frame,len(rows),rows[-1].received-rows[0].received,spread,boot_id)


def default_document():
    return dict(schema_version=1,units='m',axes='X_forward_Y_left_fixed_origin_heading',
                status='defaults',method='defaults',point_order=list(POINT_ORDER),
                points={name:list(DEFAULT_POINTS[name]) for name in POINT_ORDER},
                reference=None,measurements={})


def validate_document(document):
    if not isinstance(document,dict) or document.get('schema_version')!=1:
        raise CalibrationError('校准文件版本或格式错误，原文件不会被覆盖。')
    points=document.get('points')
    if (document.get('units')!='m' or document.get('axes')!='X_forward_Y_left_fixed_origin_heading'
            or document.get('point_order')!=list(POINT_ORDER)
            or not isinstance(points,dict) or set(points)!=set(POINT_ORDER)):
        raise CalibrationError('校准文件必须包含全部10个点，单位为米。')
    for name,xy in points.items():
        if not valid_xy(xy):
            raise CalibrationError('点 '+name+' 的坐标无效。')
    if not isinstance(document.get('measurements'),dict):
        raise CalibrationError('校准文件测量记录格式错误。')
    if tuple(points['0'])!=(0.,0.):
        raise CalibrationError('原点0必须保持(0,0)。')
    return document


def load_document(path):
    if not path.exists():
        return default_document()
    try:
        return validate_document(json.loads(path.read_text(encoding='utf-8')))
    except (OSError,json.JSONDecodeError) as error:
        raise CalibrationError('无法读取校准文件：'+str(error)) from error


def save_document(path,document):
    validate_document(document)
    path=Path(path).expanduser().resolve()
    workspace=(Path.home()/'simple_flight_ws').resolve()
    if path.suffix.lower()!='.json' or any(path.is_relative_to(workspace/name) for name in ('src','config','install','build')):
        raise CalibrationError('结果只能写独立JSON文件，不能写入飞行配置、源码或安装目录。')
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():
        backup=path.with_name(path.stem+'.backup.'+uuid.uuid4().hex[:8]+'.json')
        shutil.copy2(path,backup)
    temp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        temp.write_text(json.dumps(document,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


def render_map(points,current=None,completed=(),width=None):
    """X forward is up, Y left is left; collocated point names stay visible."""
    width=max(56,min(width or shutil.get_terminal_size((96,30)).columns,110))
    cols=width-9;rows=19
    xs=[xy[0] for xy in points.values()];ys=[xy[1] for xy in points.values()]
    xmin=min(0.,*xs)-.15;xmax=max(.5,*xs)+.15
    ymin=min(0.,*ys)-.15;ymax=max(.5,*ys)+.15
    def cell(x,y):
        return round((xmax-x)/(xmax-xmin)*(rows-1)),round((ymax-y)/(ymax-ymin)*(cols-1))
    grid=[[' ']*cols for _ in range(rows)]
    xr,yc=cell(0.,0.)
    for c in range(cols):grid[xr][c]='-'
    for r in range(rows):grid[r][yc]='|'
    groups={}
    for name in POINT_ORDER:
        xy=tuple(round(v,4) for v in points[name]);groups.setdefault(xy,[]).append(name)
    occupied=set()
    for xy,names in groups.items():
        r,c=cell(*xy);grid[r][c]='*' if current in names else ('+' if any(n in completed for n in names) else 'o')
        occupied.add((r,c))
    overflow=[]
    for xy,names in groups.items():
        r,c=cell(*xy)
        label='/'.join(('*' if n==current else '')+n for n in names)+f'({xy[0]:.2f},{xy[1]:.2f})'
        candidates=[(rr,cc) for rr in (r,r-1,r+1,r-2,r+2) for cc in (c+2,c-len(label)-1,max(0,min(c,cols-len(label))))]
        for rr,cc in candidates:
            if 0<=rr<rows and 0<=cc and cc+len(label)<=cols and all((rr,k) not in occupied for k in range(cc,cc+len(label))):
                for k,ch in enumerate(label,cc):grid[rr][k]=ch;occupied.add((rr,k))
                break
        else:overflow.append(label)
    result=['X+ 前 ↑    单位：米；* 当前点，+ 已采样，o 已有/默认点', '       '+'+'+'-'*cols+'+']
    for r,line in enumerate(grid):
        tick=f'{xmax-r/(rows-1)*(xmax-xmin):5.2f}' if r%3==0 else '     '
        result.append(tick+' |'+''.join(line)+'|')
    result.extend(['       '+'+'+'-'*cols+'+','         ← Y+ 左；原点朝向固定，后续转机身不会改变坐标轴'])
    result.extend(overflow)
    return '\n'.join(result)


def render_table(document):
    lines=['点名      X前/m     Y左/m     说明']
    for name in POINT_ORDER:
        x,y=document['points'][name]
        source=(document.get('measurements',{}).get(name) or {}).get('method','已有/默认')
        lines.append(f'{name:<5}  {x:>9.3f} {y:>9.3f}   {POINT_DESCRIPTIONS[name]} [{source}]')
    return '\n'.join(lines)


def parse_xy(text):
    parts=re.sub(r'[(),，（）]',' ',text).split()
    try:xy=tuple(float(part) for part in parts)
    except ValueError as error:raise CalibrationError('请输入两个有限数字，例如：1.70 0 或 1.70,0。') from error
    if not valid_xy(xy):
        raise CalibrationError('请输入两个有限数字，本场地坐标需在原点100米内，例如：1.70 0。')
    return list(xy)


def manual_mode(document,path,ask=input,say=print):
    working=deepcopy(document)
    while True:
        say(render_map(working['points']));say(render_table(working))
        choice=ask('选择点名修改；r=恢复全部计算默认值；s=保存并返回；q=取消本次修改：').strip().lower()
        if choice in ('q','退出'):return None
        if choice in ('r','默认'):
            working=default_document();say('已恢复计算后的默认值；输入s才保存。');continue
        if choice in ('s','保存'):
            working.update(method='manual',status='manual')
            save_document(path,working);say('已保存独立校准文件：'+str(path));return working
        if choice=='0':say('原点0固定为(0,0)，不能修改。');continue
        if choice not in POINT_ORDER:say('点名无效，请输入例如1.1、2.2、4.2。');continue
        while True:
            text=ask('点 '+choice+'，输入 X Y（米），或q取消此点：').strip()
            if text.lower()=='q':break
            try:xy=parse_xy(text)
            except CalibrationError as error:say(str(error));continue
            working['points'][choice]=xy
            working.setdefault('measurements',{})[choice]=dict(method='manual')
            say(f'点 {choice} 已修改：X={xy[0]:.3f}m，Y={xy[1]:.3f}m。')
            break


def automatic_mode(document,path,reader,ask=input,say=print):
    working=deepcopy(document);working['measurements']={};completed=set();origin=None
    say('自动校准是人工移动飞机、工具只读测量；不会起飞或驱动电机。')
    say('保持落地锁桨、机身水平。原点处机头指向场地前方X+；后续尽量保持相同摆放姿态。')
    say('未采样点显示已有/默认位置，仅作位置提示。中途q取消时保留原文件。')
    for name in POINT_ORDER:
        while True:
            say(render_map(working['points'],name,completed))
            label=name+'-'+POINT_DESCRIPTIONS[name]
            choice=ask('请将无人机放到 '+label+'，放稳后按Enter采样（q取消本轮）：').strip().lower()
            if choice=='q':say('本轮已取消，原校准文件保持。');return None
            if choice:say('请在放稳后直接按Enter，或输入q取消。');continue
            say('采样中，请保持静止，正常约0.6秒……')
            try:
                measured=reader.capture()
                if name=='0':origin=measured;xy=(0.,0.)
                else:xy=relative_xy(origin,measured)
                if not valid_xy(xy):raise CalibrationError('测量超出本场地坐标范围，请检查定位。')
            except ReferenceChangedError:
                raise
            except CalibrationError as error:
                say('本点未采纳：'+str(error));continue
            working['points'][name]=list(xy)
            working['measurements'][name]=dict(method='automatic',world_position_m=list(measured.position),
                heading_rad=measured.yaw,sample_count=measured.count,span_s=measured.span_s,spread_m=measured.spread_m)
            if name=='0':
                working['reference']=dict(position_m=list(origin.position),yaw_rad=origin.yaw,
                    frame_id=origin.frame,boot_id=origin.boot_id,pose_topic=POSE_TOPIC)
            completed.add(name)
            say(f'点 {name} 校准成功：X={xy[0]:.3f}m，Y={xy[1]:.3f}m；{measured.count}个新样本，水平变化范围{measured.spread_m*100:.1f}cm。')
            break
    working.update(status='complete',method='automatic')
    save_document(path,working)
    say('全部点校准完成。');say(render_map(working['points'],completed=completed));say(render_table(working))
    say('已保存独立校准文件：'+str(path));say('下次启动命名航线将读取本文件；本工具不发送飞行命令。')
    return working


class RosPoseReader:
    """Read-only subscribers; acquire existing mission locks during surveying."""
    def __init__(self):
        self.lock=threading.Lock();self.rows=deque(maxlen=400);self.observations={}
        self.error=None;self.node=None;self.thread=None;self.handles=[];self.initialized=False
        self.stop=threading.Event();self.last_stamp=None;self.frame=None
        self.previous_pose=None;self.lio_stamp=None

    def __enter__(self):
        try:
            import fcntl
            log=Path.home()/'simple_flight_ws/log';log.mkdir(parents=True,exist_ok=True)
            for name in ('competition.lock','smoke.lock'):
                handle=(log/name).open('a+');self.handles.append(handle)
                try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError as error:raise CalibrationError('任务进程占用中，请结束任务后再做地面校准。') from error
            for path in Path('/proc').glob('[0-9]*/cmdline'):
                try:command=path.read_bytes()
                except OSError:continue
                if any(token in command for token in (b'simple_flight_mission.competition_node',b'simple_flight_mission.route_node',b'/hover_control',b'/waypoint_mission')):
                    raise CalibrationError('仍有飞行任务进程，不能同时移动飞机做校准。')
            import rclpy
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.signals import SignalHandlerOptions
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Odometry
            from mavros_msgs.msg import State,ExtendedState,EstimatorStatus
            self.rclpy=rclpy
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO);self.initialized=True
            self.node=rclpy.create_node('coordinate_calibration_readonly')
            self.boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            self.subscriptions=[self.node.create_subscription(PoseStamped,POSE_TOPIC,self._pose,qos_profile_sensor_data)]
            for key,topic,typ in [('state','/mavros/state',State),('landed','/mavros/extended_state',ExtendedState),
                                   ('estimator','/mavros/estimator_status',EstimatorStatus),('lio','/Odometry',Odometry)]:
                self.subscriptions.append(self.node.create_subscription(typ,topic,lambda m,k=key:self._observation(k,m),qos_profile_sensor_data))
            self.thread=threading.Thread(target=self._spin,daemon=True);self.thread.start()
            return self
        except Exception:
            self.close();raise

    def _spin(self):
        try:
            while not self.stop.is_set():self.rclpy.spin_once(self.node,timeout_sec=.1)
        except Exception as error:
            with self.lock:self.error=CalibrationError('定位订阅中断：'+str(error))

    def _observation(self,key,message):
        stamp=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if stamp<=0 or not -.1<=(self.node.get_clock().now().nanoseconds-stamp)/1e9<=.5:return
        with self.lock:
            if key=='lio':
                if self.lio_stamp is not None and stamp<self.lio_stamp:
                    self.error=ReferenceChangedError('雷达定位时间重置，请回原点重新开始。');return
                if stamp==self.lio_stamp:return
                self.lio_stamp=stamp
            self.observations[key]=(time.monotonic(),message)

    def _pose(self,message):
        p=message.pose.position;q=message.pose.orientation
        stamp=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        quaternion=(q.x,q.y,q.z,q.w)
        if not all(finite(v) for v in (p.x,p.y,p.z,*quaternion)) or not .99<=sum(v*v for v in quaternion)<=1.01:return
        if stamp<=0 or not -.1<=(self.node.get_clock().now().nanoseconds-stamp)/1e9<=.5:return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            if self.last_stamp is not None and (stamp<self.last_stamp or message.header.frame_id!=self.frame):
                self.error=ReferenceChangedError('定位时间或坐标系重置，请回原点重新开始。');return
            if stamp==self.last_stamp:return
            if self.previous_pose is not None:
                old_stamp,old_position=self.previous_pose
                if math.dist((p.x,p.y,p.z),old_position)>.20+3.*(stamp-old_stamp)/1e9:
                    self.error=ReferenceChangedError('定位出现突跳或搬动过快，本轮终止，请回原点重校。');return
            self.last_stamp=stamp;self.frame=message.header.frame_id
            self.previous_pose=(stamp,(p.x,p.y,p.z))
            self.rows.append(PoseSample((p.x,p.y,p.z),yaw,self.frame,time.monotonic(),stamp))

    def capture(self):
        started=time.monotonic();deadline=started+3.
        while True:
            now=time.monotonic()
            with self.lock:
                if self.error:raise self.error
                obs=dict(self.observations);rows=[p for p in self.rows if p.received>=started]
            ready=all(k in obs and now-obs[k][0]<age for k,age in [('state',2.),('landed',.6),('estimator',.6),('lio',.4)])
            if 'state' in obs and (not obs['state'][1].connected or obs['state'][1].armed):
                raise CalibrationError('飞控必须已连接、落地且未解锁。')
            if 'landed' in obs and obs['landed'][1].landed_state!=1:
                raise CalibrationError('飞控尚未确认落地，请放稳后重试。')
            if ready:
                estimator=obs['estimator'][1]
                if not (estimator.pos_horiz_abs_status_flag or estimator.pos_horiz_rel_status_flag) or not estimator.attitude_status_flag:
                    raise CalibrationError('飞控位置或姿态估计无效，请恢复定位后再测。')
                if now-started>=.6 and rows and now-rows[-1].received<=.25:
                    return aggregate_samples(rows,started,self.boot_id)
            if now>=deadline:raise CalibrationError('定位/状态数据不新鲜或不足，请检查雷达定位后重试。')
            time.sleep(.02)

    def close(self):
        self.stop.set()
        if self.thread:self.thread.join(timeout=2.)
        if self.node:self.node.destroy_node();self.node=None
        if self.initialized:self.rclpy.shutdown();self.initialized=False
        for handle in self.handles:handle.close()
        self.handles=[]

    def __exit__(self,*unused):self.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description='独立坐标校准：自动只读测量/直接输入；不修改飞行航线。')
    parser.add_argument('--output',type=Path,default=DEFAULT_OUTPUT,help='独立JSON结果文件')
    parser.add_argument('--show',action='store_true',help='仅显示已有坐标/默认地图，不连接ROS')
    args=parser.parse_args(argv)
    args.output=args.output.expanduser().resolve()
    try:
        document=load_document(args.output)
        if args.show:
            print(render_map(document['points']));print(render_table(document));return 0
        print('坐标校准工具：0固定(0,0)，X前/Y左，单位米。')
        print('保存位置：'+str(args.output));print('保存后下次启动命名航线会读取；正在运行的任务保持原参数。')
        while True:
            choice=input('\n选择校准方式：1=自动校准（人工逐点摆放）；2=坐标直接输入；v=查看；q=退出：').strip().lower()
            if choice=='q':return 0
            if choice=='v':print(render_map(document['points']));print(render_table(document));continue
            if choice=='2':result=manual_mode(document,args.output)
            elif choice=='1':
                if not sys.stdin.isatty():print('自动逐点采样需要交互终端。');continue
                try:
                    with RosPoseReader() as reader:result=automatic_mode(document,args.output,reader)
                except (CalibrationError,ImportError,OSError) as error:
                    print('自动校准未完成：'+str(error));continue
            else:print('请输入1、2、v或q。');continue
            if result is not None:document=result
    except (EOFError,KeyboardInterrupt):
        print('\n已退出；未完成/未保存的修改不会写入文件。');return 0
    except (CalibrationError,OSError) as error:
        print('错误：'+str(error),file=sys.stderr);return 2


if __name__=='__main__':raise SystemExit(main())
