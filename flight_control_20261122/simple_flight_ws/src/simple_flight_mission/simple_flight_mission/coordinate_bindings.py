"""Named survey coordinates shared by route loading; no ROS or control I/O."""
import hashlib
import json
import math
import os
from pathlib import Path


POINT_ORDER=('0','1.1','1.2','2.1','2.2','3','4.1','4.2','5.1','5.2')
DEFAULT_POINTS={
    '0':(0.,0.),'1.1':(1.7,0.),'1.2':(2.2,0.),'2.1':(1.7,1.4),'2.2':(2.2,1.4),
    '3':(0.,1.4),'4.1':(1.2,1.4),'4.2':(2.2,1.4),'5.1':(1.2,2.7),'5.2':(2.2,2.7),
}
DEFAULT_COORDINATES_PATH=Path.home()/'simple_flight_ws/calibration/coordinates.json'


def _unique_object(pairs):
    result={}
    for key,value in pairs:
        if key in result:raise ValueError('校准文件包含重复字段：'+key)
        result[key]=value
    return result


def read_coordinate_snapshot(path=None):
    """Read atomically saved data once; a missing default uses YAML defaults."""
    configured=os.environ.get('SIMPLE_FLIGHT_COORDINATES')
    explicit=path is not None or configured is not None
    selected=Path(path if path is not None else configured if configured is not None else DEFAULT_COORDINATES_PATH).expanduser().resolve()
    try:
        with selected.open('rb') as handle:raw=handle.read(262145)
    except FileNotFoundError as error:
        if not explicit:return None
        raise ValueError('指定校准文件不存在：'+str(selected)) from error
    except OSError as error:
        raise ValueError('无法读取校准文件：'+str(error)) from error
    try:
        if len(raw)>262144:raise ValueError('校准文件过大')
        data=json.loads(raw.decode('utf-8-sig'),object_pairs_hook=_unique_object)
        if not isinstance(data,dict) or type(data.get('schema_version')) is not int or data['schema_version']!=1:
            raise ValueError('校准文件版本错误')
        if (data.get('units')!='m' or data.get('axes')!='X_forward_Y_left_fixed_origin_heading'
                or data.get('point_order')!=list(POINT_ORDER)):
            raise ValueError('校准文件单位、点名或坐标轴约定错误')
        if data.get('status') not in ('manual','complete','defaults'):
            raise ValueError('校准文件未完成，不能用于飞行')
        if data.get('method')=='automatic' and data.get('status')!='complete':
            raise ValueError('自动校准必须完整完成')
        points=data.get('points')
        if not isinstance(points,dict) or set(points)!=set(POINT_ORDER):raise ValueError('校准文件缺少点位')
        for name,xy in points.items():
            if (not isinstance(xy,list) or len(xy)!=2
                    or not all(type(v) in (int,float) and math.isfinite(v) for v in xy)
                    or math.hypot(*xy)>100):raise ValueError('点 '+name+' 坐标无效')
        if tuple(points['0'])!=(0.,0.):raise ValueError('原点0必须为(0,0)')
    except (ValueError,UnicodeError) as error:
        raise ValueError('校准参数无效，拒绝使用旧坐标继续：'+str(error)) from error
    # Tuples are copied out of the parsed document so a task keeps its snapshot.
    return {name:tuple(points[name]) for name in POINT_ORDER},str(selected),hashlib.sha256(raw).hexdigest()
