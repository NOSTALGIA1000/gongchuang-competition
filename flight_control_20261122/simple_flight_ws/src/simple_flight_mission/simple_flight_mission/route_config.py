"""Relative waypoint configuration. Loading/previewing never imports ROS."""
from dataclasses import dataclass, fields, replace
import math
from pathlib import Path

from .hover_controller import HoverConfig


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class RoutePoint:
    forward_m: float
    left_m: float = 0.0
    hold_s: float = 3.0
    stage: str = "waypoint"

    def __post_init__(self):
        if (not all(_number(x) for x in (self.forward_m, self.left_m, self.hold_s))
                or self.hold_s < 0 or not isinstance(self.stage, str) or not self.stage.strip()):
            raise ValueError("航点坐标必须为有限数，hold_s 必须非负；不接受布尔值")


@dataclass(frozen=True)
class RouteConfig:
    height_m: float = 1.0
    climb_mps: float = 0.2
    horizontal_mps: float = 0.2
    descent_source: str = "fcu"
    origin_hold_s: float = 3.0
    return_hold_s: float = 2.0
    arrival_xy_m: float = 0.15
    arrival_z_m: float = 0.10
    arrival_speed_mps: float = 0.10
    arrival_stable_s: float = 1.0
    path_margin_m: float = 0.40
    target_lead_m: float = 0.25
    transit_timeout_extra_s: float = 10.0
    landing_timeout_extra_s: float = 10.0
    mission_timeout_s: float = 300.0
    return_indices: tuple = ()
    points: tuple = (RoutePoint(1.0),)
    point_names: tuple = ()
    search_point_name: str = ""
    search_xy: tuple = ()
    coordinate_source: str = "route_yaml"
    coordinate_sha256: str = ""
    # Keep the aircraft at a scoring waypoint for its configured hold_s after
    # the task finishes, instead of departing immediately.
    zone_dwell_after_task: bool = False
    # If the hazardous target is never recognised, skip that drop and keep
    # flying the rest of the route instead of aborting the mission.
    continue_after_target_not_found: bool = False
    # Scored release: keep the annotation visible this long before opening the
    # door. Zero opens on the first centered frame (historical behaviour).
    mark_hold_s: float = 0.
    # Upper bound on waiting for that mark, so a zone can never hang the sortie.
    mark_max_wait_s: float = 12.
    # Constant-ratio setpoint-lead taper into a task-zone waypoint. With
    # position-only OFFBOARD control the travel speed is the PX4 position gain
    # times this lead, so shrinking the lead is what actually brakes. None
    # disables the taper and keeps the fixed target_lead_m.
    zone_approach_taper: float | None = None
    zone_approach_min_lead_m: float | None = None

    def __post_init__(self):
        for item in fields(self):
            value = getattr(self, item.name)
            if item.name not in ("points", "return_indices", "descent_source", "point_names", "search_point_name",
                                 "search_xy", "coordinate_source", "coordinate_sha256",
                                 "zone_dwell_after_task", "continue_after_target_not_found",
                                 "mark_hold_s", "mark_max_wait_s",
                                 "zone_approach_taper", "zone_approach_min_lead_m") and (
                    not _number(value) or value <= 0):
                raise ValueError(item.name + " 必须为有限正数")
        if self.descent_source != "fcu":
            raise ValueError("下降速度必须读取飞控参数：descent_source: fcu")
        if not isinstance(self.points, tuple) or not self.points or not all(isinstance(p, RoutePoint) for p in self.points):
            raise ValueError("points 必须是非空航点列表")
        if (not isinstance(self.return_indices, tuple)
                or not all(type(i) is int and 0 <= i < len(self.points) for i in self.return_indices)):
            raise ValueError("return_indices 必须是航点索引元组")
        if any(math.hypot(p.forward_m, p.left_m) > 100 for p in self.points):
            raise ValueError("本入口仅接受起飞点 100 米内的相对航点；实际可飞范围需现场另行验证")
        from .coordinate_bindings import POINT_ORDER
        if (not isinstance(self.point_names,tuple) or self.point_names and
                (len(self.point_names)!=len(self.points) or any(n not in POINT_ORDER for n in self.point_names))):
            raise ValueError("point_names必须与各航点一一对应")
        if (not isinstance(self.search_point_name,str) or self.search_point_name and self.search_point_name not in POINT_ORDER
                or not isinstance(self.search_xy,tuple) or self.search_point_name and not self.search_xy):
            raise ValueError("人员搜索端点配置无效")
        if self.search_xy and (len(self.search_xy)!=2 or not all(_number(v) for v in self.search_xy) or math.hypot(*self.search_xy)>100):
            raise ValueError("人员搜索端点必须是原点100米内的二维坐标")
        if not isinstance(self.coordinate_source,str) or not isinstance(self.coordinate_sha256,str):
            raise ValueError("坐标来源必须是字符串")
        # Preserve the hover stage's height and vertical/horizontal speed limits.
        HoverConfig(height_m=self.height_m, climb_mps=self.climb_mps)
        if self.horizontal_mps > .3:
            raise ValueError("本阶段水平速度不得超过 0.3 m/s")
        if type(self.zone_dwell_after_task) is not bool:
            raise ValueError("任务区停留开关必须为布尔值")
        if type(self.continue_after_target_not_found) is not bool:
            raise ValueError("识别失败继续任务开关必须为布尔值")
        if not _number(self.mark_hold_s) or self.mark_hold_s < 0 or self.mark_hold_s > 10:
            raise ValueError("标注停留时间必须为 0..10 秒")
        if not _number(self.mark_max_wait_s) or not self.mark_hold_s < self.mark_max_wait_s <= 30:
            raise ValueError("标注等待上限必须大于停留时间且不超过 30 秒")
        taper = (self.zone_approach_taper, self.zone_approach_min_lead_m)
        if any(v is not None for v in taper):
            if not all(_number(v) and v > 0 for v in taper):
                raise ValueError("进场减速的两项必须同时配置为有限正数")
            if not 0 < self.zone_approach_taper <= 1:
                raise ValueError("进场超前收缩比例必须在 (0, 1] 内")
            if self.zone_approach_min_lead_m >= self.target_lead_m:
                raise ValueError("进场最小超前量必须小于常规超前量")
        if self.arrival_xy_m >= self.path_margin_m or self.target_lead_m >= self.path_margin_m:
            raise ValueError("到点容差和目标超前距离必须小于航线保护余量")
        if self.arrival_z_m >= self.height_m / 2:
            raise ValueError("高度到点容差必须小于目标高度的一半")

    def core_config(self):
        radius = max([math.hypot(p.forward_m, p.left_m) for p in self.points]
                     + ([math.hypot(*self.search_xy)] if self.search_xy else []))
        return replace(HoverConfig(), height_m=self.height_m, climb_mps=self.climb_mps,
                       max_horizontal_m=max(.6, radius + self.path_margin_m))

    def world_points(self, origin):
        x, y, z, yaw = origin
        c, s = math.cos(yaw), math.sin(yaw)
        return tuple((x + p.forward_m*c - p.left_m*s,
                      y + p.forward_m*s + p.left_m*c, z + self.height_m, yaw)
                     for p in self.points)


def load_route_config(path=None, coordinate_file=None):
    if path is None:
        return RouteConfig()
    import yaml

    class UniqueLoader(yaml.SafeLoader):
        pass

    def mapping(loader, node):
        result = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node)
            if not isinstance(key, str) or key in result:
                raise ValueError("配置字段必须为不重复的字符串")
            result[key] = loader.construct_object(value_node, deep=True)
        return result

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    try:
        value = yaml.load(Path(path).read_text(encoding="utf-8"), Loader=UniqueLoader)
    except yaml.YAMLError as error:
        raise ValueError("YAML 格式错误：" + str(error)) from error
    allowed = {f.name for f in fields(RouteConfig)} - {"search_xy", "coordinate_source", "coordinate_sha256"}
    if not isinstance(value, dict) or not set(value) <= allowed:
        raise ValueError("配置必须为 RouteConfig 支持的字段，不接受未知字段")
    if "points" in value:
        points = value["points"]
        if not isinstance(points, list) or not points or not all(isinstance(p, dict) for p in points):
            raise ValueError("points 必须为非空航点列表")
        value["points"] = tuple(RoutePoint(**p) for p in points)
    if "return_indices" in value:
        indices = value["return_indices"]
        if not isinstance(indices, list):
            raise ValueError("return_indices 必须为列表")
        value["return_indices"] = tuple(indices)
    if "point_names" in value:
        names=value["point_names"]
        from .coordinate_bindings import POINT_ORDER,DEFAULT_POINTS,read_coordinate_snapshot
        if (not isinstance(names,list) or len(names)!=len(value.get("points",()))
                or not names or any(type(n) is not str or n not in POINT_ORDER for n in names)):
            raise ValueError("point_names必须使用字符串点名并对应每个航点")
        value["point_names"]=tuple(names)
        snapshot=read_coordinate_snapshot(coordinate_file)
        if snapshot is not None:
            named,source,digest=snapshot
            value["points"]=tuple(replace(point,forward_m=named[name][0],left_m=named[name][1])
                                  for name,point in zip(names,value["points"]))
            value.update(coordinate_source=source,coordinate_sha256=digest)
        else:
            named=DEFAULT_POINTS
            value["coordinate_source"]=str(Path(path).resolve())+" [yaml-defaults]"
        search_name=value.get("search_point_name","")
        if search_name:
            if search_name not in POINT_ORDER:raise ValueError("人员搜索点名无效")
            value["search_xy"]=named[search_name]
    return RouteConfig(**value)
