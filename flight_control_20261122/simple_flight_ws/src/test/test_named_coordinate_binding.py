"""Saved survey points bind flight routes/search/return by name, not magnitude."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import pytest

from simple_flight_mission.route_config import load_route_config


ROOT=Path(__file__).resolve().parents[1]
NAMES={1:('1.2','2.2','3','4.2','5.2'),2:('1.2','2.2','3','4.1','5.1'),
       3:('1.1','2.1','3','4.2','5.2'),4:('1.1','2.1','3','4.1','5.1')}
POINTS={'0':[0.,0.],'1.1':[1.61,.02],'1.2':[2.17,-.03],
        '2.1':[1.66,1.36],'2.2':[2.15,1.42],'3':[.03,1.39],
        '4.1':[1.13,1.43],'4.2':[2.24,1.48],'5.1':[1.09,2.65],'5.2':[2.31,2.74]}


def document(points=None):
    return dict(schema_version=1,units='m',axes='X_forward_Y_left_fixed_origin_heading',status='manual',
                method='manual',point_order=list(POINTS),points=deepcopy(points or POINTS),reference=None,measurements={})


@pytest.fixture
def saved(tmp_path):
    p=tmp_path/'coordinates.json';p.write_text(json.dumps(document()),encoding='utf-8');return p


@pytest.mark.parametrize('number',[1,2,3,4])
def test_routes_bind_every_point_and_opposite_search_end_from_one_file(number,saved):
    cfg=load_route_config(ROOT/f'config/route_{number}.yaml',coordinate_file=saved)
    assert cfg.point_names==NAMES[number]
    assert [(p.forward_m,p.left_m) for p in cfg.points]==[tuple(POINTS[n]) for n in NAMES[number]]
    opposite='5.1' if number in (1,3) else '5.2'
    assert cfg.search_point_name==opposite and cfg.search_xy==tuple(POINTS[opposite])
    assert cfg.coordinate_sha256==hashlib.sha256(saved.read_bytes()).hexdigest()
    assert cfg.coordinate_source==str(saved.resolve())
    assert cfg.return_indices==(3,1,0)
    assert tuple(cfg.point_names[i] for i in cfg.return_indices)==(NAMES[number][3],NAMES[number][1],NAMES[number][0])


def test_coincident_stage_names_remain_independent_on_route_one(saved):
    cfg=load_route_config(ROOT/'config/route_1.yaml',coordinate_file=saved)
    assert cfg.points[1]!=cfg.points[3]
    assert cfg.point_names[1]=='2.2' and cfg.point_names[3]=='4.2'


def test_running_route_snapshot_does_not_change_when_the_survey_file_changes(saved):
    path=ROOT/'config/route_1.yaml'
    old=load_route_config(path,coordinate_file=saved)
    newer=document();newer['points']['1.2']=[2.3,.04];saved.write_text(json.dumps(newer))
    new=load_route_config(path,coordinate_file=saved)
    assert old.points[0].forward_m==2.17 and new.points[0].forward_m==2.3
    assert old.coordinate_sha256!=new.coordinate_sha256


@pytest.mark.parametrize('error',['origin','missing','nan','bool','units','partial','axes'])
def test_invalid_saved_file_is_rejected_instead_of_silently_using_old_routes(saved,error):
    data=document()
    if error=='origin':data['points']['0']=[.1,0.]
    if error=='missing':del data['points']['4.2']
    if error=='nan':data['points']['1.1']=[math.nan,0.]
    if error=='bool':data['points']['1.1']=[True,0.]
    if error=='units':data['units']='cm'
    if error=='partial':data.update(method='automatic',status='partial')
    if error=='axes':data['axes']='world_xy'
    saved.write_text(json.dumps(data))
    with pytest.raises(ValueError):load_route_config(ROOT/'config/route_1.yaml',coordinate_file=saved)


def test_duplicate_coordinate_keys_are_rejected(saved):
    saved.write_text(json.dumps(document()).replace('"schema_version": 1','"schema_version": 1, "schema_version": 2'))
    with pytest.raises(ValueError):load_route_config(ROOT/'config/route_1.yaml',coordinate_file=saved)


def test_explicit_missing_file_is_an_error_and_generic_routes_stay_independent(tmp_path):
    with pytest.raises(ValueError):load_route_config(ROOT/'config/route_1.yaml',coordinate_file=tmp_path/'missing.json')
    simple=tmp_path/'custom.yaml';simple.write_text('points: [{forward_m: 0.7, left_m: 0.2}]\n')
    cfg=load_route_config(simple)
    assert cfg.points[0].forward_m==.7 and cfg.point_names==()


def test_default_yamls_use_the_calculated_coordinates_when_no_saved_file(monkeypatch,tmp_path):
    import simple_flight_mission.coordinate_bindings as binding
    monkeypatch.setattr(binding,'DEFAULT_COORDINATES_PATH',tmp_path/'absent.json')
    monkeypatch.delenv('SIMPLE_FLIGHT_COORDINATES',raising=False)
    for route in (1,2,3,4):
        cfg=load_route_config(ROOT/f'config/route_{route}.yaml')
        assert [(p.forward_m,p.left_m) for p in cfg.points]==[binding.DEFAULT_POINTS[n] for n in NAMES[route]]
        assert cfg.coordinate_sha256==''


def test_short_flight_endpoint_uses_the_same_saved_1_2_point(saved):
    cfg=load_route_config(ROOT/'config/route_smoke.yaml',coordinate_file=saved)
    assert cfg.point_names==('1.2',)
    assert (cfg.points[0].forward_m,cfg.points[0].left_m)==tuple(POINTS['1.2'])


def test_search_endpoint_counts_toward_configured_operating_radius(saved):
    data=document();data['points']['5.1']=[4.,3.];saved.write_text(json.dumps(data))
    cfg=load_route_config(ROOT/'config/route_1.yaml',coordinate_file=saved)
    assert cfg.core_config().max_horizontal_m>=5.+cfg.path_margin_m


@pytest.mark.parametrize('route',[1,2,3,4])
def test_competition_search_transforms_both_calibrated_axes_with_initial_heading(saved,monkeypatch,route):
    from test import test_rescue_aisle_workflow as workflow
    cfg=load_route_config(ROOT/f'config/route_{route}.yaml',coordinate_file=saved)
    monkeypatch.setattr(workflow,'load_route_config',lambda _:cfg)
    rig=workflow.RescueRig(route,yaw=.8);rig.reach_rescue()
    actual=rig.controller.task.scan_points[-1]
    expected=rig.world(*cfg.search_xy)
    assert actual[:2]==pytest.approx(expected)
    assert math.dist(actual[:2],rig.controller.world_route[4][:2])>.5


def test_return_visits_distinct_4_2_before_2_2(saved):
    from test.test_route_flight import route_rig,follow
    cfg=load_route_config(ROOT/'config/route_1.yaml',coordinate_file=saved)
    rig=route_rig(cfg);rig.active();sequence=[]
    for _ in range(5000):
        out=follow(rig)[0]
        if out.phase=='RETURN' and (not sequence or sequence[-1]!=out.point_index):sequence.append(out.point_index)
        if out.phase=='DESCEND':break
    assert sequence==[3,1,0,None]
