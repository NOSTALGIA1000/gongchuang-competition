"""Ground-only coordinate calibration; no ROS/device required for these tests."""
import importlib.util
import json
import math
from pathlib import Path
import sys

import pytest


@pytest.fixture
def tool():
    path = Path(__file__).parents[1]/'scripts/coordinate_calibration.py'
    spec = importlib.util.spec_from_file_location('coordinate_calibration_tool', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def capture(tool, x, y, yaw=0., frame='odom'):
    return tool.Capture((x, y, 0.), yaw, frame, 15, .5, .002, 'boot-a')


def test_defaults_include_both_names_of_overlapping_points(tool):
    assert tool.POINT_ORDER == ('0','1.1','1.2','2.1','2.2','3','4.1','4.2','5.1','5.2')
    assert tool.DEFAULT_POINTS == {'0':(0.,0.),'1.1':(1.7,0.),'1.2':(2.2,0.),
        '2.1':(1.7,1.4),'2.2':(2.2,1.4),'3':(0.,1.4),
        '4.1':(1.2,1.4),'4.2':(2.2,1.4),'5.1':(1.2,2.7),'5.2':(2.2,2.7)}


def test_origin_heading_defines_forward_left_even_after_current_heading_changes(tool):
    origin = capture(tool, 10., 20., math.pi/2)
    point = capture(tool, 8.6, 21.7, -math.pi/2)
    assert tool.relative_xy(origin, point) == pytest.approx((1.7, 1.4))


@pytest.mark.parametrize('change', ['frame','boot'])
def test_reference_reset_cannot_mix_coordinate_frames(tool, change):
    from dataclasses import replace
    origin = capture(tool, 0., 0.)
    point = replace(origin, **({'frame':'new_frame'} if change=='frame' else {'boot_id':'boot-b'}))
    with pytest.raises(tool.CalibrationError):tool.relative_xy(origin, point)


def samples(tool, movement=0., frame='odom'):
    return [tool.PoseSample((1.+movement*i/9,2.,0.), math.pi-.001 if i%2 else -math.pi+.001,
                           frame, 10.+i*.05, 1_000_000_000+i*50_000_000) for i in range(10)]


def test_sampling_uses_fresh_stationary_window_and_circular_heading(tool):
    result = tool.aggregate_samples(samples(tool), 10., 'boot-a')
    assert result.position == pytest.approx((1.,2.,0.))
    assert abs(abs(result.yaw)-math.pi)<.002
    assert result.count==10 and result.span_s==pytest.approx(.45)


@pytest.mark.parametrize('failure', ['moving','old','few','frame','frozen'])
def test_bad_measurement_windows_are_rejected(tool, failure):
    from dataclasses import replace
    rows=samples(tool, .1 if failure=='moving' else 0.)
    if failure=='old':rows=[replace(p,received=p.received-1.) for p in rows]
    if failure=='few':rows=rows[:2]
    if failure=='frame':rows[-1]=replace(rows[-1],frame='reset')
    if failure=='frozen':rows=[replace(p,stamp_ns=1_000_000_000) for p in rows]
    with pytest.raises(tool.CalibrationError):tool.aggregate_samples(rows,10.,'boot-a')


def test_map_keeps_overlapping_ids_current_point_and_all_coordinates(tool):
    output=tool.render_map(tool.DEFAULT_POINTS,current='4.2',completed={'0','2.2'},width=80)
    assert '*4.2' in output and '2.2' in output
    for name in tool.POINT_ORDER:assert name in output
    assert '2.20' in output and '2.70' in output and 'X+' in output and 'Y+' in output
    assert max(map(len,output.splitlines()))<=100


def test_manual_edits_are_independent_origin_is_fixed_and_nan_rejected(tool,tmp_path):
    output=[]
    answers=iter(['2.2','2.25, 1.45','0','5.1','nan 2.7','1.25 2.75','s'])
    result=tool.manual_mode(tool.default_document(),tmp_path/'coordinates.json',lambda _:next(answers),output.append)
    assert result['points']['2.2']==[2.25,1.45]
    assert result['points']['4.2']==[2.2,1.4]
    assert result['points']['0']==[0.,0.]
    assert result['points']['5.1']==[1.25,2.75]
    assert tool.load_document(tmp_path/'coordinates.json')['points']==result['points']


def test_restore_defaults_requires_save_and_never_modifies_input_document(tool,tmp_path):
    document=tool.default_document();document['points']['1.1']=[9.,8.]
    answers=iter(['r','q'])
    assert tool.manual_mode(document,tmp_path/'coordinates.json',lambda _:next(answers),lambda _:None) is None
    assert document['points']['1.1']==[9.,8.] and not (tmp_path/'coordinates.json').exists()
    answers=iter(['r','s'])
    result=tool.manual_mode(document,tmp_path/'coordinates.json',lambda _:next(answers),lambda _:None)
    assert result['points']['1.1']==[1.7,0.] and result['reference'] is None


def test_automatic_prompts_all_points_and_converts_with_single_origin(tool,tmp_path):
    readings=[]
    for name in tool.POINT_ORDER:
        x,y=tool.DEFAULT_POINTS[name]
        readings.append(capture(tool,10.-y,20.+x,math.pi/2))
    class Reader:
        def capture(self):return readings.pop(0)
    prompts=[];output=[]
    def enter(prompt):prompts.append(prompt);return ''
    result=tool.automatic_mode(tool.default_document(),tmp_path/'coordinates.json',Reader(),enter,output.append)
    assert len(prompts)==10 and all(name in prompt for name,prompt in zip(tool.POINT_ORDER,prompts))
    assert result['status']=='complete' and result['method']=='automatic'
    for name,xy in tool.DEFAULT_POINTS.items():assert result['points'][name]==pytest.approx(xy)
    assert result['reference']['yaw_rad']==pytest.approx(math.pi/2)
    assert len(result['measurements'])==10


def test_auto_failure_retries_same_point_and_cancel_keeps_previous_file(tool,tmp_path):
    p=tmp_path/'coordinates.json';tool.save_document(p,tool.default_document());before=p.read_bytes()
    readings=iter([capture(tool,0.,0.),tool.CalibrationError('定位过期')])
    class Reader:
        def capture(self):
            value=next(readings)
            if isinstance(value,Exception):raise value
            return value
    answers=iter(['','','q']);prompts=[]
    def answer(prompt):prompts.append(prompt);return next(answers)
    assert tool.automatic_mode(tool.default_document(),p,Reader(),answer,lambda _:None) is None
    assert '1.1' in prompts[1] and '1.1' in prompts[2]
    assert p.read_bytes()==before


def test_save_backup_and_reject_malformed_file(tool,tmp_path):
    p=tmp_path/'coordinates.json';tool.save_document(p,tool.default_document());before=p.read_bytes()
    new=tool.default_document();new['points']['1.1']=[1.8,.01];tool.save_document(p,new)
    assert any(q.read_bytes()==before for q in tmp_path.glob('coordinates.backup.*.json'))
    bad=tool.default_document();bad['points']['0']=[.1,0.];p.write_text(json.dumps(bad))
    with pytest.raises(tool.CalibrationError):tool.load_document(p)


def test_flight_config_paths_are_not_valid_calibration_outputs(tool,tmp_path):
    with pytest.raises(tool.CalibrationError):tool.save_document(tmp_path/'route_1.yaml',tool.default_document())
    with pytest.raises(tool.CalibrationError):tool.save_document(Path.home()/'simple_flight_ws/config/doors.json',tool.default_document())


def test_live_reader_ignores_duplicates_and_aborts_on_jump_or_clock_reset(tool):
    from types import SimpleNamespace as N
    reader=tool.RosPoseReader()
    clock=N(nanoseconds=1_500_000_000)
    reader.node=N(get_clock=lambda:N(now=lambda:clock))
    def message(ns,x=0.):
        return N(header=N(stamp=N(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000),frame_id='odom'),
                 pose=N(position=N(x=x,y=0.,z=0.),orientation=N(x=0.,y=0.,z=0.,w=1.)))
    reader._pose(message(1_100_000_000));reader._pose(message(1_100_000_000))
    assert len(reader.rows)==1
    reader._pose(message(1_200_000_000,2.))
    assert isinstance(reader.error,tool.ReferenceChangedError) and len(reader.rows)==1
    reader=tool.RosPoseReader();reader.node=N(get_clock=lambda:N(now=lambda:clock))
    reader._pose(message(1_200_000_000));reader._pose(message(1_100_000_000))
    assert isinstance(reader.error,tool.ReferenceChangedError)


def test_live_reader_does_not_refresh_lidar_health_with_frozen_stamp(tool):
    from types import SimpleNamespace as N
    reader=tool.RosPoseReader();reader.node=N(get_clock=lambda:N(now=lambda:N(nanoseconds=1_500_000_000)))
    msg=N(header=N(stamp=N(sec=1,nanosec=200_000_000)))
    reader._observation('lio',msg);before=reader.observations['lio'][0]
    reader._observation('lio',msg)
    assert reader.observations['lio'][0]==before


def test_frame_change_aborts_whole_auto_survey_without_saving(tool,tmp_path):
    from dataclasses import replace
    readings=iter([capture(tool,0.,0.),replace(capture(tool,1.7,0.),frame='reset')])
    class Reader:
        def capture(self):return next(readings)
    p=tmp_path/'coordinates.json'
    with pytest.raises(tool.ReferenceChangedError):
        tool.automatic_mode(tool.default_document(),p,Reader(),lambda _:'',lambda _:None)
    assert not p.exists()


def test_mismatched_coordinate_convention_and_unreasonable_input_are_rejected(tool,tmp_path):
    doc=tool.default_document();doc['axes']='world_xyz'
    with pytest.raises(tool.CalibrationError):tool.save_document(tmp_path/'coordinates.json',doc)
    with pytest.raises(tool.CalibrationError):tool.parse_xy('1e300 0')


def test_capture_accepts_actual_mavros_estimator_flag_field_names(tool,monkeypatch):
    from types import SimpleNamespace as N
    reader=tool.RosPoseReader();reader.boot_id='boot-a'
    reader.rows.extend(tool.PoseSample((1.,2.,0.),0.,'odom',100.01+i*.05,1_000_000_000+i*50_000_000) for i in range(11))
    reader.observations={'state':(100.7,N(connected=True,armed=False)),
                         'landed':(100.7,N(landed_state=1)),
                         'estimator':(100.7,N(pos_horiz_abs_status_flag=False,pos_horiz_rel_status_flag=True,attitude_status_flag=True)),
                         'lio':(100.7,N())}
    times=iter([100.,100.7]);monkeypatch.setattr(tool.time,'monotonic',lambda:next(times))
    result=reader.capture()
    assert result.position==(1.,2.,0.) and result.count==11
