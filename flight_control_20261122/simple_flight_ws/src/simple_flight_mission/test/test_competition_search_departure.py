"""Offline regressions from the route-1 competition flight log."""
import math

import pytest

from simple_flight_mission.competition_controller import CompetitionController
from simple_flight_mission.hover_controller import Observation
from simple_flight_mission.route_config import RouteConfig
from simple_flight_mission.visual_task import VisualTask
from test.test_competition import frame
from test.test_route_flight import RouteRig, follow


ANCHOR=(2.87,2.7,1.,0.)
SCANS=[(2.035,2.7,1.,0.),(1.2,2.7,1.,0.)]


def person_frame(now, pose, target_x=2.37):
    u=320+400*(pose[0]-target_x)
    return frame(now,kind=0,box=(u-20,220,u+20,260))


def test_far_visible_person_enters_alignment_instead_of_returning():
    task=VisualTask(2,ANCHOR,SCANS,0.,jacobian=((400.,0.),(0.,400.)))
    pose=(*ANCHOR,0.)
    for i in range(1,16):
        now=i*.1
        out=task.step(pose,(0.,0.,0.),now,person_frame(now,pose))
        assert out.failed is None, 'A visible person outside fine-alignment radius must not skip aisle search'
    assert task.phase=='ALIGN' and 1.2<=task.desired[0]<ANCHOR[0]
    assert task.desired[1:]==ANCHOR[1:] and not task.open_sent


def test_rescue_search_moves_only_on_aisle_and_opens_on_a_new_centered_frame():
    task=VisualTask(2,ANCHOR,SCANS,0.,jacobian=((400.,0.),(0.,400.)))
    pose=(*ANCHOR,0.);opened=False
    for i in range(1,180):
        now=i*.1
        image=person_frame(now,pose)
        out=task.step(pose,(0.,0.,0.),now,image)
        assert out.failed is None
        assert out.target[1:]==ANCHOR[1:]
        assert 1.2<=out.target[0]<=2.87
        assert math.dist(out.target[:2],pose[:2])<=.020001
        if out.action=='open':
            assert abs(image['detections'][0]['box'][0]+20-320)<=50
            opened=True;break
        pose=(*out.target,0.)
    assert opened


@pytest.mark.parametrize('door',[1])
def test_actual_fine_alignment_excursion_still_aborts_without_opening(door):
    task=VisualTask(door,ANCHOR,SCANS if door==2 else [],0.,jacobian=((400.,0.),(0.,400.)))
    task.phase='ALIGN'
    pose=(ANCHOR[0]+.36,*ANCHOR[1:],0.)
    out=task.step(pose,(0.,0.,0.),.1,frame(.1,kind=0 if door==2 else 2))
    assert out.failed=='visual_alignment_bound' and out.action is None


@pytest.mark.parametrize('door',[1])
def test_far_target_without_any_approved_search_point_keeps_the_bound(door):
    task=VisualTask(door,ANCHOR,[],0.,jacobian=((400.,0.),(0.,400.)))
    task.phase='ALIGN';pose=(*ANCHOR,0.)
    for i in range(1,8):
        now=i*.1
        out=task.step(pose,(0.,0.,0.),now,frame(now,kind=0 if door==2 else 2,box=(500,220,540,260)))
        if out.failed:break
    assert out.failed=='visual_alignment_bound' and not task.open_sent


def competition_rig():
    class Rig(RouteRig):
        def tick(self, **changes):
            self.controller.vision_data=dict(frame(self.t+.05),stream_id='camera-one')
            return super().tick(**changes)
    rig=Rig('execute')
    doors={'doors':{str(n):dict(min_us=1000,max_us=2000,open_us=1056,close_us=1500) for n in (1,2)}}
    rig.controller=CompetitionController('execute',RouteConfig(),doors,'rc')
    rig.controller.payload_initialized=True
    rig.params.update(EKF2_GPS_CTRL=0,COM_DISARM_LAND=2.,LNDMC_Z_VEL_MAX=.25,
                      MPC_LAND_SPEED=.7,MPC_LAND_CRWL=.3,MPC_LAND_ALT3=1.,MPC_Z_VEL_MAX_DN=1.5,
                      PWM_AUX_FUNC7=301,PWM_AUX_FUNC8=302,PWM_AUX_MIN7=1000,
                      PWM_AUX_MAX7=2000,PWM_AUX_MIN8=1000,PWM_AUX_MAX8=2000,PWM_AUX_REV=0)
    return rig


def test_competition_departs_at_target_height_without_origin_hover_dwell():
    rig=competition_rig();rig.active()
    for _ in range(400):
        before=rig.pose
        out=follow(rig)[0]
        assert out.phase!='ORIGIN_HOLD', 'Competition entry must not add an origin hover timer'
        if out.phase=='TRANSIT':
            goal=rig.controller.origin[2]+rig.controller.route_config.height_m
            assert abs(before[2]-goal)<=rig.controller.route_config.arrival_z_m+1e-8
            assert out.point_index==0
            assert out.target[:2]==rig.controller.origin[:2]
            return
        assert out.phase not in ('HANDOVER','EXITED')
    pytest.fail('Competition did not start its first leg')


def test_competition_does_not_depart_below_target_height():
    rig=competition_rig();rig.active()
    rig.pose=(*rig.pose[:2],rig.pose[2]+.4,*rig.pose[3:]);rig.landed=2
    for _ in range(80):
        out=rig.tick()
        assert out.phase=='TAKEOFF'


@pytest.mark.parametrize('fault',['stale_pose','manual_mode','vertical_speed','heading','rc_takeover'])
def test_departure_at_height_does_not_bypass_flight_guards(fault):
    rig=competition_rig();rig.active()
    goal=rig.controller.origin[2]+rig.controller.route_config.height_m
    while rig.pose[2]<goal-.2:
        assert follow(rig)[0].phase=='TAKEOFF'
    rig.pose=(*rig.pose[:2],goal,*rig.pose[3:]);rig.landed=2
    changes={}
    if fault=='stale_pose':changes['pose']=Observation(rig.pose,rig.t,2.)
    elif fault=='manual_mode':rig.mode='POSCTL'
    elif fault=='vertical_speed':changes['velocity']=Observation((0.,0.,.8),rig.t,0.)
    elif fault=='heading':rig.heading_offset=math.radians(15)
    else:rig.rc[6]=1024
    out=rig.tick(**changes)
    assert out.phase in ('HANDOVER','EXITED') and out.phase!='TRANSIT'


def test_rescue_coarse_search_does_not_reset_the_task_deadline():
    task=VisualTask(2,ANCHOR,SCANS,0.,jacobian=((400.,0.),(0.,400.)))
    pose=(*ANCHOR,0.)
    for i in range(1,16):
        now=i*.1
        task.step(pose,(0.,0.,0.),now,person_frame(now,pose))
    assert task.phase=='ALIGN' and task.started==0.
    out=task.step(pose,(0.,0.,0.),45.1,person_frame(45.1,pose))
    assert out.failed=='visual_task_timeout' and not task.open_sent


def test_coarse_rescue_search_is_visible_in_operator_progress():
    from simple_flight_mission.competition_node import task_progress_text
    text=task_progress_text(dict(task_door=2,task_phase='SEARCH',task_diagnostics={
        'wait_reason':'person_outside_fine_radius_searching_aisle'}))
    assert '沿既定救援通道继续搜索' in text
