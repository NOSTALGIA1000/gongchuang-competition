"""Four-route rescue aisle contract; virtual observations only, no ROS/I/O."""
import math
from pathlib import Path

import pytest

from simple_flight_mission.competition_controller import CompetitionController
from simple_flight_mission.route_config import load_route_config
from simple_flight_mission.visual_task import VisualTask
from test.test_competition import frame
from test.test_route_flight import RouteRig, follow


ROOT=Path(__file__).resolve().parents[1]


class RescueRig(RouteRig):
    def __init__(self, route_number, yaw=0., person=False):
        super().__init__('execute')
        self.pose=(*self.pose[:3],yaw,0.)
        route=load_route_config(ROOT/f'config/route_{route_number}.yaml')
        doors={'doors':{str(n):dict(min_us=1000,max_us=2000,open_us=1056,close_us=1500) for n in (1,2)}}
        self.controller=CompetitionController('execute',route,doors,'rc')
        self.controller.payload_initialized=True
        self.controller.jacobian=((400.,0.),(0.,400.))
        self.params.update(EKF2_GPS_CTRL=0,COM_DISARM_LAND=2.,LNDMC_Z_VEL_MAX=.25,
                           MPC_LAND_SPEED=.7,MPC_LAND_CRWL=.3,MPC_LAND_ALT3=1.,MPC_Z_VEL_MAX_DN=1.5,
                           PWM_AUX_FUNC7=301,PWM_AUX_FUNC8=302,PWM_AUX_MIN7=1000,
                           PWM_AUX_MAX7=2000,PWM_AUX_MIN8=1000,PWM_AUX_MAX8=2000,PWM_AUX_REV=0)
        self.rescue_index=next(i for i,p in enumerate(route.points) if p.stage=='task_zone_2')
        self.start_forward=route.points[self.rescue_index].forward_m
        self.end_forward=route.search_xy[0]
        self.direction=1 if self.end_forward>self.start_forward else -1
        self.show_person=person;self.person_seen_at=None;self.person_seen_forward=None
        self.actions=[]

    def relative(self, pose):
        origin=self.controller.origin
        dx,dy=pose[0]-origin[0],pose[1]-origin[1]
        c,s=math.cos(origin[3]),math.sin(origin[3])
        return dx*c+dy*s,-dx*s+dy*c

    def world(self, forward,left):
        x,y,z,yaw=self.controller.origin;c,s=math.cos(yaw),math.sin(yaw)
        return x+forward*c-left*s,y+forward*s+left*c

    def tick(self, **changes):
        now=self.t+.05;controller=self.controller
        rescue=(controller.point_index==self.rescue_index
                or bool(controller.task and controller.task.door==2))
        snapshot=dict(frame(now,kind=0 if rescue else 2),stream_id='camera-one')
        if rescue:
            forward,_=self.relative(self.pose)
            progress=self.direction*(forward-self.start_forward)
            if self.show_person and (progress>=.4 or self.person_seen_at is not None):
                if self.person_seen_at is None:self.person_seen_at=now;self.person_seen_forward=forward
                target=self.world(self.start_forward+self.direction*.65,2.82)
                u,v=320+400*(self.pose[0]-target[0]),240+400*(self.pose[1]-target[1])
                snapshot['detections'][0]['box']=[u-20,v-20,u+20,v+20]
            else:snapshot['detections']=[]
        controller.vision_data=snapshot
        return super().tick(**changes)

    def advance(self):
        out=follow(self)[0]
        action=self.controller.take_payload_action()
        if action:
            self.actions.append(action);self.controller.payload_ack=(*action,True)
        return out

    def reach_rescue(self):
        self.active()
        for _ in range(5000):
            out=self.advance()
            assert out.phase not in ('HANDOVER','EXITED'),out
            if self.controller.task and self.controller.task.door==2:return out
        pytest.fail('Did not reach the route-5 rescue anchor')


@pytest.mark.parametrize('route,yaw',[(1,0.),(2,0.),(3,1.1),(4,-.9)])
def test_each_route_starts_search_toward_the_opposite_aisle_end_without_fixed_wait(route,yaw):
    rig=RescueRig(route,yaw);rig.reach_rescue()
    start=rig.relative(rig.pose)[0]
    for _ in range(10):
        out=rig.advance()
        forward,left=rig.relative(out.target)
        assert min(rig.start_forward,rig.end_forward)-1e-6<=forward<=max(rig.start_forward,rig.end_forward)+1e-6
        assert left==pytest.approx(2.7,abs=1e-6)
    assert rig.direction*(rig.relative(rig.pose)[0]-start)>.02,'No 2.5s wait at waypoint 5 before scanning'


def test_no_person_search_does_not_pause_at_the_midpoint():
    rig=RescueRig(1);rig.reach_rescue()
    # The scoring waypoint itself is now a real stationary dwell; the interior
    # of the aisle must still be traversed without any timed pause.
    previous=rig.pose;stationary=longest=0
    for _ in range(450):
        out=rig.advance();forward,_=rig.relative(rig.pose)
        if 1.0<forward<1.95:
            stationary=stationary+1 if math.dist(previous[:2],rig.pose[:2])<1e-9 else 0
            longest=max(longest,stationary)
        previous=rig.pose
        if rig.controller.returning:break
    assert longest*.05<.5,'The aisle midpoint is not a timed dwell waypoint'


@pytest.mark.parametrize('route',[1,2,3,4])
def test_search_exhaustion_returns_to_its_own_waypoint5_before_route_return(route):
    rig=RescueRig(route,yaw=.7);rig.reach_rescue()
    anchor=rig.controller.world_route[rig.rescue_index];end=rig.end_forward
    visited_other_end=False;rejoined=False
    started=rig.controller.task.started
    for _ in range(1500):
        out=rig.advance()
        assert out.phase not in ('HANDOVER','EXITED'),out
        assert rig.controller.task.started==started
        forward,left=rig.relative(rig.pose)
        assert left==pytest.approx(2.7,abs=1e-6)
        if abs(forward-end)<=.15:visited_other_end=True
        if visited_other_end and math.dist(rig.pose[:2],anchor[:2])<=.15:rejoined=True
        if rig.controller.returning and out.point_index!=rig.rescue_index:
            assert visited_other_end and rejoined,'Route return must follow actual return to waypoint 5'
            assert rig.controller.visual_error=='correct_target_not_found'
            assert not any(door==2 for door,_ in rig.actions)
            return
    pytest.fail('Search exhaustion did not terminate through the original rescue anchor')


@pytest.mark.parametrize('route',[1,2])
def test_person_detected_mid_search_is_pursued_and_door2_drop_rejoins_waypoint5(route):
    rig=RescueRig(route,yaw=.7,person=True);rig.reach_rescue()
    anchor=rig.controller.world_route[rig.rescue_index];opened=False;rejoined=False;lateral_started=False
    for _ in range(1500):
        out=rig.advance()
        assert out.phase not in ('HANDOVER','EXITED'),out
        forward,left=rig.relative(rig.pose)
        if rig.person_seen_at is not None and not lateral_started and abs(left-2.7)>.015:
            lateral_started=True
            assert rig.direction*(forward-rig.person_seen_forward)<.12,'A seen person must interrupt blind aisle scanning'
        if (2,'open') in rig.actions:opened=True
        if opened and math.dist(rig.pose[:2],anchor[:2])<=.15:rejoined=True
        if rig.controller.returning:
            assert opened and lateral_started and rejoined
            assert rig.actions[-2:]==[(2,'open'),(2,'close')]
            assert rig.controller.visual_error is None
            return
    pytest.fail('The detected person was not rescued followed by return through waypoint 5')


def empty_camera(now,sequence=None,age=0.):
    return dict(frame(now,kind=0,sequence=sequence),detections=[],age_s=age)


def test_initial_invalid_camera_never_starts_rescue_motion():
    anchor=(2.87,2.7,1.,0.)
    task=VisualTask(2,anchor,[(1.2,2.7,1.,0.)],0.)
    for i in range(1,61):
        now=i*.1
        out=task.step((*anchor,0.),(0.,0.,0.),now,dict(empty_camera(now),valid=False))
        assert out.target==anchor and out.action is None and out.failed is None


def test_invalid_camera_pauses_an_active_scan_and_new_camera_resumes_it():
    anchor=(2.87,2.7,1.,0.);endpoint=(1.2,2.7,1.,0.)
    task=VisualTask(2,anchor,[endpoint],0.);pose=(*anchor,0.)
    for i in range(1,11):
        now=i*.1
        out=task.step(pose,(0.,0.,0.),now,empty_camera(now));pose=(*out.target,0.)
    stopped=pose[:4]
    assert stopped[0]<anchor[0]-.02
    for i in range(11,17):
        now=i*.1
        out=task.step(pose,(0.,0.,0.),now,dict(empty_camera(now),valid=False))
        assert out.target==stopped and out.action is None
        pose=(*out.target,0.)
    out=task.step(pose,(0.,0.,0.),1.7,empty_camera(1.7))
    assert endpoint[0]<=out.target[0]<stopped[0]
    assert math.dist(out.target[:2],stopped[:2])<=.012001


@pytest.mark.parametrize('cached_case',['same_sequence','before_arrival'])
def test_search_endpoint_requires_a_new_post_arrival_frame_before_failure_return(cached_case):
    anchor=(2.87,2.7,1.,0.);endpoint=(1.2,2.7,1.,0.)
    task=VisualTask(2,anchor,[endpoint],0.);pose=(*anchor,0.)
    for i in range(1,180):
        now=i*.1
        out=task.step(pose,(0.,0.,0.),now,empty_camera(now,sequence=i))
        pose=(*out.target,0.)
        if math.dist(out.target[:2],endpoint[:2])<1e-8:break
    else:pytest.fail('Continuous search did not reach the opposite endpoint')
    arrival=now+.1
    seq=i if cached_case=='same_sequence' else i+1
    age=0. if cached_case=='same_sequence' else .2
    out=task.step(pose,(0.,0.,0.),arrival,empty_camera(arrival,sequence=seq,age=age))
    assert out.target==endpoint and out.failed is None,'Cached endpoint absence cannot end the search'
    out=task.step(pose,(0.,0.,0.),arrival+.1,empty_camera(arrival+.1,sequence=i+2))
    assert endpoint[0]<out.target[0]<=anchor[0] and out.failed is None
    assert not task.open_sent
