"""Rescue scan may meet its first off-center target without cached calibration."""
import math

import pytest

from test.test_rescue_aisle_workflow import RescueRig


@pytest.mark.parametrize('route',[1,2,3,4])
def test_uncalibrated_person_acquisition_measures_axes_then_drops_and_rejoins(route):
    rig=RescueRig(route,yaw=.7,person=True)
    rig.controller.jacobian=None
    rig.reach_rescue()
    anchor=rig.controller.world_route[4]
    phases=set();rejoined=False;opened=False
    for _ in range(1500):
        out=rig.advance()
        assert out.phase not in ('HANDOVER','EXITED'),out
        phases.add(rig.controller.task.phase)
        if (2,'open') in rig.actions:opened=True
        if opened and math.dist(rig.pose[:2],anchor[:2])<=.15:rejoined=True
        if rig.controller.returning:
            assert {'PROBE_X','PROBE_Y'}<=phases
            assert rig.controller.jacobian is not None
            assert opened and rejoined and rig.controller.visual_error is None
            assert rig.actions[-2:]==[(2,'open'),(2,'close')]
            return
    pytest.fail('Uncalibrated acquisition did not complete via waypoint 5')
