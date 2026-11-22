from simple_flight_mission.hover_takeover import HoverTakeover
from simple_flight_mission.hover_controller import EXPECTED_PARAMETERS
from test.test_hover_controller import Rig

def test_new_airframe_mapping():
    assert EXPECTED_PARAMETERS['RC_MAP_OFFB_SW']==7
    assert EXPECTED_PARAMETERS['RC_MAP_FLTMODE']==5
    assert EXPECTED_PARAMETERS['RC7_REV']==1.0

def test_ch7_start_and_takeover():
    gate=HoverTakeover()
    channels=[1514,1514,1024,1514,2003,1024,1024]
    assert gate.observe(channels,0).action=='none'
    assert gate.observe(channels,.25).action=='none'
    channels[6]=2003
    assert gate.observe(channels,.3,entry_ready=True).action=='start'
    gate.confirm_offboard(.31)
    channels[6]=1024
    assert gate.observe(channels,.4).action=='exit'

def test_ch5_position_gate():
    rig=Rig()
    rig.rc=[1514,1514,1024,1514,2003,1024,1024]
    rig.ready()
    rig.rc[4]=1514
    assert 'main_switch_not_position' in rig.controller._ground_reasons(rig.vehicle(),rig.t)
