"""Linux PTY tests: fake Python/ROS and camera, never contact flight hardware."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import time

import pytest


class Launcher:
    def __init__(self,tmp_path,check_exit=0):
        self.root=tmp_path
        self.events=tmp_path/'events.jsonl'
        home=tmp_path/'home';bin_dir=tmp_path/'bin';bin_dir.mkdir()
        for folder in ('simple_flight_ws/install','livox_ws/install','fastlio_ws/install'):
            path=home/folder;path.mkdir(parents=True);(path/'setup.bash').write_text(':\n')
        cfg=home/'simple_flight_ws/config';cfg.mkdir();(cfg/'route_smoke.yaml').write_text('points: []\n')
        setup=tmp_path/'setup.bash';setup.write_text(':\n')
        source=Path(os.environ['TEST_LAUNCHER'])
        self.script=tmp_path/'competitionctl.sh'
        self.script.write_text(source.read_text().replace('/opt/ros/humble/setup.bash',str(setup)))
        python=bin_dir/'python3'
        python.write_text('#!'+sys.executable+'''\nimport sys,os,json,time
args=sys.argv[1:]
if args[:2]==['-m','simple_flight_mission.competition_node']:
    with open(os.environ['FAKE_EVENTS'],'a') as f:f.write(json.dumps(args)+'\\n')
    if '--check' in args:
        print('FAKE_CHECK',flush=True);time.sleep(.25)
        raise SystemExit(int(os.environ['FAKE_CHECK_EXIT']))
    if '--wait-enter' not in args:
        raise SystemExit('Missing READY-stage Enter gate')
    from types import SimpleNamespace
    from simple_flight_mission.competition_node import CompetitionNode
    import signal
    node=CompetitionNode.__new__(CompetitionNode)
    node.controller=SimpleNamespace(require_enter=True,enter_confirmed=False,phase='PRESTREAM')
    def confirm(vehicle,now):
        node.controller.enter_confirmed=True
        return True
    node.controller.confirm_start=confirm
    node.node=SimpleNamespace(get_logger=lambda:SimpleNamespace(info=lambda s:print(s,flush=True),warning=lambda s:print(s,flush=True)))
    node.enter_prompted=False;node.snapshot=lambda:None;node.stopped=False;node.code=2
    def stop():node.stopped=True
    node.request_stop=stop
    def interrupt(*unused):node.code=130;stop()
    signal.signal(signal.SIGINT,interrupt)
    print('FAKE_PREPARING_CLOSE_AND_PRESTREAM',flush=True);time.sleep(.25)
    if int(os.environ['FAKE_CHECK_EXIT']):
        print('FAKE_PREPARATION_FAILED',flush=True)
        raise SystemExit(2)
    node.controller.phase='READY'
    while not node.stopped and not node.controller.enter_confirmed:
        node._poll_enter(time.monotonic());time.sleep(.01)
    if node.stopped:raise SystemExit(node.code)
    print('FAKE_OFFBOARD_NO_HARDWARE',flush=True)
else:
    sys.stdin.read()  # Fake only the camera-readiness inline script.
''')
        python.chmod(0o755)
        curl=bin_dir/'curl';curl.write_text('#!/bin/sh\nexit 0\n');curl.chmod(0o755)
        self.env=dict(os.environ,HOME=str(home),PATH=str(bin_dir)+':'+os.environ['PATH'],
                      FAKE_EVENTS=str(self.events),FAKE_CHECK_EXIT=str(check_exit))
        self.process=None;self.master=None;self.output=b''

    def start(self):
        master,slave=pty.openpty();self.master=master
        self.process=subprocess.Popen(['bash',str(self.script),'run','1'],env=self.env,
                                      stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
        os.close(slave)
        return self

    def read_until(self,text,timeout=4.):
        deadline=time.monotonic()+timeout
        while text.encode() not in self.output:
            if time.monotonic()>deadline:pytest.fail(self.output.decode(errors='replace'))
            if select.select([self.master],[],[],.05)[0]:
                try:chunk=os.read(self.master,65536)
                except OSError:chunk=b''
                self.output+=chunk
                if not chunk:pytest.fail('Launcher exited before prompt: '+self.output.decode(errors='replace'))
        return self.output.decode(errors='replace')

    def calls(self):
        return [json.loads(x) for x in self.events.read_text().splitlines()] if self.events.exists() else []

    def close(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        if self.master is not None:os.close(self.master);self.master=None


@pytest.fixture
def launch(tmp_path):
    instance=Launcher(tmp_path)
    yield instance
    instance.close()


def test_precheck_then_wait_for_enter_before_execute(launch):
    launch.start();launch.read_until('准备完成：')
    assert len(launch.calls())==1 and '--execute' in launch.calls()[0] and '--wait-enter' in launch.calls()[0]
    assert launch.process.poll() is None
    os.write(launch.master,b'\n')
    launch.read_until('FAKE_OFFBOARD_NO_HARDWARE')
    assert launch.process.wait(timeout=3)==0
    assert len(launch.calls())==1, 'Preparation and confirmation must use one controller process'


def test_enter_typed_during_preparation_is_discarded(launch):
    launch.start();launch.read_until('FAKE_PREPARING_CLOSE_AND_PRESTREAM')
    os.write(launch.master,b'\n')
    launch.read_until('准备完成：')
    time.sleep(.1)
    assert len(launch.calls())==1 and launch.process.poll() is None
    os.write(launch.master,b'\n')
    launch.read_until('FAKE_OFFBOARD_NO_HARDWARE')
    assert launch.process.wait(timeout=3)==0


def test_failed_precheck_never_reaches_execute(tmp_path):
    launcher=Launcher(tmp_path,check_exit=2).start()
    try:
        launcher.read_until('FAKE_PREPARATION_FAILED')
        assert launcher.process.wait(timeout=3)==2
        assert len(launcher.calls())==1 and '--wait-enter' in launcher.calls()[0]
    finally:launcher.close()


def test_piped_enter_cannot_confirm_noninteractive_run(launch):
    result=subprocess.run(['bash',str(launch.script),'run','1'],env=launch.env,input='\n',
                          capture_output=True,text=True,timeout=4)
    assert result.returncode==2 and '交互终端' in result.stderr
    assert launch.calls()==[]


def test_terminal_disconnect_cancels_without_execute(launch):
    launch.start();launch.read_until('准备完成：')
    os.close(launch.master);launch.master=None
    assert launch.process.wait(timeout=3)!=0
    assert len(launch.calls())==1


def test_ctrl_c_while_waiting_cancels_without_execute(launch):
    launch.start();launch.read_until('准备完成：')
    os.kill(launch.process.pid,signal.SIGINT)
    assert launch.process.wait(timeout=3)==130
    assert len(launch.calls())==1


def test_nonempty_input_does_not_confirm(launch):
    launch.start();launch.read_until('准备完成：')
    os.write(launch.master,b'yes\n')
    launch.read_until('请只按 Enter')
    assert len(launch.calls())==1
    os.write(launch.master,b'\n')
    launch.read_until('FAKE_OFFBOARD_NO_HARDWARE')
    assert launch.process.wait(timeout=3)==0


@pytest.mark.parametrize('action',['check','smoke-run'])
def test_waiting_for_enter_keeps_both_mission_locks(launch,action):
    launch.start();launch.read_until('准备完成：')
    result=subprocess.run(['bash',str(launch.script),action,'1'],env=launch.env,
                          input='',capture_output=True,text=True,timeout=4)
    assert result.returncode==3
    assert len(launch.calls())==1


def test_check_stays_read_only_and_does_not_need_enter(launch):
    result=subprocess.run(['bash',str(launch.script),'check','1'],env=launch.env,
                          input='',capture_output=True,text=True,timeout=4)
    assert result.returncode==0
    assert len(launch.calls())==1 and '--check' in launch.calls()[0]
