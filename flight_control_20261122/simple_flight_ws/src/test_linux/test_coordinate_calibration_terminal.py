"""Real Linux terminal interaction, using fake poses and no ROS/hardware."""
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


BOOTSTRAP = '''
import importlib.util,os,sys,math
spec=importlib.util.spec_from_file_location('calibration',os.environ['CALIBRATION_TOOL'])
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
class FakeReader:
    def __init__(self):self.i=0
    def __enter__(self):return self
    def __exit__(self,*unused):pass
    def capture(self):
        x,y=m.DEFAULT_POINTS[m.POINT_ORDER[self.i]];self.i+=1
        return m.Capture((10.-y,20.+x,0.),math.pi/2,'fake_odom',15,.5,.002,'fake-boot')
m.RosPoseReader=FakeReader
raise SystemExit(m.main(['--output',os.environ['CALIBRATION_OUTPUT']]))
'''


class Terminal:
    def __init__(self,tmp_path):
        self.path=tmp_path/'coordinates.json'
        self.master,slave=pty.openpty();self.output=b''
        self.process=subprocess.Popen([sys.executable,'-u','-c',BOOTSTRAP],stdin=slave,stdout=slave,stderr=slave,
            start_new_session=True,env=dict(os.environ,CALIBRATION_OUTPUT=str(self.path),PYTHONIOENCODING='utf-8'))
        os.close(slave)

    def read(self,text,timeout=8.):
        end=time.monotonic()+timeout
        while text.encode() not in self.output:
            if time.monotonic()>end:pytest.fail(self.output.decode(errors='replace'))
            if select.select([self.master],[],[],.05)[0]:
                try:chunk=os.read(self.master,65536)
                except OSError:chunk=b''
                self.output+=chunk
                if not chunk:pytest.fail('Exited: '+self.output.decode(errors='replace'))
        self.output=b''

    def send(self,text):os.write(self.master,text.encode())

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        os.close(self.master)


@pytest.fixture
def terminal(tmp_path):
    t=Terminal(tmp_path)
    try:yield t
    finally:t.close()


def test_manual_save_and_restore_defaults_in_real_terminal(terminal):
    t=terminal;t.read('选择校准方式');t.send('2\n');t.read('选择点名修改')
    t.send('1.1\n');t.read('输入 X Y');t.send('1.81,0.03\n');t.read('选择点名修改')
    t.send('s\n');t.read('选择校准方式')
    assert json.loads(t.path.read_text())['points']['1.1']==[1.81,.03]
    t.send('2\n');t.read('选择点名修改');t.send('r\n');t.read('选择点名修改')
    t.send('s\n');t.read('选择校准方式')
    assert json.loads(t.path.read_text())['points']['1.1']==[1.7,0.]
    t.send('q\n');assert t.process.wait(timeout=3)==0


def test_all_ten_auto_points_wait_for_enter_and_save_only_at_end(terminal):
    t=terminal;t.read('选择校准方式');t.send('1\n')
    names=('0','1.1','1.2','2.1','2.2','3','4.1','4.2','5.1','5.2')
    for name in names:
        t.read('请将无人机放到 '+name+'-')
        assert not t.path.exists()
        t.send('\n')
    t.read('选择校准方式')
    data=json.loads(t.path.read_text());assert data['status']=='complete'
    assert data['points']['2.2']==pytest.approx((2.2,1.4))
    assert data['points']['4.2']==pytest.approx((2.2,1.4))
    t.send('q\n');assert t.process.wait(timeout=3)==0


def test_cancel_auto_does_not_save_partial_survey(terminal):
    t=terminal;t.read('选择校准方式');t.send('1\n');t.read('请将无人机放到 0-');t.send('\n')
    t.read('请将无人机放到 1.1-');t.send('q\n');t.read('选择校准方式')
    assert not t.path.exists()
    t.send('q\n');assert t.process.wait(timeout=3)==0


def test_ctrl_c_keeps_unsaved_manual_edits_out_of_file(terminal):
    t=terminal;t.read('选择校准方式');t.send('2\n');t.read('选择点名修改')
    t.send('1.1\n');t.read('输入 X Y');t.send('1.81 0.03\n');t.read('选择点名修改')
    t.process.send_signal(signal.SIGINT)
    assert t.process.wait(timeout=3)==0 and not t.path.exists()
