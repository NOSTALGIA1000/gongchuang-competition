"""Local pipeline continuation: evaluate once training writes a completed status."""
from pathlib import Path
import json,time,subprocess,sys,traceback,os,ctypes

BASE=Path(__file__).resolve().parent
WORK=BASE/'training/targets_v3_yolov5s'

def state(value):
    p=WORK/'completion_pipeline.json';t=p.with_suffix('.tmp');t.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf8');t.replace(p)

def main():
    began=time.time();state({'status':'waiting_for_training','pid':os.getpid()})
    while time.time()-began<4*3600:
        try:r=json.loads((WORK/'run_info.json').read_text(encoding='utf8'))
        except (FileNotFoundError,json.JSONDecodeError):time.sleep(10);continue
        if r['status']=='failed':raise RuntimeError('Training failed: '+r.get('error','unknown'))
        if r['status']=='completed':break
        import psutil
        if 'pid' in r:
            try:alive=psutil.Process(r['pid']);assert any('run_v3_training.py' in a for a in alive.cmdline())
            except (psutil.NoSuchProcess,AssertionError):raise RuntimeError('Training process exited before completion; checkpoints and logs preserved.')
        time.sleep(20)
    else:raise TimeoutError('Training did not complete within four hours; checkpoints are preserved.')
    state({'status':'evaluating','training_epochs_completed':r['completed_epoch'],'pid':os.getpid()})
    with (WORK/'evaluation_stdout.log').open('w',encoding='utf8') as out,(WORK/'evaluation_stderr.log').open('w',encoding='utf8') as err:
        subprocess.run([sys.executable,'-X','utf8','-u',str(BASE/'evaluate_v3_training.py')],cwd=str(BASE.parent/'third_party/yolov5'),stdout=out,stderr=err,check=True)
    report=json.loads((WORK/'evaluation/evaluation.json').read_text(encoding='utf8'))
    state({'status':'completed','weights':r['weights'],'report':str(WORK/'训练结果说明.md'),'overall':report['overall'],'epochs_completed':r['completed_epoch']})

if __name__=='__main__':
    # Prevent idle system sleep only while this training/evaluation pipeline runs.
    # Restore normal behavior on exit; no global power-policy changes.
    ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)
    try:main()
    except BaseException as e:
        state({'status':'failed','error':repr(e),'traceback':traceback.format_exc()});raise
    finally:ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
