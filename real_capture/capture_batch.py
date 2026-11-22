"""Bounded camera snapshot collection. No flight-control endpoints are used."""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import requests

TZ = timezone(timedelta(hours=8))

def validate_candidate(pixels):
    """Reject placeholder-like black frames, preserving genuinely dark texture."""
    if pixels is None or pixels.shape!=(480,640,3):
        raise ValueError('Unexpected snapshot dimensions')
    candidate=pixels[32:480,0:640].copy()
    if float(candidate.mean())<1.0 and float(candidate.std())<1.0:
        raise ValueError('Uniform black camera image; reject placeholder')
    return candidate

def now():
    return datetime.now(TZ)

def write_json(path, data):
    temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    temp.replace(path)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--url',default='http://10.42.0.1:8088')
    ap.add_argument('--frames',type=int,default=30)
    ap.add_argument('--interval',type=float,default=2)
    ap.add_argument('--batch',default='mixed_signs')
    ap.add_argument('--restore-enabled',action='store_true')
    args=ap.parse_args()
    if args.frames<1 or args.frames>1000 or args.interval<1:raise ValueError('Use 1..1000 frames and interval >=1 second')
    root=Path(__file__).resolve().parent
    token=now().strftime('%Y%m%d_%H%M%S_%f')
    folder=root/'sessions'/(token+'_'+''.join(c for c in args.batch if c.isalnum() or c in '-_'))
    originals=folder/'preview_original';images=folder/'images'
    originals.mkdir(parents=True);images.mkdir()
    state_path=root/'capture_state.json';stop=root/'STOP_CAPTURE'
    if stop.exists():raise RuntimeError('STOP_CAPTURE exists: remove it before starting a new batch')
    state={'active':True,'session':str(folder),'saved':0,'attempts':0,'skipped':0,
           'limit':args.frames,'interval_s':args.interval,'started_at':now().isoformat(),
           'image_kind':'JPEG preview decoded and cropped; NOT native full camera frames',
           'crop_xyxy':[0,32,640,480],'flight_controls_used':False}
    write_json(state_path,state)
    session=requests.Session();initial_stream=None;seen=set();previous_gray=None
    try:
        r=session.get(args.url+'/api/status',timeout=5);r.raise_for_status();initial=r.json()
        initial_stream=initial['stream_id'];state['initial_status']=initial
        if initial['source']!='camera':raise RuntimeError('Input is not a live camera')
        r=session.post(args.url+'/api/detection',json={'enabled':False},timeout=5);r.raise_for_status()
        time.sleep(.5)
        deadline=time.monotonic()+args.frames*args.interval+30
        with (folder/'frames.jsonl').open('a',encoding='utf-8') as log:
            while state['saved']<args.frames and time.monotonic()<deadline:
                if stop.exists():state['stop_reason']='user_stop';break
                tick=time.monotonic();state['attempts']+=1
                try:
                    r=session.get(args.url+'/api/status',timeout=4);r.raise_for_status();before=r.json()
                    if before.get('stream_id')!=initial_stream:raise RuntimeError('Camera stream restarted; stop batch')
                    if before.get('enabled'):raise RuntimeError('Detection re-enabled; reject annotated snapshots')
                    if before.get('source')!='camera' or not before.get('frame_fresh') or before.get('error'):
                        raise ValueError('Camera frame is not fresh')
                    r=session.get(args.url+'/snapshot.jpg',timeout=4);r.raise_for_status();payload=r.content
                    pixels=cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_COLOR)
                    if pixels is None or pixels.shape!=(480,640,3):raise ValueError('Unexpected snapshot dimensions')
                    r=session.get(args.url+'/api/status',timeout=4);r.raise_for_status();after=r.json()
                    if after.get('enabled') or after.get('detections'):
                        raise RuntimeError('Detection results appeared; reject snapshot')
                    if after.get('stream_id')!=initial_stream:raise RuntimeError('Stream changed')
                    if not after.get('frame_fresh'):raise ValueError('Camera frame became stale')
                    digest=hashlib.sha256(payload).hexdigest()
                    if digest in seen:state['skipped']+=1;state['last_skip']='Exact duplicate'
                    else:
                        candidate=validate_candidate(pixels)
                        gray=cv2.resize(cv2.cvtColor(candidate,cv2.COLOR_BGR2GRAY),(64,48))
                        delta=None if previous_gray is None else float(np.abs(gray.astype(float)-previous_gray.astype(float)).mean())
                        name=f"frame_{state['saved']+1:05d}"
                        (originals/(name+'.jpg')).write_bytes(payload)
                        ok,png=cv2.imencode('.png',candidate)
                        if not ok:raise RuntimeError('PNG encode failed')
                        (images/(name+'.png')).write_bytes(png.tobytes())
                        metadata={'image':f'images/{name}.png','original':f'preview_original/{name}.jpg',
                            'pc_time':now().isoformat(),'jpeg_sha256':digest,'crop_xyxy':[0,32,640,480],
                            'status_before':before,'status_after':after,
                            'sequence_note':'Status queries bracket snapshot; exact snapshot sequence is not provided by API',
                            'mean_gray_change_from_previous':delta,'near_duplicate_candidate':delta is not None and delta<2,
                            'laplacian_variance':float(cv2.Laplacian(cv2.cvtColor(candidate,cv2.COLOR_BGR2GRAY),cv2.CV_64F).var()),
                            'labels_generated':False}
                        log.write(json.dumps(metadata,ensure_ascii=False)+'\n');log.flush()
                        previous_gray=gray;seen.add(digest);state['saved']+=1
                        state['last_image']=str(images/(name+'.png'));state.pop('last_error',None)
                except ValueError as exc:
                    state['skipped']+=1;state['last_error']=str(exc)
                except requests.RequestException as exc:
                    state['skipped']+=1;state['last_error']=str(exc)
                write_json(state_path,state)
                remaining=args.interval-(time.monotonic()-tick)
                while remaining>0 and not stop.exists():
                    time.sleep(min(.1,remaining));remaining=args.interval-(time.monotonic()-tick)
        state.setdefault('stop_reason','frame_limit' if state['saved']==args.frames else 'time_limit')
    except BaseException as exc:
        state['error']=str(exc);state['stop_reason']='error'
    finally:
        if args.restore_enabled:
            try:
                r=session.get(args.url+'/api/status',timeout=4);r.raise_for_status()
                if r.json().get('stream_id')==initial_stream:
                    r=session.post(args.url+'/api/detection',json={'enabled':True},timeout=4);r.raise_for_status()
                    state['detection_restored']=r.json().get('enabled') is True
                else:state['detection_restore_skipped']='Stream changed'
            except Exception as exc:state['restore_error']=str(exc)
        state['active']=False;state['finished_at']=now().isoformat()
        write_json(state_path,state);write_json(folder/'session.json',state)
        (folder/'README.txt').write_text(
            'images/: 640x448 training candidates; 32px preview status bar cropped.\n'
            'preview_original/: original 640x480 JPEG previews, with status bar; NOT training inputs.\n'
            'No automatic labels. Blur/near-duplicate scores are metadata only; no blurred frames removed.\n'
            'Keep entire session in a single train/validation/test split.\n'
            'Jetson clock is not synchronized; use pc_time and session folder for collection dates.\n',encoding='utf-8')

if __name__=='__main__':main()
