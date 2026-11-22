"""Train native YOLOv5s with local-only logging and persisted per-epoch status."""
from pathlib import Path
import os,sys,time,json,traceback,math,argparse

BASE=Path(__file__).resolve().parent
WORK=BASE/'training/targets_v3_yolov5s'
REPO=BASE.parent/'third_party/yolov5'
for k,v in {'PYTHONUTF8':'1','PYTHONIOENCODING':'utf-8','YOLO_CONFIG_DIR':str(WORK/'ultralytics_config'),
            'MPLCONFIGDIR':str(WORK/'matplotlib_config'),'YOLO_AUTOINSTALL':'false','WANDB_MODE':'disabled',
            'COMET_MODE':'DISABLED','CLEARML_OFFLINE_MODE':'1','WANDB_DISABLED':'true'}.items():os.environ[k]=v
for optional in ['wandb','comet_ml','clearml']:sys.modules[optional]=None
sys.path.insert(0,str(REPO))

def atomic_json(path,data):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8');temp.replace(path)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--batch',type=int,default=16);parser.add_argument('--resume',action='store_true');args=parser.parse_args()
    import torch,psutil,importlib.metadata
    import train
    from utils.callbacks import Callbacks
    import utils.loggers as loggers
    import utils.dataloaders as dataloaders
    from multiprocessing.pool import ThreadPool
    # Windows process-based label scanning duplicates Python/Torch across eight
    # workers. Verification is read-only IO, so use two threads on this 16GB host.
    dataloaders.NUM_THREADS=2
    dataloaders.Pool=lambda processes:ThreadPool(2)
    # Disable optional uploads and TensorBoard graph tracing; keep native CSV and plots.
    original=loggers.Loggers.__init__
    def local_loggers(self,*a,**kw):
        kw['include']=('csv',);original(self,*a,**kw)
    loggers.Loggers.__init__=local_loggers
    assert torch.cuda.is_available()
    torch.set_num_threads(4)
    split=json.loads((WORK/'split_manifest.json').read_text(encoding='utf8'))
    started=time.time();history=[]
    state={'status':'initializing','model':'native YOLOv5s','classes':['Person','Hazardous'],
           'pid':os.getpid(),'epochs_max':200,'patience':40,'batch':args.batch,'imgsz':640,
           'workers':0,'cache':False,'optimizer':'SGD','lr0':.005,'seed':42,
           'torch':torch.__version__,'gpu':torch.cuda.get_device_name(0),'source':str(BASE/'datasets/targets_2class_20261115_v1'),
           'split':split['summary'],'repo_commit':split['repo_commit'],'pretrained_sha256':split['pretrained_sha256'],
           'limitations':split['limitations'],'rotation_degrees':0,'flipud':0,'fliplr':0,
           'started_at_unix':started,'completed_epoch':0,'host_available_gib_at_start':round(psutil.virtual_memory().available/2**30,2),
           'package_versions':{k:importlib.metadata.version(k) for k in ['torch','torchvision','ultralytics','numpy','opencv-python','seaborn','ultralytics-thop']}}
    atomic_json(WORK/'run_info.json',state)
    callbacks=Callbacks()
    def epoch_end(log_vals,epoch,best_fitness,fi):
        values=[float(v) for v in log_vals]
        assert all(math.isfinite(v) for v in values),f'Nonfinite loss/metric at epoch {epoch+1}: {values}'
        row={'epoch':epoch+1,'elapsed_seconds':round(time.time()-started,1),'train_box_loss':values[0],
             'train_obj_loss':values[1],'train_cls_loss':values[2],'precision':values[3],'recall':values[4],
             'map50':values[5],'map50_95':values[6],'fitness':float(fi),'best_fitness':float(best_fitness)}
        history.append(row);state.update(status='training',completed_epoch=epoch+1,last_metrics=row)
        atomic_json(WORK/'run_info.json',state)
        with (WORK/'epoch_history.jsonl').open('a',encoding='utf8') as f:f.write(json.dumps(row)+'\n')
        print('EPOCH_STATUS '+json.dumps(row),flush=True)
    callbacks.register_action('on_fit_epoch_end',name='persist_status',callback=epoch_end)
    opt=train.parse_opt(True)
    settings=dict(weights=str(WORK/'yolov5s_coco_v7.pt'),data=str(WORK/'data.yaml'),hyp=str(WORK/'hyp.yaml'),
                  epochs=200,batch_size=args.batch,imgsz=640,device='0',workers=0,cache=False,
                  optimizer='SGD',cos_lr=True,patience=40,seed=42,save_period=25,
                  project=str(WORK),name='run',exist_ok=True,noval=False,noplots=False)
    if not args.resume:
        assert not (WORK/'run/weights/last.pt').exists(),'Existing training checkpoint: use --resume instead of overwriting.'
    if args.resume:settings['resume']=str(WORK/'run/weights/last.pt')
    for k,v in settings.items():setattr(opt,k,v)
    atomic_json(WORK/'requested_settings.json',settings)
    try:
        train.main(opt,callbacks)
        best=WORK/'run/weights/best.pt';last=WORK/'run/weights/last.pt'
        assert best.exists() and last.exists()
        state.update(status='completed',elapsed_seconds=round(time.time()-started,1),weights=str(best),last_weights=str(last))
        atomic_json(WORK/'run_info.json',state)
        print('TRAINING_COMPLETE '+json.dumps(state,ensure_ascii=False),flush=True)
    except BaseException as e:
        state.update(status='failed',elapsed_seconds=round(time.time()-started,1),error=repr(e),traceback=traceback.format_exc())
        atomic_json(WORK/'run_info.json',state);raise

if __name__=='__main__':main()
