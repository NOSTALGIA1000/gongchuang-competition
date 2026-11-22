"""Train all currently annotated classes; measure fitting only, not generalization."""
from pathlib import Path
from collections import Counter
import os
import json
import shutil
import time

ROOT=Path('F:/工创比赛/real_capture')
WORK=ROOT/'training/person_combined_v2'
WORK.mkdir(parents=True,exist_ok=True)
os.environ['YOLO_CONFIG_DIR']=str(WORK/'ultralytics_config')
os.environ['MPLCONFIGDIR']=str(WORK/'matplotlib_config')
os.environ['YOLO_AUTOINSTALL']='false'

def main():
    import torch
    import ultralytics
    from ultralytics import YOLO,settings
    import cv2
    assert torch.cuda.is_available()
    settings.update({k:False for k in ('sync','wandb','comet','mlflow','clearml','tensorboard') if k in settings})
    data=WORK/'dataset'
    counts=Counter()
    sources=['camera_roll_20261114_final_v1','person_20261114_batch01']
    for folder in ('images','labels'):(data/folder).mkdir(parents=True,exist_ok=True)
    for source in sources:
        src=ROOT/'datasets'/source
        for p in sorted((src/'images').glob('*.jpg')):
            label=src/'labels'/p.with_suffix('.txt').name
            assert label.exists()
            for line in label.read_text().splitlines():
                cls,x,y,w,h=map(float,line.split())
                assert cls in [0,1,2] and 0<=x-w/2<=x+w/2<=1 and 0<=y-h/2<=y+h/2<=1
                counts[int(cls)]+=1
            for original,dest in [(p,data/'images'/p.name),(label,data/'labels'/label.name)]:
                assert not dest.exists() or dest.read_bytes()==original.read_bytes()
                shutil.copy2(original,dest)
    assert len(list((data/'images').glob('*.jpg')))==376
    assert counts=={0:246,1:100,2:113}
    config=WORK/'data_trial.yaml'
    config.write_text(f'path: {data.as_posix()}\ntrain: images\nval: images\nnames:\n  0: Person\n  1: Interference\n  2: Hazardous\n',encoding='utf-8')
    weight=ROOT/'training/trial_v1/yolov8n.pt'
    assert weight.exists()
    info={'status':'training','model':'YOLOv8n COCO pretrained','epochs':60,'batch':16,'imgsz':640,
          'degrees':0,'train_images':376,'class_instances':dict(counts),
          'evaluation':'Train and val are the SAME images. Fitting check only, NOT independent validation.',
          'torch':torch.__version__,'ultralytics':ultralytics.__version__,
          'gpu':torch.cuda.get_device_name(0)}
    (WORK/'run_info.json').write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8')
    start=time.time()
    model=YOLO(str(weight))
    model.train(data=str(config),epochs=60,batch=16,imgsz=640,device=0,workers=0,
                project=str(WORK),name='run',exist_ok=True,amp=False,cache=False,
                seed=42,deterministic=True,val=False,plots=True,save=True,
                degrees=0,perspective=0,shear=0,fliplr=0,flipud=0,
                hsv_h=.005,hsv_s=.3,hsv_v=.3,mosaic=.5,close_mosaic=10)
    checkpoint=WORK/'run/weights/best.pt'
    trained=YOLO(str(checkpoint))
    assert trained.names=={0:'Person',1:'Interference',2:'Hazardous'}
    names=['person_0001.jpg','person_0073.jpg','person_0151.jpg','person_0183.jpg','person_0246.jpg',
           'capture_0002.jpg','capture_0090.jpg','capture_0120.jpg']
    output=WORK/'predictions';output.mkdir(exist_ok=True)
    predictions=[]
    for result in trained.predict([str(data/'images'/n) for n in names],imgsz=640,device=0,conf=.25,verbose=False):
        im=result.orig_img.copy();objects=[]
        for box in result.boxes:
            cls=int(box.cls.item());conf=float(box.conf.item());coords=list(map(int,box.xyxy[0].tolist()))
            x1,y1,x2,y2=coords;color={0:(255,0,0),1:(0,165,255),2:(0,0,255)}[cls]
            cv2.rectangle(im,(x1,y1),(x2,y2),color,3)
            cv2.putText(im,f'{trained.names[cls]} {conf:.2f}',(x1,max(20,y1-8)),cv2.FONT_HERSHEY_SIMPLEX,.7,color,2)
            objects.append({'class_id':cls,'confidence':conf,'box_xyxy':coords})
        cv2.imencode('.jpg',im)[1].tofile(str(output/Path(result.path).name))
        predictions.append({'image':Path(result.path).name,'objects':objects})
    (output/'predictions.json').write_text(json.dumps(predictions,ensure_ascii=False,indent=2),encoding='utf-8')
    info.update(status='completed',elapsed_seconds=round(time.time()-start,1),weights=str(checkpoint))
    (WORK/'run_info.json').write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8')
    (WORK/'README_试训说明.md').write_text('# 加入Person的三类试训\n\n376张、459个目标。Person246、Interference100、Hazardous113。\nYOLOv8n，60轮，batch16，640输入，随机旋转及翻转关闭。\n从官方COCO预训练权重训练，并保留旧危险品、干扰数据。\ntrain与val是同一批图片；最终回测指标仅衡量拟合，不是独立验证精度。\n预测预览为模型输出，Person蓝框、干扰橙框、危险品红框。\n旧试训结果保留。正式训练仍需补齐图样及独立拍摄验证场景。\n',encoding='utf-8')
    print('TRAINING_COMPLETE',json.dumps(info,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
