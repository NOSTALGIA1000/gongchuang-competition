"""Local smoke training; validation deliberately measures training-set fit only."""
from pathlib import Path
import os
import json
import shutil
import time

ROOT = Path('F:/工创比赛')
WORK = ROOT / 'real_capture/training/trial_v1'
WORK.mkdir(parents=True, exist_ok=True)
os.environ['YOLO_CONFIG_DIR'] = str(WORK / 'ultralytics_config')
os.environ['MPLCONFIGDIR'] = str(WORK / 'matplotlib_config')
os.environ['YOLO_AUTOINSTALL'] = 'false'

def main():
    import requests
    import torch
    import ultralytics
    from ultralytics import YOLO, settings
    integrations = ('sync', 'wandb', 'comet', 'mlflow', 'clearml', 'neptune', 'tensorboard')
    settings.update({key: False for key in integrations if key in settings})
    assert torch.cuda.is_available(), 'CUDA required for this run'
    source = ROOT / 'real_capture/datasets/camera_roll_20261114_final_v1'
    data_dir = WORK / 'dataset'
    for folder in ('images', 'labels'):
        (data_dir / folder).mkdir(parents=True, exist_ok=True)
        for p in (source / folder).iterdir():
            if p.is_file() and p.name != 'classes.txt':
                shutil.copy2(p, data_dir / folder / p.name)
    config = WORK / 'data_trial.yaml'
    config.write_text(f'path: {data_dir.as_posix()}\ntrain: images\nval: images\n'
                      'names:\n  0: Person\n  1: Interference\n  2: Hazardous\n', encoding='utf-8')
    weight = WORK / 'yolov8n.pt'
    if not weight.exists():
        url = 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt'
        response = requests.get(url, timeout=90)
        response.raise_for_status()
        weight.write_bytes(response.content)
    info = {'purpose': 'Pipeline smoke test and training-set fit; NOT independent validation',
            'model': 'YOLOv8n', 'epochs': 30, 'imgsz': 640, 'batch': 8,
            'train_images': 130, 'validation': 'same 130 training images; resubstitution only',
            'missing_classes': ['Person'], 'torch': torch.__version__,
            'ultralytics': ultralytics.__version__, 'gpu': torch.cuda.get_device_name(0)}
    (WORK/'run_info.json').write_text(json.dumps(info, ensure_ascii=False, indent=2), encoding='utf-8')
    model = YOLO(str(weight))
    start = time.time()
    model.train(data=str(config), epochs=30, imgsz=640, batch=8, device=0,
                workers=0, project=str(WORK), name='run', exist_ok=True,
                amp=False, cache=False, seed=42, deterministic=True,
                val=False, plots=True, save=True, patience=30, close_mosaic=5)
    run = WORK/'run'
    best = run/'weights/best.pt'
    last = run/'weights/last.pt'
    chosen = best if best.exists() else last
    trained = YOLO(str(chosen))
    sample_names = ['capture_0002.jpg', 'capture_0030.jpg', 'capture_0069.jpg',
                    'capture_0090.jpg', 'capture_0120.jpg', 'capture_0158.jpg']
    samples = [str(data_dir/'images'/n) for n in sample_names if (data_dir/'images'/n).exists()]
    predictions = trained.predict(samples, imgsz=640, device=0, conf=0.25, verbose=False)
    import cv2
    import numpy as np
    preview_dir = WORK/'predictions'
    preview_dir.mkdir(exist_ok=True)
    for result in predictions:
        image = result.orig_img.copy()
        for box in result.boxes:
            cls = int(box.cls.item())
            score = float(box.conf.item())
            x1,y1,x2,y2 = map(int,box.xyxy[0].tolist())
            color = {0:(255,0,0),1:(0,165,255),2:(0,0,255)}[cls]
            cv2.rectangle(image,(x1,y1),(x2,y2),color,3)
            cv2.putText(image,f'{result.names[cls]} {score:.2f}',(x1,max(20,y1-8)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.7,color,2)
        dest = preview_dir/Path(result.path).name
        ok, encoded = cv2.imencode('.jpg',image)
        assert ok
        encoded.tofile(str(dest))
    info.update({'status':'completed', 'elapsed_seconds':round(time.time()-start,1),
                 'weights':str(chosen),'prediction_examples':len(predictions)})
    (WORK/'run_info.json').write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8')
    (WORK/'README_试训说明.md').write_text(
        '# 本机试训完成\n\nYOLOv8n，640输入，30轮，130张训练图片。\n'
        '类别保持0 Person、1 Interference、2 Hazardous；本批没有Person。\n'
        'val与train使用同一批图片，仅用于流程和拟合检查。results.csv中的指标不是独立验证精度。\n'
        'predictions目录是训练图片上的模型预测，不是人工标注。红色危险品、橙色干扰、蓝色Person。\n'
        '正式训练需补齐类别并拍摄独立验证/测试场景。\n',encoding='utf-8')
    print('TRIAL_COMPLETE',json.dumps(info,ensure_ascii=False),flush=True)

if __name__ == '__main__':
    main()
