"""Portable still-image inference demo. Colors: Person blue, interference orange, hazardous red."""
import argparse
import json
import os
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True, help='Image file or folder; relative to current working directory')
    parser.add_argument('--weights', default=str(Path(__file__).resolve().parent/'best.pt'))
    parser.add_argument('--output', default='predictions')
    parser.add_argument('--device', default='cpu', help='cpu or 0 for the first NVIDIA GPU')
    parser.add_argument('--conf', type=float, default=0.25)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('YOLO_CONFIG_DIR', str(output/'.runtime/ultralytics'))
    os.environ.setdefault('MPLCONFIGDIR', str(output/'.runtime/matplotlib'))
    from ultralytics import YOLO
    import cv2
    import numpy as np
    source = Path(args.source)
    supported = {'.jpg','.jpeg','.png','.bmp','.webp'}
    files = sorted(p for p in source.iterdir() if p.suffix.lower() in supported) if source.is_dir() else [source]
    if not files or any(not p.is_file() for p in files):
        raise FileNotFoundError(f'No input images: {source}')
    model = YOLO(args.weights)
    expected = {0:'Person',1:'Interference',2:'Hazardous'}
    if model.names != expected:
        raise ValueError(f'Unexpected class mapping: {model.names}')
    records = []
    for path in files:
        image = cv2.imdecode(np.fromfile(str(path),dtype=np.uint8),cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'Unreadable image: {path}')
        result = model.predict(image, imgsz=640, conf=args.conf, device=args.device, verbose=False)[0]
        objects = []
        for box in result.boxes:
            cls = int(box.cls.item())
            confidence = float(box.conf.item())
            coords = list(map(float,box.xyxy[0].tolist()))
            objects.append({'class_id':cls,'name':expected[cls],'confidence':confidence,'xyxy_pixels':coords})
            x1,y1,x2,y2 = map(int,coords)
            color = {0:(255,0,0),1:(0,165,255),2:(0,0,255)}[cls]
            cv2.rectangle(image,(x1,y1),(x2,y2),color,3)
            cv2.putText(image,f'{expected[cls]} {confidence:.2f}',(x1,max(20,y1-8)),cv2.FONT_HERSHEY_SIMPLEX,.7,color,2)
        target = output/f'{path.stem}_pred.jpg'
        ok, encoded = cv2.imencode('.jpg',image)
        if not ok:
            raise RuntimeError(f'Cannot encode result: {path}')
        encoded.tofile(str(target))
        records.append({'source':str(path),'preview':str(target),'objects':objects})
    (output/'results.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'images':len(records),'objects':sum(len(r['objects']) for r in records),'output':str(output)},ensure_ascii=False))

if __name__ == '__main__':
    main()
