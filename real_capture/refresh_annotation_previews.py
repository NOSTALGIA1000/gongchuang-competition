"""Draw actual YOLO labels using the agreed competition colour palette."""
from pathlib import Path
import json
import hashlib
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent/'datasets'
PALETTE={0:(255,0,0),1:(0,165,255),2:(0,0,255)}
NAMES={0:'Person',1:'Interference',2:'Hazardous'}

def load(path):
    return cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)

def render(image,labels,large=False):
    image=image.copy();h,w=image.shape[:2]
    for line in labels.splitlines():
        cls,cx,cy,bw,bh=map(float,line.split());cls=int(cls)
        x0=round((cx-bw/2)*w);y0=round((cy-bh/2)*h)
        x1=round((cx+bw/2)*w);y1=round((cy+bh/2)*h)
        colour=PALETTE[cls]
        cv2.rectangle(image,(x0,y0),(x1,y1),colour,3 if large else 2)
        title=f'{cls}: {NAMES[cls]}' if large else NAMES[cls]
        scale=.7 if large else .55
        (tw,th),base=cv2.getTextSize(title,cv2.FONT_HERSHEY_SIMPLEX,scale,2)
        ty=max(th+7,y0-8)
        cv2.rectangle(image,(x0,ty-th-5),(min(w-1,x0+tw+8),ty+base),colour,-1)
        cv2.putText(image,title,(x0+4,ty),cv2.FONT_HERSHEY_SIMPLEX,scale,(255,255,255),2,cv2.LINE_AA)
    return image

def main():
    hashes={}
    for folder in ROOT.glob('camera_roll_20261114_batch01*'):
        manifest=folder/'manifest.json'
        if not manifest.exists():continue
        rows=json.loads(manifest.read_text(encoding='utf8'))
        for page,start in enumerate(range(0,len(rows),24),1):
            sheet=np.full((4*240,6*300,3),245,np.uint8)
            for j,row in enumerate(rows[start:start+24]):
                labels=folder/('review_labels' if row['quality']=='review' else 'labels')/(row['stem']+'.txt')
                hashes[str(labels)]=hashlib.sha256(labels.read_bytes()).hexdigest()
                preview=render(load(Path(row['source'])),labels.read_text(encoding='utf8'))
                x=j%6*300;y=j//6*240
                sheet[y:y+163,x:x+290]=cv2.resize(preview,(290,163))
                cv2.putText(sheet,f"{row['index']:04d} {row['quality']} n={len(row['objects'])}",(x+3,y+195),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),1)
            cv2.imencode('.jpg',sheet,[cv2.IMWRITE_JPEG_QUALITY,93])[1].tofile(str(folder/'qa'/f'labels_{page:02d}.jpg'))
        (folder/'preview_colors.json').write_text(json.dumps({NAMES[c]:{'RGB':list(reversed(PALETTE[c]))} for c in NAMES},indent=2),encoding='utf8')
    folder=ROOT/'camera_roll_20261114_batch01_reviewed'
    example=render(load(folder/'images/capture_0002.jpg'),(folder/'labels/capture_0002.txt').read_text(),True)
    for name in ['example_capture_0002.png','example_capture_0002_orange.png']:
        cv2.imencode('.png',example)[1].tofile(str(folder/'qa'/name))
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    print('Preview palettes updated; YOLO labels unchanged:',len(hashes))

if __name__=='__main__':main()
