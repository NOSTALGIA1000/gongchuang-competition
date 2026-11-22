"""Propose printed Person picture bounds; retain full audit sheets for visual review."""
from pathlib import Path
import json
import hashlib
import shutil
import cv2
import numpy as np

BASE = Path(__file__).resolve().parent
REVIEW = BASE/'annotation_review/person_new'
OUT = BASE/'datasets/person_20261114_batch01'

def read(path):
    return cv2.imdecode(np.fromfile(str(path),dtype=np.uint8),cv2.IMREAD_COLOR)

def propose(im):
    h,w = im.shape[:2]
    # All source scenes were inspected: one printed face near the image center.
    # Search constraints are fixture-specific annotation aids, not a detector.
    xoff,yoff = int(w*.25),int(h*.12)
    roi=im[yoff:int(h*.91),xoff:int(w*.76)]
    gray=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
    mask=cv2.inRange(gray,0,180)
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    candidates=[]
    for cnt in contours:
        x,y,bw,bh=cv2.boundingRect(cnt)
        area=cv2.contourArea(cnt)
        if not(35<=bw<=210 and 45<=bh<=220 and .4<bw/bh<1.65 and area>600):continue
        if area/(bw*bh)<.35:continue
        # Printed picture has white paper around it; reject floor or footwear.
        gx,gy=x+xoff,y+yoff
        if float(np.mean(gray[y:y+bh,x:x+bw]<110))<.12:continue
        pad=8
        top=im[max(0,gy-pad):gy,gx:gx+bw]
        bottom=im[gy+bh:min(h,gy+bh+pad),gx:gx+bw]
        surround=np.concatenate([top.reshape(-1,3),bottom.reshape(-1,3)])
        brightness=float(np.median(surround))
        if brightness<165:continue
        candidates.append((area,(max(0,gx-2),max(0,gy-2),min(w,gx+bw+2),min(h,gy+bh+2))))
    if not candidates:return None
    return max(candidates)[1]

def main():
    paths=json.loads((REVIEW/'sources.json').read_text(encoding='utf-8'))
    for folder in ['images','labels','qa']:(OUT/folder).mkdir(parents=True,exist_ok=True)
    records=[]
    for i,source in enumerate(paths,1):
        im=read(source); h,w=im.shape[:2]; box=propose(im)
        stem=f'person_{i:04d}'
        record={'index':i,'stem':stem,'source':source,'box_xyxy':box,
                'capture_group':'person_same_floor_20261114_1404_1406',
                'sha256':hashlib.sha256(Path(source).read_bytes()).hexdigest()}
        records.append(record)
        if box:
            shutil.copy2(source,OUT/'images'/f'{stem}.jpg')
            x1,y1,x2,y2=box
            (OUT/'labels'/f'{stem}.txt').write_text(
                f'0 {(x1+x2)/2/w:.8f} {(y1+y2)/2/h:.8f} {(x2-x1)/w:.8f} {(y2-y1)/h:.8f}\n',encoding='utf-8')
    (OUT/'manifest.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    (OUT/'classes.txt').write_text('Person\nInterference\nHazardous\n',encoding='utf-8')
    for start in range(0,len(records),30):
        sheet=np.full((5*200,6*300,3),245,np.uint8)
        crops=np.full((5*180,6*200,3),245,np.uint8)
        for j,r in enumerate(records[start:start+30]):
            im=read(r['source']); box=r['box_xyxy']
            if box:
                x1,y1,x2,y2=box
                crop=im[max(0,y1-15):y2+15,max(0,x1-15):x2+15].copy()
                cv2.rectangle(crop,(x1-max(0,x1-15),y1-max(0,y1-15)),
                              (x2-max(0,x1-15),y2-max(0,y1-15)),(255,0,0),2)
                ch,cw=crop.shape[:2]; ratio=min(180/cw,145/ch)
                crop=cv2.resize(crop,(int(cw*ratio),int(ch*ratio)))
                xx=(j%6)*200; yy=(j//6)*180
                crops[yy:yy+crop.shape[0],xx:xx+crop.shape[1]]=crop
                cv2.rectangle(im,(x1,y1),(x2,y2),(255,0,0),3)
                cv2.putText(im,'0 Person',(x1,max(20,y1-6)),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,0,0),2)
                if r['index'] in [1,73,151,246]:
                    cv2.imencode('.jpg',im)[1].tofile(str(OUT/'qa'/f'example_{r["index"]:04d}.jpg'))
            thumb=cv2.resize(im,(300,169)); xx=(j%6)*300; yy=(j//6)*200
            sheet[yy:yy+169,xx:xx+300]=thumb
            text=f'{r["index"]:03d} '+('Person' if box else 'REVIEW')
            cv2.putText(sheet,text,(xx+3,yy+190),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,0,0) if box else (0,0,255),1)
            cv2.putText(crops,text,((j%6)*200+3,(j//6)*180+169),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,0,0),1)
        page=start//30+1
        cv2.imencode('.jpg',sheet,[cv2.IMWRITE_JPEG_QUALITY,94])[1].tofile(str(OUT/'qa'/f'scene_{page:02d}.jpg'))
        cv2.imencode('.jpg',crops,[cv2.IMWRITE_JPEG_QUALITY,95])[1].tofile(str(OUT/'qa'/f'crops_{page:02d}.jpg'))
    print(json.dumps({'total':len(records),'proposed':sum(bool(r['box_xyxy']) for r in records),
                      'missing':[r['index'] for r in records if not r['box_xyxy']]},ensure_ascii=False))

if __name__=='__main__':main()
