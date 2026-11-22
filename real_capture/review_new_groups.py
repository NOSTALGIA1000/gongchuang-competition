"""Frozen new-capture proposals and visual audit sheets. Never edits originals."""
from pathlib import Path
import json
import cv2
import numpy as np
from annotate_camera_roll import read, proposals

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'annotation_review/new_13groups'

def save(path,img):
    cv2.imencode('.jpg',img,[cv2.IMWRITE_JPEG_QUALITY,94])[1].tofile(str(path))

def main():
    groups=json.loads((OUT/'groups.json').read_text(encoding='utf8'))
    records=[]
    for group in groups:
        for src in group['files']:
            img=read(src); h,w=img.shape[:2]; boxes=[list(map(int,b)) for b in proposals(img)]
            records.append(dict(index=len(records)+1,batch=group['batch'],source=src,width=w,height=h,boxes=boxes))
    (OUT/'proposals.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf8')
    # Full scene and enlarged crops for every frame, including clipped proposals.
    for page,start in enumerate(range(0,len(records),48),1):
        sheet=np.full((6*250,8*250,3),245,np.uint8)
        for j,r in enumerate(records[start:start+48]):
            img=read(r['source']); x=j%8*250;y=j//8*250
            scene=img.copy()
            for k,b in enumerate(r['boxes']):
                a,c,d,e=b;cv2.rectangle(scene,(a,c),(d,e),(0,0,255),2)
            sheet[y:y+135,x:x+240]=cv2.resize(scene,(240,135))
            for k,b in enumerate(r['boxes'][:3]):
                a,c,d,e=b;crop=img[c:e,a:d];hh,ww=crop.shape[:2]
                sc=min(76/ww,76/hh);crop=cv2.resize(crop,(max(1,round(ww*sc)),max(1,round(hh*sc))))
                sheet[y+139:y+139+crop.shape[0],x+k*80:x+k*80+crop.shape[1]]=crop
            cv2.putText(sheet,f"{r['index']:04d} G{r['batch']:02d} n={len(r['boxes'])}",(x+2,y+238),cv2.FONT_HERSHEY_SIMPLEX,.47,(0,0,0),1)
        save(OUT/f'audit_{page:02d}.jpg',sheet)
    # Reference candidates: 3 complete frames per group.
    selected=[]
    for g in groups:
        rs=[r for r in records if r['batch']==g['batch'] and len(r['boxes'])==2 and all(b[1]>5 and b[3]<r['height']-5 for b in r['boxes'])]
        for r in rs[::max(1,len(rs)//3)][:3]:
            for k,b in enumerate(r['boxes']):selected.append((r,k,b))
    sheet=np.full((((len(selected)+9)//10)*150,1500,3),245,np.uint8)
    for j,(r,k,b) in enumerate(selected):
        img=read(r['source']);a,c,d,e=b;crop=cv2.resize(img[c:e,a:d],(115,115));x=j%10*150;y=j//10*150
        sheet[y:y+115,x:x+115]=crop
        cv2.putText(sheet,f"{r['index']}:{k} G{r['batch']}",(x,y+135),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),1)
    save(OUT/'references.jpg',sheet)
    print('frames',len(records),'proposals',sum(len(r['boxes']) for r in records),'sheets',page)

if __name__=='__main__': main()
