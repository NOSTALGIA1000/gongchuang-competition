"""Two-sign dataset annotation from observed yellow triangles, with audit images."""
from pathlib import Path
import json
import hashlib
import shutil
import cv2
import numpy as np

BASE=Path(__file__).resolve().parent
REVIEW=BASE/'annotation_review'

def read(path):
    image=cv2.imdecode(np.frombuffer(Path(path).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    if image is None:raise ValueError(f'Unreadable image: {path}')
    return image

def proposals(image):
    h,w=image.shape[:2]
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([12,65,45]),np.array([42,255,255]))
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    n,labels,stats,_=cv2.connectedComponentsWithStats(mask)
    boxes=[]
    for lab in range(1,n):
        x,y,bw,bh,area=stats[lab]
        if area<100 or bw<18 or bh<12 or bw>400 or bh>350:continue
        if not .3<bw/max(bh,1)<3.0 or area/(bw*bh)<.12:continue
        pad=max(3,round(max(bw,bh)*.12))
        x0=max(0,x-pad);y0=max(0,y-pad);x1=min(w,x+bw+pad);y1=min(h,y+bh+pad)
        gray=cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2GRAY)
        dark=cv2.inRange(gray,0,100)
        dark=cv2.morphologyEx(dark,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
        contours,_=cv2.findContours(dark,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        candidates=[]
        for c in contours:
            cx,cy,cw,ch=cv2.boundingRect(c)
            gx,gy=x0+cx,y0+cy
            ix=max(0,min(gx+cw,x+bw)-max(gx,x));iy=max(0,min(gy+ch,y+bh)-max(gy,y))
            coverage=ix*iy/(bw*bh)
            if coverage>.7 and cv2.contourArea(c)>.24*bw*bh:candidates.append((cv2.contourArea(c),(gx,gy,gx+cw,gy+ch)))
        if candidates:box=max(candidates)[1]
        else:
            border=max(3,round(max(bw,bh)*.045))
            box=(max(0,x-border),max(0,y-border),min(w,x+bw+border),min(h,y+bh+border))
        border=max(3,round(max(bw,bh)*.035))
        box=(min(box[0],max(0,x-border)),min(box[1],max(0,y-border)),
             max(box[2],min(w,x+bw+border)),max(box[3],min(h,y+bh+border)))
        if any(intersection_over_union(box,b)>.5 for b in boxes):continue
        boxes.append(box)
    return sorted(boxes,key=lambda b:b[1])

def intersection_over_union(a,b):
    i=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    return i/max(1,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-i)

def feature(image,box):
    x0,y0,x1,y1=box
    crop=cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2GRAY)
    return cv2.resize(crop,(64,64)).astype(np.float32)

def correlate(a,b):
    return float(cv2.matchTemplate(a,b,cv2.TM_CCOEFF_NORMED)[0,0])

def printed_sign_ratio(image,box):
    """Use observed glyph size relative to its physical red board, never y order."""
    x0,y0,x1,y1=map(int,box);h,w=image.shape[:2]
    cx=(x0+x1)//2;cy=(y0+y1)//2;span=max(80,(x1-x0)*2)
    xa=max(0,cx-span);xb=min(w,cx+span)
    roi=image[max(0,cy-5):min(h,cy+6),xa:xb]
    hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    red=((hsv[:,:,0]<12)|(hsv[:,:,0]>170))&(hsv[:,:,1]>65)&(hsv[:,:,2]>55)
    xs=np.where(red.sum(axis=0)>=2)[0]+xa
    left=xs[xs<x0-4];right=xs[xs>x1+4]
    if not len(left) or not len(right):return None
    width=int(right.min()-left.max())
    if width<70:return None
    return float((x1-x0)/width)

def main():
    inventory=json.loads((REVIEW/'inventory.json').read_text(encoding='utf8'))
    reference=read(inventory[1]['path']);ref_boxes=proposals(reference)
    assert len(ref_boxes)==2,ref_boxes
    # These two reference glyphs are visually checked, independent of screen orientation.
    templates={2:feature(reference,ref_boxes[0]),1:feature(reference,ref_boxes[1])}
    for cls,box in [(2,ref_boxes[0]),(1,ref_boxes[1])]:
        x0,y0,x1,y1=box
        cv2.imencode('.png',reference[y0:y1,x0:x1])[1].tofile(str(REVIEW/f'reference_class_{cls}.png'))
    out=BASE/'datasets/camera_roll_20261114_batch01_reviewed'
    for folder in ['images','labels','review_images','review_labels','qa']:(out/folder).mkdir(parents=True,exist_ok=True)
    results=[]
    for index,item in enumerate(inventory,1):
        image=read(item['path']);h,w=image.shape[:2];boxes=proposals(image);objects=[];issues=[]
        for box in boxes:
            f=feature(image,box);scores={c:correlate(f,t) for c,t in templates.items()};cls=max(scores,key=scores.get)
            margin=scores[cls]-scores[3-cls]
            ratio=printed_sign_ratio(image,box)
            # This fixture-specific size distinction is checked on every QA page.
            # It is not a general model and must not be used for new sign batches.
            if ratio is not None:cls=2 if ratio>.49 else 1
            x0,y0,x1,y1=box
            clipped=bool(x0<3 or y0<3 or x1>w-3 or y1>h-3)
            if ratio is None or .46<ratio<.52 or clipped:issues.append(f'uncertain_or_clipped:{box}')
            objects.append({'class_id':cls,'box_xyxy':list(map(int,box)),'scores':scores,'margin':margin,'clipped':clipped,'triangle_to_board_width':ratio})
        if not objects:issues.append('no_clear_triangle_proposed')
        if len(objects)>2:issues.append('more_than_two_proposals')
        if len(objects)==2 and len({o['class_id'] for o in objects})<2:issues.append('duplicate_class_pair')
        stem=f'capture_{index:04d}';quality='review' if issues else 'ready'
        image_folder='review_images' if issues else 'images';label_folder='review_labels' if issues else 'labels'
        shutil.copy2(item['path'],out/image_folder/(stem+'.jpg'))
        lines=[]
        for o in objects:
            x0,y0,x1,y1=o['box_xyxy']
            lines.append(f"{o['class_id']} {(x0+x1)/2/w:.8f} {(y0+y1)/2/h:.8f} {(x1-x0)/w:.8f} {(y1-y0)/h:.8f}")
        (out/label_folder/(stem+'.txt')).write_text('\n'.join(lines)+('\n' if lines else ''),encoding='utf8')
        results.append({'index':index,'stem':stem,'source':item['path'],'sha256':hashlib.sha256(Path(item['path']).read_bytes()).hexdigest(),
                        'quality':quality,'issues':issues,'objects':objects})
    (out/'manifest.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf8')
    (out/'classes.txt').write_text('Person\nInterference\nHazardous\n',encoding='utf8')
    (out/'dataset.yaml').write_text('path: '+out.as_posix()+'\ntrain: images\nnames:\n  0: Person\n  1: Interference\n  2: Hazardous\n',encoding='utf8')
    for page,start in enumerate(range(0,len(results),24),1):
        sheet=np.full((4*240,6*300,3),245,np.uint8)
        for j,result in enumerate(results[start:start+24]):
            image=read(result['source']);ih,iw=image.shape[:2]
            for o in result['objects']:
                x0,y0,x1,y1=o['box_xyxy'];color={0:(255,0,0),1:(0,165,255),2:(0,0,255)}[o['class_id']]
                cv2.rectangle(image,(x0,y0),(x1,y1),color,2)
                cv2.putText(image,'H' if o['class_id']==2 else 'I',(x0,max(12,y0-4)),cv2.FONT_HERSHEY_SIMPLEX,.7,color,2)
            thumb=cv2.resize(image,(290,163));x=j%6*300;y=j//6*240
            sheet[y:y+163,x:x+290]=thumb
            cv2.putText(sheet,f"{result['index']:04d} {result['quality']} n={len(result['objects'])}",(x+3,y+195),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),1)
        cv2.imencode('.jpg',sheet,[cv2.IMWRITE_JPEG_QUALITY,93])[1].tofile(str(out/'qa'/f'labels_{page:02d}.jpg'))
    summary={'total':len(results),'ready':sum(r['quality']=='ready' for r in results),'review':sum(r['quality']=='review' for r in results),
             'ready_objects':{str(c):sum(o['class_id']==c for r in results if r['quality']=='ready' for o in r['objects']) for c in [1,2]}}
    print(json.dumps(summary));print('review ids',[r['index'] for r in results if r['quality']=='review'])
    (out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf8')

if __name__=='__main__':main()
