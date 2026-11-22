"""Assist human review using visually identified glyph references, not deployed classes."""
from pathlib import Path
import json, cv2, numpy as np
from annotate_camera_roll import read

OUT=Path(__file__).resolve().parent/'annotation_review/new_13groups'
REFS={'toxic':[(55,1),(84,1),(968,1),(1022,1),(1082,0),(1148,0),(1289,0),(1380,1)],
      'fire':[(135,1),(196,1),(253,0),(289,0),(751,1),(851,0),(913,0)],
      'explosion':[(353,1),(387,1),(460,0),(478,0),(536,0),(624,1),(682,1)],
      'ignore':[(55,0),(84,0),(135,0),(196,0),(253,1),(353,0),(478,1),(536,1),(624,0),(751,0),(913,1),(968,0),(1082,1),(1289,1)]}

def merge(boxes):
    boxes=[list(b) for b in boxes]
    changed=True
    while changed:
        changed=False
        for i,a in enumerate(boxes):
            for j in range(i+1,len(boxes)):
                b=boxes[j]
                ix=min(a[2],b[2])-max(a[0],b[0]);iy=min(a[3],b[3])-max(a[1],b[1])
                if ix>=-6 and iy>min(a[3]-a[1],b[3]-b[1])*.45:
                    boxes[i]=[min(a[0],b[0]),min(a[1],b[1]),max(a[2],b[2]),max(a[3],b[3])];boxes.pop(j);changed=True;break
            if changed:break
    return sorted(boxes,key=lambda b:b[1])

def feat(crop):
    hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([12,60,40]),np.array([42,255,255]))
    pts=cv2.findNonZero(mask)
    tri=None
    if pts is not None and len(pts)>20:
        hull=cv2.convexHull(pts);peri=cv2.arcLength(hull,True)
        for eps in np.linspace(.015,.18,30):
            p=cv2.approxPolyDP(hull,eps*peri,True).reshape(-1,2)
            if len(p)==3:
                tri=p.astype(np.float32);break
    if tri is not None:
        bottom=int(tri[:,1].argmax());p=tri[bottom];top=np.delete(tri,bottom,axis=0);top=top[np.argsort(top[:,0])]
        src=np.array([top[0],top[1],p],np.float32)
        m=cv2.getAffineTransform(src,np.array([[2,2],[93,2],[48,93]],np.float32))
        crop=cv2.warpAffine(crop,m,(96,96),borderMode=cv2.BORDER_REPLICATE)
    else:crop=cv2.resize(crop,(96,96))
    g=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
    g=cv2.resize(g,(48,48)).astype(np.float32)
    yy,xx=np.mgrid[:48,:48]
    inside=(yy>5)&(yy<41)&(xx>yy*.5+4)&(xx<44-yy*.5)
    g=g[inside];g-=g.mean();return (g/(np.linalg.norm(g)+1e-8)).ravel()

def main():
    rs=json.loads((OUT/'proposals.json').read_text(encoding='utf8'))
    bank=[];types=[]
    for name,refs in REFS.items():
        for idx,k in refs:
            r=rs[idx-1];im=read(r['source']);a,b,c,d=r['boxes'][k];crop=im[b:d,a:c]
            for angle in [0]:
                h,w=crop.shape[:2];m=cv2.getRotationMatrix2D((w/2,h/2),angle,1)
                rot=cv2.warpAffine(crop,m,(w,h),borderMode=cv2.BORDER_REPLICATE)
                bank.append(feat(rot));types.append(name)
    bank=np.array(bank)
    for r in rs:
        im=read(r['source']);objs=[]
        for b in merge(r['boxes']):
            x,y,x1,y1=b; scores=bank@feat(im[y:y1,x:x1])
            bytype={t:float(max(scores[k] for k,n in enumerate(types) if n==t)) for t in REFS}
            name=max(bytype,key=bytype.get); vals=sorted(bytype.values(),reverse=True)
            objs.append(dict(box_xyxy=b,glyph=name,score=bytype[name],margin=vals[0]-vals[1],scores=bytype))
        r['candidates']=objs
    (OUT/'classified.json').write_text(json.dumps(rs,ensure_ascii=False,indent=2),encoding='utf8')
    # Review every candidate, sorted by group/frame. Numbers identify global frame/candidate.
    items=[(r,k,o) for r in rs for k,o in enumerate(r['candidates'])]
    for page,start in enumerate(range(0,len(items),160),1):
        sheet=np.full((10*140,16*125,3),245,np.uint8)
        for j,(r,k,o) in enumerate(items[start:start+160]):
            im=read(r['source']);a,b,c,d=o['box_xyxy'];crop=cv2.resize(im[b:d,a:c],(100,100));x=j%16*125;y=j//16*140
            sheet[y:y+100,x:x+100]=crop
            color=(0,0,200) if o['glyph']!='ignore' else (70,70,70)
            cv2.putText(sheet,f"{r['index']}:{k} {o['glyph'][:3]}",(x,y+115),cv2.FONT_HERSHEY_SIMPLEX,.42,color,1)
            cv2.putText(sheet,f"{o['score']:.2f}/{o['margin']:.2f}",(x,y+132),cv2.FONT_HERSHEY_SIMPLEX,.39,color,1)
        cv2.imencode('.jpg',sheet,[cv2.IMWRITE_JPEG_QUALITY,94])[1].tofile(str(OUT/f'crop_review_{page:02d}.jpg'))
    print('candidates',len(items),'crop sheets',page)

if __name__=='__main__':main()
