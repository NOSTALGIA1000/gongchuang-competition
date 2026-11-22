"""Refine triangle bounds and propagate visually audited reference glyphs for review."""
from pathlib import Path
import json,cv2,numpy as np
from annotate_camera_roll import read

OUT=Path(__file__).resolve().parent/'annotation_review/new_13groups'
HAZ={1:'toxic',2:'fire',3:'fire',4:'explosion',5:'explosion',6:'explosion',7:'explosion',8:'fire',9:'fire',10:'toxic',11:'toxic',12:'toxic',13:'toxic'}

def refine(im,b):
    a,y,c,d=b;h,w=im.shape[:2];pad=round(max(c-a,d-y)*.28)
    x0=max(0,a-pad);y0=max(0,y-pad);x1=min(w,c+pad);y1=min(h,d+pad)
    gray=cv2.cvtColor(im[y0:y1,x0:x1],cv2.COLOR_BGR2GRAY)
    mask=cv2.inRange(gray,0,140);mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    options=[]
    for ct in contours:
        x,z,ww,hh=cv2.boundingRect(ct);bb=[x+x0,z+y0,x+x0+ww,z+y0+hh]
        ix=max(0,min(c,bb[2])-max(a,bb[0]));iy=max(0,min(d,bb[3])-max(y,bb[1]))
        if ix*iy/((c-a)*(d-y))<.65 or cv2.contourArea(ct)<.25*(c-a)*(d-y):continue
        if ww*hh>2*(c-a)*(d-y) or min(ww,hh)<25:continue
        options.append((cv2.contourArea(ct),bb))
    if options:
        b=max(options)[1];return [max(0,b[0]-2),max(0,b[1]-2),min(w,b[2]+2),min(h,b[3]+2)]
    return b

def raw_feat(im,b):
    a,y,c,d=b;g=cv2.cvtColor(im[y:d,a:c],cv2.COLOR_BGR2GRAY)
    g=cv2.resize(g,(48,48)).astype(np.float32)
    yy,xx=np.mgrid[:48,:48];mask=(yy>5)&(yy<42)&(xx>yy*.43+2)&(xx<46-yy*.43)
    g=g[mask];g-=g.mean();return g/(np.linalg.norm(g)+1e-8)

def main():
    rs=json.loads((OUT/'classified.json').read_text(encoding='utf8'))
    banks={g:[] for g in HAZ};cache={}
    for r in rs:
        im=read(r['source'])
        for k,o in enumerate(r['candidates']):
            o['box_original']=o['box_xyxy'];o['box_xyxy']=refine(im,o['box_xyxy'])
            b=o['box_xyxy'];a,y,c,d=b;feat=raw_feat(im,b);cache[(r['index'],k)]=feat
            o['small_or_strip']=min(c-a,d-y)<36 or not .48<(c-a)/(d-y)<2.0
            o['edge']=a<5 or y<5 or c>r['width']-5 or d>r['height']-5
            if o['score']>.60 and o['margin']>.30 and not o['edge'] and not o['small_or_strip']:
                if o['glyph'] in ['ignore',HAZ[r['batch']]]:
                    banks[r['batch']].append((feat,o['glyph'],r['index'],k))
    for r in rs:
        bank=banks[r['batch']];features=np.array([t[0] for t in bank]);labels=[t[1] for t in bank]
        for k,o in enumerate(r['candidates']):
            scores=features@cache[(r['index'],k)]
            values={t:float(max(scores[i] for i,n in enumerate(labels) if n==t)) for t in ['ignore',HAZ[r['batch']]]}
            name=max(values,key=values.get);o['review_glyph']=name;o['reference_similarity']=values[name]
            o['reference_margin']=abs(values['ignore']-values[HAZ[r['batch']]])
            o['review_scores']=values
    (OUT/'refined.json').write_text(json.dumps(rs,ensure_ascii=False,indent=2),encoding='utf8')
    uncertain=[(r,k,o) for r in rs for k,o in enumerate(r['candidates']) if not o['edge'] and not o['small_or_strip'] and (o['reference_similarity']<.68 or o['reference_margin']<.12)]
    for page,start in enumerate(range(0,len(uncertain),100),1):
        sheet=np.full((10*130,10*150,3),245,np.uint8)
        for j,(r,k,o) in enumerate(uncertain[start:start+100]):
            im=read(r['source']);a,y,c,d=o['box_xyxy'];crop=cv2.resize(im[y:d,a:c],(100,100));xx=j%10*150;yy=j//10*130
            sheet[yy:yy+100,xx:xx+100]=crop
            cv2.putText(sheet,f"{r['index']}:{k} {o['review_glyph'][:3]}",(xx,yy+112),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,0,0),1)
            cv2.putText(sheet,f"{o['reference_similarity']:.2f}/{o['reference_margin']:.2f}",(xx,yy+127),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
        cv2.imencode('.jpg',sheet,[cv2.IMWRITE_JPEG_QUALITY,94])[1].tofile(str(OUT/f'uncertain_{page:02d}.jpg'))
    print('reference crops',sum(len(b) for b in banks.values()),'uncertain',len(uncertain),'sheets',(len(uncertain)+99)//100)

if __name__=='__main__':main()
