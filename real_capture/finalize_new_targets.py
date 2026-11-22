"""Assistant visually reviewed two-class export. Originals and old releases stay intact."""
from pathlib import Path
from collections import Counter
import json,hashlib,shutil,csv,cv2,numpy as np
from annotate_camera_roll import read,intersection_over_union

BASE=Path(__file__).resolve().parent
REVIEW=BASE/'annotation_review/new_13groups'
OUT=BASE/'datasets/targets_2class_20261115_v1'
HAZ={1:'toxic',2:'fire',3:'fire',4:'explosion',5:'explosion',6:'explosion',7:'explosion',8:'fire',9:'fire',10:'toxic',11:'toxic',12:'toxic',13:'toxic'}
# All 58 uncertain interior crops inspected individually on uncertain_01.jpg.
EXP='411:1 426:2 427:1 492:0 493:0 494:0 495:0 506:0 507:0 508:0 509:0 510:0 524:0 534:0 625:0 626:1 628:1 631:0 632:0 636:0 649:1 661:0 664:1 665:1 684:2 685:1 686:1 687:1 688:0 689:0 690:0 691:0 692:0 693:0 694:0 721:1 722:1 723:1 735:1 736:1 737:1 738:1 739:1 740:0 741:0'
IGNORE='594:0 596:0 681:0 683:0 684:0 684:1 914:2 915:1 1041:0'
TOX='1037:1 1073:1 1234:0 1333:1 1414:1'
OVERRIDES={**{s:'explosion' for s in EXP.split()},**{s:'ignore' for s in IGNORE.split()},**{s:'toxic' for s in TOX.split()}}

def yellow_edge(im):
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);mask=cv2.inRange(hsv,np.array([12,65,45]),np.array([42,255,255]))
    n,_,stats,_=cv2.connectedComponentsWithStats(mask)
    h,w=mask.shape
    # Cropped glyph fragments are excluded even when too small for box proposals.
    for x,y,ww,hh,area in stats[1:]:
        if area>=35 and max(ww,hh)>=15 and (x<=2 or y<=2 or x+ww>=w-2 or y+hh>=h-2):return True
    return False

def write_json(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')

def draw(im,objects):
    im=im.copy()
    for o in objects:
        a,b,c,d=o['box_xyxy'];color=(255,0,0) if o['class_id']==0 else (0,0,255)
        cv2.rectangle(im,(a,b),(c,d),color,2)
        cv2.putText(im,o['name'],(a,max(18,b-6)),cv2.FONT_HERSHEY_SIMPLEX,.6,color,2)
    return im

def save(path,im):cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,94])[1].tofile(str(path))

def main():
    for folder in ['images','labels','excluded','qa','audit']:(OUT/folder).mkdir(parents=True,exist_ok=True)
    rs=json.loads((REVIEW/'refined.json').read_text(encoding='utf8'))
    manifest=[];seen={}
    for r in rs:
        im=read(r['source']);h,w=im.shape[:2];issues=[];objects=[];reviewed=[]
        if r['index']==1:issues.append('拍摄准备画面：衣物遮挡，无有效任务标识')
        if r['index']==2:issues.append('首帧运动模糊明显，危险品图样细节不清')
        if yellow_edge(im):issues.append('画面边缘有截断的黄色标识：保守排除整帧')
        valid=[]
        for k,o in enumerate(r['candidates']):
            glyph=OVERRIDES.get(f"{r['index']}:{k}",o['review_glyph'])
            candidate={**o,'review_glyph':glyph,'manual_override':f"{r['index']}:{k}" in OVERRIDES}
            reviewed.append(candidate)
            if o['edge']:
                # Tiny floor paint/skin proposals are background, not sign boxes.
                a,b,c,d=o['box_xyxy']
                if min(c-a,d-b)>=25:issues.append('标识候选贴边或截断：不把可见残缺危险品当负样本')
                continue
            if o['small_or_strip']:continue
            # Every valid-size candidate is inspected in the original full scene and crop sheets.
            valid.append(candidate)
        if not valid:issues.append('无可确认的完整任务标识，空拍或仅有残片')
        for o in sorted(valid,key=lambda o:o['reference_similarity'],reverse=True):
            if o['review_glyph']=='ignore':continue
            b=o['box_xyxy']
            if any(intersection_over_union(b,x['box_xyxy'])>.4 for x in objects):continue
            objects.append({'class_id':1,'name':'Hazardous','glyph':HAZ[r['batch']],'box_xyxy':b,
                            'box_scope':'完整黄色三角标识及黑边；不含白纸、红色任务板',
                            'review_method':'助手逐张全景与目标裁剪视觉复核；疑难图样逐项修正；参考相似度非模型置信度'})
        digest=hashlib.sha256(Path(r['source']).read_bytes()).hexdigest()
        if digest in seen:issues.append(f'字节完全重复，保留{seen[digest]}')
        seen.setdefault(digest,r['index'])
        stem=f"new_g{r['batch']:02d}_{r['index']:04d}"
        decision='excluded' if issues else 'accepted'
        row=dict(index=r['index'],stem=stem,batch=r['batch'],capture_group=f"new_capture_group_{r['batch']:02d}",
                 source=r['source'],sha256=digest,width=w,height=h,decision=decision,reasons=list(dict.fromkeys(issues)),
                 objects=objects if decision=='accepted' else [],review_candidates=reviewed,
                 review_status='逐帧全景、全部候选裁剪及疑难图样已复核',origin='new_13groups')
        manifest.append(row)
    # Pre-existing accepted releases: preserve source annotations, only transform taxonomy.
    for name,prefix in [('camera_roll_20261114_final_v1','old_hazard'),('person_20261114_batch01','old_person')]:
        srcdir=BASE/'datasets'/name
        old=json.loads((srcdir/'manifest.json').read_text(encoding='utf8'))
        for r in old:
            if r['decision']!='accepted':continue
            src=srcdir/'images'/(r['stem']+'.jpg');im=read(src);h,w=im.shape[:2];objects=[]
            label=srcdir/'labels'/(r['stem']+'.txt')
            for line in label.read_text(encoding='utf8').splitlines():
                if not line.strip():continue
                cls,cx,cy,bw,bh=map(float,line.split());cls=int(cls)
                if cls==1:continue
                assert cls in [0,2]
                bbox=[round((cx-bw/2)*w),round((cy-bh/2)*h),round((cx+bw/2)*w),round((cy+bh/2)*h)]
                objects.append(dict(class_id=0 if cls==0 else 1,name='Person' if cls==0 else 'Hazardous',
                                    glyph='person' if cls==0 else 'toxic',box_xyxy=bbox,
                                    review_method='沿用此前逐张复核标签，仅删除Interference、映射Hazardous 2→1'))
            manifest.append(dict(stem=f"{prefix}_{r['stem']}",source=str(src),original_source=r['source'],
                                 sha256=hashlib.sha256(src.read_bytes()).hexdigest(),width=w,height=h,
                                 decision='accepted',reasons=[],objects=objects,capture_group=r['capture_group'],origin=name))
    # Reconcile a previous draft without deleting originals or any training images.
    # Any image newly rejected by final QA moves inside this release's excluded folder.
    root=OUT.resolve()
    for r in manifest:
        if r['decision']!='excluded':continue
        for folder,ext in [('images','.jpg'),('labels','.txt')]:
            old=OUT/folder/(r['stem']+ext)
            if old.exists():
                assert root in old.resolve().parents
                target=OUT/'excluded'/(r['stem']+ext)
                assert root in target.resolve().parents
                if target.exists():
                    assert target.read_bytes()==old.read_bytes()
                    old.unlink()
                else:old.replace(target)
    # Write originals as copies; rejected frames never enter training images/labels.
    for r in manifest:
        src=Path(r['source']);stem=r['stem']
        if r['decision']=='excluded':shutil.copy2(src,OUT/'excluded'/(stem+'.jpg'));continue
        shutil.copy2(src,OUT/'images'/(stem+'.jpg'))
        lines=[];w=r['width'];h=r['height']
        for o in r['objects']:
            a,b,c,d=o['box_xyxy'];assert 0<=a<c<=w and 0<=b<d<=h
            lines.append(f"{o['class_id']} {(a+c)/2/w:.8f} {(b+d)/2/h:.8f} {(c-a)/w:.8f} {(d-b)/h:.8f}")
        (OUT/'labels'/(stem+'.txt')).write_text('\n'.join(lines)+('\n' if lines else ''),encoding='utf8')
    write_json(OUT/'manifest.json',manifest)
    write_json(OUT/'audit/manual_glyph_overrides.json',OVERRIDES)
    new=[r for r in manifest if r['origin']=='new_13groups'];accepted=[r for r in manifest if r['decision']=='accepted']
    summary={'new_total':len(new),'new_accepted':sum(r['decision']=='accepted' for r in new),'new_excluded':sum(r['decision']=='excluded' for r in new),
             'new_positive':sum(r['decision']=='accepted' and bool(r['objects']) for r in new),
             'new_background':sum(r['decision']=='accepted' and not r['objects'] for r in new),
             'total_accepted':len(accepted),'total_objects':dict(Counter(o['name'] for r in accepted for o in r['objects'])),
             'hazard_glyphs':dict(Counter(o['glyph'] for r in accepted for o in r['objects'] if o['class_id']==1)),
             'new_by_group':[dict(batch=g,total=sum(r['batch']==g for r in new),accepted=sum(r['batch']==g and r['decision']=='accepted' for r in new),
                                  positive=sum(r['batch']==g and r['decision']=='accepted' and bool(r['objects']) for r in new),
                                  background=sum(r['batch']==g and r['decision']=='accepted' and not r['objects'] for r in new),
                                  excluded=sum(r['batch']==g and r['decision']=='excluded' for r in new)) for g in range(1,14)],
             'trained':False,'split':'未划分训练/验证；须按采集组划分，Person仍缺独立验证场景'}
    write_json(OUT/'summary.json',summary)
    with (OUT/'逐组统计.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(summary['new_by_group'][0]));writer.writeheader();writer.writerows(summary['new_by_group'])
    with (OUT/'排除清单.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.writer(f);writer.writerow(['group','index','source','excluded_copy','reason'])
        for r in new:
            if r['decision']=='excluded':writer.writerow([r['batch'],r['index'],r['source'],r['stem']+'.jpg','；'.join(r['reasons'])])
    (OUT/'classes.txt').write_text('Person\nHazardous\n',encoding='utf8')
    (OUT/'collection.yaml').write_text('path: '+OUT.as_posix()+'\nimages: images\nlabels: labels\nnc: 2\nnames:\n  0: Person\n  1: Hazardous\n# 标注集合清单，尚非训练/验证划分配置。\n',encoding='utf8')
    (OUT/'README.md').write_text('''# 两类实拍 YOLO 标注集

- 0 Person：赛事救助人物印刷图，复核预览用蓝框。
- 1 Hazardous：仅易燃、易爆、有毒标识，统一危险品类，复核预览用红框。
- 人行横道、感叹号、触电、跌倒等均是背景，无框；只含干扰项的清晰图片配空txt，作为负样本。
- 原始照片保留；排除图片另存excluded，原因见排除清单.csv。
- images/labels一一对应，YOLO格式为 class cx cy width height，坐标归一化。
- 新13组逐帧全景、全部候选裁剪和疑难图样已看图复核。轻微模糊仍可辨认者保留；截断标识帧保守排除。
- 合入此前已复核376张：仅删除旧Interference标注、旧Hazardous编号2映射为1。旧发布目录未改。
- collection.yaml是集合清单；尚未划分训练/验证，未训练。后续按采集组划分；Person缺独立验证场景，不能随机拆相邻连拍并宣称独立精度。
- 当前Jetson旧模型与程序未改。新模型两类输出与旧三类接口不兼容，部署前须将新Hazardous=1映射到飞行任务原Hazardous=2，且取消Interference输出；不能直接覆盖旧引擎。
- 参考相似度仅用于辅助整理，不代表YOLO置信度或标注准确率。最终标签以复核记录为准。
''',encoding='utf8')
    # Target-only crop QA, and all accepted backgrounds for false-negative checking.
    crops=[(r,o) for r in accepted if r['origin']=='new_13groups' for o in r['objects']]
    for page,start in enumerate(range(0,len(crops),120),1):
        sheet=np.full((10*130,12*140,3),245,np.uint8)
        for j,(r,o) in enumerate(crops[start:start+120]):
            im=read(r['source']);a,b,c,d=o['box_xyxy'];pad=5
            crop=im[max(0,b-pad):min(r['height'],d+pad),max(0,a-pad):min(r['width'],c+pad)]
            xx=j%12*140;yy=j//12*130;sheet[yy:yy+100,xx:xx+100]=cv2.resize(crop,(100,100))
            cv2.putText(sheet,f"{r['index']} {o['glyph'][:3]}",(xx,yy+120),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,200),1)
        save(OUT/'qa'/f'targets_{page:02d}.jpg',sheet)
    negatives=[r for r in accepted if r['origin']=='new_13groups' and not r['objects']]
    for page,start in enumerate(range(0,len(negatives),64),1):
        sheet=np.full((8*150,8*250,3),245,np.uint8)
        for j,r in enumerate(negatives[start:start+64]):
            im=read(r['source']);xx=j%8*250;yy=j//8*150;sheet[yy:yy+125,xx:xx+240]=cv2.resize(im,(240,125))
            cv2.putText(sheet,str(r['index']),(xx,yy+143),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),1)
        save(OUT/'qa'/f'backgrounds_{page:02d}.jpg',sheet)
    print(json.dumps(summary,ensure_ascii=False))

if __name__=='__main__':main()
