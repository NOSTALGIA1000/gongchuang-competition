"""Apply explicit visual review decisions to a versioned, audited YOLO dataset."""
from pathlib import Path
import json,hashlib,shutil
import cv2,numpy as np
from refresh_annotation_previews import render

BASE=Path(__file__).resolve().parent
SOURCE=BASE/'datasets/camera_roll_20261114_batch01_reviewed'
OUT=BASE/'datasets/camera_roll_20261114_final_v1'
PROMOTED={68,69}
EXCLUDED={10,11,15,24,25,39,40,41,45,55,59,74,75,76,80,81,92,93,98,113,114,116,130,131,136,137,141,142,154,160,161}
NO_RELIABLE_GLYPH={11,41,75,114,137,161}

def load(path):
    return cv2.imdecode(np.frombuffer(Path(path).read_bytes(),np.uint8),1)

def main():
    rows=json.loads((SOURCE/'manifest.json').read_text(encoding='utf8'))
    assert len(rows)==161
    for sub in ['images','labels','excluded/images','qa']:(OUT/sub).mkdir(parents=True,exist_ok=True)
    final=[];counts={0:0,1:0,2:0}
    for row in rows:
        i=row['index'];accepted=i not in EXCLUDED
        assert accepted==(row['quality']=='ready' or i in PROMOTED)
        src=Path(row['source']);a=load(src);h,w=a.shape[:2]
        assert (w,h)==(1280,720)
        result={'index':i,'stem':row['stem'],'source':str(src),'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),
                'decision':'accepted' if accepted else 'excluded','review_method':'Assistant visual review of full-scene contact sheets and each target crop',
                'capture_group':'same_floor_two_signs_20261114','objects':[]}
        if not accepted:
            result['reason']='No reliably identifiable glyph remains in frame' if i in NO_RELIABLE_GLYPH else 'Truncated target at image boundary; excluded rather than guessing full extent'
            dest=OUT/'excluded/images'/(row['stem']+'.jpg');shutil.copy2(src,dest)
        else:
            lines=[]
            for o in row['objects']:
                cls=o['class_id'];assert cls in [1,2]
                x0,y0,x1,y1=map(int,o['box_xyxy'])
                padding=max(3,round(max(x1-x0,y1-y0)*.04))
                box=[max(0,x0-padding),max(0,y0-padding),min(w,x1+padding),min(h,y1+padding)]
                x0,y0,x1,y1=box
                result['objects'].append({'class_id':cls,'name':{1:'Interference',2:'Hazardous'}[cls],
                     'box_xyxy':box,'initial_box_xyxy':o['box_xyxy'],
                     'box_review':'Expanded a small margin to include printed triangle rim; excludes paper/red task board'})
                counts[cls]+=1
                lines.append(f'{cls} {(x0+x1)/2/w:.8f} {(y0+y1)/2/h:.8f} {(x1-x0)/w:.8f} {(y1-y0)/h:.8f}')
            assert lines
            dest=OUT/'images'/(row['stem']+'.jpg');shutil.copy2(src,dest)
            (OUT/'labels'/(row['stem']+'.txt')).write_text('\n'.join(lines)+'\n',encoding='utf8')
        assert hashlib.sha256(dest.read_bytes()).hexdigest()==result['source_sha256']
        final.append(result)
    classes='Person\nInterference\nHazardous\n'
    (OUT/'classes.txt').write_text(classes,encoding='utf8');(OUT/'labels/classes.txt').write_text(classes,encoding='utf8')
    (OUT/'manifest.json').write_text(json.dumps(final,ensure_ascii=False,indent=2),encoding='utf8')
    summary={'inspected_images':161,'accepted_images':130,'excluded_images':31,'pending_review':0,
       'class_instances':{'Person':counts[0],'Interference':counts[1],'Hazardous':counts[2]},
       'independent_validation_available':False,'training_started':False,
       'limitation':'Visual review is not an independent accuracy benchmark; target set is only two pictograms in one scene'}
    assert sum(x['decision']=='accepted' for x in final)==130
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
    (OUT/'train_candidates.txt').write_text('\n'.join((OUT/'images'/(r['stem']+'.jpg')).as_posix() for r in final if r['decision']=='accepted')+'\n',encoding='utf8')
    # No fabricated val/test split from adjacent frames in this single capture session.
    (OUT/'dataset_collection.yaml').write_text('path: '+OUT.as_posix()+'\ntrain: images\n# Add an independent validation batch before training.\nnames:\n  0: Person\n  1: Interference\n  2: Hazardous\n',encoding='utf8')
    for start in range(0,len(final),24):
        sheet=np.full((4*240,6*300,3),245,np.uint8)
        for j,r in enumerate(final[start:start+24]):
            a=load(r['source'])
            labels=(OUT/'labels'/(r['stem']+'.txt')).read_text() if r['decision']=='accepted' else ''
            a=render(a,labels)
            x=j%6*300;y=j//6*240;sheet[y:y+163,x:x+290]=cv2.resize(a,(290,163))
            color=(0,110,0) if r['decision']=='accepted' else (0,0,180)
            cv2.putText(sheet,f"{r['index']:04d} {r['decision']} n={len(r['objects'])}",(x+3,y+195),cv2.FONT_HERSHEY_SIMPLEX,.43,color,1)
        cv2.imencode('.jpg',sheet,[cv2.IMWRITE_JPEG_QUALITY,94])[1].tofile(str(OUT/'qa'/f'final_{start//24+1:02d}.jpg'))
    a=render(load(OUT/'images/capture_0002.jpg'),(OUT/'labels/capture_0002.txt').read_text(),True)
    cv2.imencode('.png',a)[1].tofile(str(OUT/'qa/example_final_0002.png'))
    notes='''# 本批已完成复核：使用此版本

161张全部完成逐场景、逐目标图案目视检查，保留130张，排除31张，待复核为0。排除图仍保留在excluded/images，并在manifest.json写明原因，不要求用户再处理这31张。
原始相机胶卷不修改；复制图像SHA256与原图一致。

类别：0 Person、1 Interference、2 Hazardous。感叹号=Interference，有毒骷髅=Hazardous；按图案而不是上下位置判断。腿脚和鞋子不是本赛题人员救助区的人头图像，不标Person。
颜色：Person蓝、Interference橙、Hazardous红。YOLO标签本身不保存颜色。

images/和labels/是一一对应的130张正式候选与YOLO标签；每行class x_center y_center width height，坐标归一化。标注框包括三角形外缘，不包括整张白纸、红色任务区外框。
复核中修正过5张原有错类（仍因残缺排除），额外接受原复核区68、69号两张完整可辨认照片。框在原初标基础上小幅放宽，涵盖三角形印刷边缘。

质量边界：这是目视复核完成的当前批次，不是经独立标注员对照计算出的精度百分比。单一场景、两张固定标识，不能据此宣称模型泛化性能。
本批没有Person，也没有独立验证/测试场景。不要随机将相邻连拍分成训练/验证。dataset_collection.yaml为采集期配置，故意不编造val目录。尚未训练。

旧的batch01和batch01_reviewed目录属于历史初稿，以本目录final_v1为准。
'''
    (OUT/'README_复核完成.md').write_text(notes,encoding='utf8')
    registry_path=BASE/'datasets/dataset_registry.json'
    registry=json.loads(registry_path.read_text(encoding='utf8'))
    registry['batches'][0].update({'directory':str(OUT),'ready_images':130,'review_images':0,'excluded_images':31,'class_instances':summary['class_instances'],'review_complete':True})
    registry_path.write_text(json.dumps(registry,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(summary,ensure_ascii=False))

if __name__=='__main__':main()
