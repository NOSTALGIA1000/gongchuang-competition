"""Verify the visually reviewed Person batch and record collection limitations."""
from pathlib import Path
import json
import hashlib
from PIL import Image

ROOT=Path(__file__).resolve().parent/'datasets/person_20261114_batch01'
records=json.loads((ROOT/'manifest.json').read_text(encoding='utf-8'))
assert len(records)==246
images={p.stem:p for p in (ROOT/'images').glob('*.jpg')}
labels={p.stem:p for p in (ROOT/'labels').glob('*.txt')}
assert images.keys()==labels.keys() and len(images)==246
for r in records:
    p=images[r['stem']]
    assert hashlib.sha256(p.read_bytes()).hexdigest()==r['sha256']
    with Image.open(p) as im:w,h=im.size
    lines=labels[r['stem']].read_text().strip().splitlines()
    assert len(lines)==1
    cls,x,y,bw,bh=map(float,lines[0].split())
    assert cls==0 and 0<bw<=1 and 0<bh<=1
    assert x-bw/2>=0 and x+bw/2<=1 and y-bh/2>=0 and y+bh/2<=1
    x1,y1,x2,y2=r['box_xyxy']
    restored=[(x-bw/2)*w,(y-bh/2)*h,(x+bw/2)*w,(y+bh/2)*h]
    assert max(abs(a-b) for a,b in zip(restored,[x1,y1,x2,y2]))<.01
    r.update({'decision':'accepted','class_id':0,'name':'Person',
              'review_method':'Full-scene contact sheets and every annotated target crop visually inspected; floor false proposals corrected; frame 183 lower edge expanded',
              'box_scope':'Printed head picture including its thin black rectangle; excludes white paper and cyan task-area frame'})
(ROOT/'manifest.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
summary={'images_inspected':246,'accepted_images':246,'excluded_images':0,'pending_review':0,
         'class_instances':{'Person':246,'Interference':0,'Hazardous':0},
         'image_label_pairing':'PASS','yolo_coordinates':'PASS','copied_image_sha256':'PASS',
         'independent_validation_available':False,'training_started':False,
         'limitation':'One printed head, same floor, continuous capture; images are highly correlated, not independent validation.'}
(ROOT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
(ROOT/'dataset_collection.yaml').write_text('path: '+ROOT.as_posix()+'\ntrain: images\n# Add independent validation scenes before formal training.\nnames:\n  0: Person\n  1: Interference\n  2: Hazardous\n',encoding='utf-8')
(ROOT/'README_标注完成.md').write_text('''# Person批次：246张，246个人头目标

已检查全部场景缩略图与逐目标蓝框放大图，修正自动候选误框、漏边；246张均保留。
类别固定：0 Person、1 Interference、2 Hazardous。本批标签全部为0。
Person预览框为蓝色；YOLO文本标签不保存颜色。框住人头图片的完整黑色小边框，不框整张白纸或浅蓝任务区外框。
原图未旋转、未增强、未改写；复制SHA256一致。每张同名YOLO标签一行，坐标归一化，配对及边界验证通过。
画面中的真人腿脚、鞋子不属于赛事印刷人头目标，不标Person。
本批是2026-11-14 14:04—14:06同一地面场景的一段连续拍摄；近似帧较多，保留全量供后续筛选。
人头在原始拍摄中朝下并带有轻微实际角度变化；保留真实朝向，没有人工随机旋转。
此批尚未训练，也未改变旧试训模型。当前独立整理，不冒充独立验证集，不随机将相邻帧分入训练和验证。
结合旧批次已标注总量：376张、459个目标，其中Person246、Interference100、Hazardous113。
正式训练仍缺易燃、易爆及其他干扰图样、独立验证/测试场景。目视复核不等于经过独立标注员测得的精度百分比。
''',encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False))

if __name__=='__main__':pass
