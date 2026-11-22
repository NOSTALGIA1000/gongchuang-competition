"""Final filesystem/YOLO audit and resumable project status update."""
from pathlib import Path
from collections import Counter
import json,hashlib,html,cv2,numpy as np
from annotate_camera_roll import read
from finalize_new_targets import draw,save

BASE=Path(__file__).resolve().parent
OUT=BASE/'datasets/targets_2class_20261115_v1'

def dump(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')

def main():
    rs=json.loads((OUT/'manifest.json').read_text(encoding='utf8'))
    accepted=[r for r in rs if r['decision']=='accepted'];rejected=[r for r in rs if r['decision']=='excluded']
    stems={r['stem'] for r in accepted}
    assert {p.stem for p in (OUT/'images').glob('*.jpg')}==stems
    assert {p.stem for p in (OUT/'labels').glob('*.txt')}==stems
    counts=Counter();empty=0
    for r in rs:
        src=Path(r['source']);assert hashlib.sha256(src.read_bytes()).hexdigest()==r['sha256']
        path=OUT/('images' if r['decision']=='accepted' else 'excluded')/(r['stem']+'.jpg')
        assert hashlib.sha256(path.read_bytes()).hexdigest()==r['sha256']
        if r['decision']!='accepted':continue
        im=read(path);h,w=im.shape[:2];assert (w,h)==(r['width'],r['height'])
        lines=(OUT/'labels'/(r['stem']+'.txt')).read_text(encoding='utf8').splitlines()
        assert len(lines)==len(r['objects']);empty+=not lines
        for line,o in zip(lines,r['objects']):
            parts=line.split();assert len(parts)==5
            cls=int(parts[0]);cx,cy,bw,bh=map(float,parts[1:]);assert cls in [0,1] and cls==o['class_id']
            assert 0<bw<=1 and 0<bh<=1 and cx-bw/2>=-1e-7 and cx+bw/2<=1+1e-7 and cy-bh/2>=-1e-7 and cy+bh/2<=1+1e-7
            counts[cls]+=1
    assert len(accepted)==1242 and len(rejected)==554 and dict(counts)=={1:694,0:246}
    assert empty==302
    dump(OUT/'class_mapping.json',{'names':{'0':'Person','1':'Hazardous'},'old_dataset_to_new':{'0':0,'1':None,'2':1},
                                  'new_model_to_current_flight_task':{'0':0,'1':2},'interference_inference':False,
                                  'deployment_status':'未部署；两类模型需要重导出引擎并适配旧三类接口'})
    for p in (OUT/'excluded').glob('*.txt'):
        dest=OUT/'audit'/('discarded_draft_'+p.name)
        assert OUT.resolve() in p.resolve().parents and OUT.resolve() in dest.resolve().parents
        p.replace(dest)
    # A compact, actual image-label preview: hazardous examples, negative, Person.
    examples=[]
    for group in [1,2,4,6,8,10,12,13]:
        examples.append(next(r for r in accepted if r.get('batch')==group and r['objects']))
    examples.append(next(r for r in accepted if r.get('batch')==1 and not r['objects']))
    examples.extend([r for r in accepted if r['origin']=='person_20261114_batch01'][::100][:3])
    sheet=np.full((4*220,3*400,3),245,np.uint8)
    for j,r in enumerate(examples):
        im=draw(read(r['source']),r['objects']);xx=j%3*400;yy=j//3*220
        sheet[yy:yy+202,xx:xx+360]=cv2.resize(im,(360,202))
        cv2.putText(sheet,r['stem'],(xx,yy+216),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,0,0),1)
        save(OUT/'qa'/(r['stem']+'_example.jpg'),im)
    save(OUT/'qa/annotation_examples.jpg',sheet)
    # Review index uses local assets only; no network and no script required.
    s=json.loads((OUT/'summary.json').read_text(encoding='utf8'))
    body=['<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>两类YOLO数据复核</title>',
          '<style>body{font:16px system-ui;max-width:1250px;margin:30px auto;background:#f5f6f8;color:#18202c}table{border-collapse:collapse;width:100%;background:white}td,th{border:1px solid #ddd;padding:8px}img{max-width:100%}details{margin:12px 0}a{color:#075eb0}</style>',
          '<h1>两类YOLO数据：已复核，未训练</h1>',
          '<p>0 Person（蓝框）；1 Hazardous（红框，仅易燃、易爆、有毒）。干扰项不画框，纯干扰图片配空txt作为负样本。</p>',
          f'<p>新13组1420张：保留{s["new_accepted"]}，排除{s["new_excluded"]}；合入旧376张后共{s["total_accepted"]}张。原片和旧发布目录保留。</p>',
          '<p>坐标/编号/图像标签配对与SHA256已校验。尚未划分独立验证集；Person仍需新场景验证。Jetson旧模型未改。</p>',
          '<p><a href="README.md">说明</a> · <a href="逐组统计.csv">逐组统计</a> · <a href="排除清单.csv">排除原因</a></p>',
          '<img src="qa/annotation_examples.jpg" alt="实际标签预览"><h2>逐组统计</h2><table><tr><th>组</th><th>原始</th><th>危险品正样本</th><th>背景负样本</th><th>排除</th></tr>']
    for g in s['new_by_group']:body.append('<tr>'+''.join(f'<td>{g[k]}</td>' for k in ['batch','total','positive','background','excluded'])+'</tr>')
    body.append('</table><h2>逐组图像和标签</h2>')
    for group in range(1,14):
        body.append(f'<details><summary>第 {group} 组</summary><p>')
        for r in accepted:
            if r.get('batch')==group:
                stem=r['stem'];kind='危险品' if r['objects'] else '背景'
                body.append(f'<a href="images/{stem}.jpg">{stem} {kind}</a> / <a href="labels/{stem}.txt">标签</a><br>')
        body.append('</p></details>')
    body.append('<h2>全量最终复核页</h2>')
    for p in sorted((OUT/'qa').glob('targets_*.jpg'))+sorted((OUT/'qa').glob('backgrounds_*.jpg')):
        body.append(f'<details><summary>{p.stem}</summary><img loading="lazy" src="qa/{p.name}"></details>')
    (OUT/'查看标注.html').write_text('\n'.join(body),encoding='utf8')
    audit={'status':'passed','image_label_pairs':len(accepted),'excluded_images':len(rejected),'objects_by_class':dict(counts),
           'empty_background_labels':empty,'source_and_copy_sha256':'all matched','normalized_coordinates':'all valid',
           'new_full_scene_sheets_inspected':30,'initial_candidate_crop_sheets_inspected':12,
           'uncertain_crop_overrides_inspected':58,'final_target_crops_inspected':583,
           'final_background_frames_inspected_before_conservative_edge_exclusion':290,
           'final_QA_change':'7 additional edge-candidate frames excluded, including 5 backgrounds; accepted set is strict subset of inspected draft',
           'trained':False,'independent_validation':False}
    dump(OUT/'audit/verification.json',audit)
    # Refresh inventory from actual disk; retain frozen processing list and capture history.
    invpath=BASE/'last_photo_inventory.json';inv=json.loads(invpath.read_text(encoding='utf8'))
    backup=BASE/'annotation_review/new_13groups/inventory_before_annotation.json'
    if not backup.exists():dump(backup,inv)
    known=set(r['source'].casefold() for r in rs if r['origin']=='new_13groups')
    for name in ['camera_roll_20261114_final_v1','person_20261114_batch01']:
        old=json.loads((BASE/'datasets'/name/'manifest.json').read_text(encoding='utf8'))
        known.update(r['source'].casefold() for r in old)
    current=sorted((p for p in Path(r'C:\Users\ASUS\Pictures\Camera Roll').iterdir() if p.is_file() and p.suffix.lower() in ['.jpg','.jpeg','.png']),key=lambda p:(p.stat().st_mtime,p.name))
    pending=[str(p) for p in current if str(p).casefold() not in known]
    inv['processed_new_files']=[r['source'] for r in rs if r['origin']=='new_13groups']
    inv['unprocessed_files']=pending
    inv['summary'].update(total=len(current),previously_processed=sum(str(p).casefold() in known for p in current),
                          unprocessed=len(pending),unprocessed_first=Path(pending[0]).name if pending else None,
                          unprocessed_last=Path(pending[-1]).name if pending else None,new_since_previous_count=max(0,len(current)-1827),
                          reviewed_new=1420,new_accepted=866,new_excluded=554,accepted_dataset_images=1242,
                          dataset=str(OUT),annotation_status_date='2026-11-15',new_data_trained=False)
    for group,g in zip(inv['batch_history'],s['new_by_group']):group.update(annotation_status='reviewed',accepted=g['accepted'],excluded=g['excluded'],positive=g['positive'],background=g['background'])
    dump(invpath,inv)
    print(json.dumps({'verification':audit,'current_camera_roll':len(current),'pending':len(pending)},ensure_ascii=False))

if __name__=='__main__':main()
