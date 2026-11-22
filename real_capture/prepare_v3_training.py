"""Freeze group-held-out YOLOv5 split and assert taxonomy/leakage constraints."""
from pathlib import Path
from collections import Counter
import json,hashlib,subprocess
import yaml

BASE=Path(__file__).resolve().parent
SOURCE=BASE/'datasets/targets_2class_20261115_v1'
WORK=BASE/'training/targets_v3_yolov5s'
REPO=BASE.parent/'third_party/yolov5'

def main():
    WORK.mkdir(parents=True,exist_ok=True)
    records=json.loads((SOURCE/'manifest.json').read_text(encoding='utf8'))
    records=[r for r in records if r['decision']=='accepted']
    persons=sorted([r for r in records if r['origin']=='person_20261114_batch01'],key=lambda r:r['stem'])
    assert len(persons)==246
    person_val={r['stem'] for r in persons[-51:]}
    person_gap={r['stem'] for r in persons[-61:-51]}
    held_groups={3,6,11};split={'train':[],'val':[],'gap':[]}
    for r in records:
        which='gap' if r['stem'] in person_gap else 'val' if r['stem'] in person_val or r.get('batch') in held_groups else 'train'
        image=SOURCE/'images'/(r['stem']+'.jpg');label=SOURCE/'labels'/(r['stem']+'.txt')
        assert image.exists() and label.exists()
        assert hashlib.sha256(image.read_bytes()).hexdigest()==r['sha256']
        assert all(int(line.split()[0]) in [0,1] for line in label.read_text(encoding='utf8').splitlines())
        split[which].append({'stem':r['stem'],'image':image.as_posix(),'sha256':r['sha256'],
                             'capture_group':r['capture_group'],'origin':r['origin'],
                             'objects':r['objects'],'label_sha256':hashlib.sha256(label.read_bytes()).hexdigest()})
    assert {k:len(v) for k,v in split.items()}=={'train':1002,'val':230,'gap':10}
    for a,b in [('train','val'),('train','gap'),('val','gap')]:
        assert not ({r['stem'] for r in split[a]}&{r['stem'] for r in split[b]})
        assert not ({r['sha256'] for r in split[a]}&{r['sha256'] for r in split[b]})
    assert Counter(o['name'] for r in split['val'] for o in r['objects'])=={'Person':51,'Hazardous':113}
    assert set(o['glyph'] for r in split['val'] for o in r['objects'] if o['class_id']==1)=={'fire','explosion','toxic'}
    for which,rs in split.items():
        (WORK/f'{which}.txt').write_text('\n'.join(r['image'] for r in rs)+'\n',encoding='utf8')
    config={'path':SOURCE.as_posix(),'train':(WORK/'train.txt').as_posix(),'val':(WORK/'val.txt').as_posix(),
            'nc':2,'names':['Person','Hazardous']}
    (WORK/'data.yaml').write_text(yaml.safe_dump(config,sort_keys=False,allow_unicode=False),encoding='utf8')
    hyp=yaml.safe_load((REPO/'data/hyps/hyp.scratch-low.yaml').read_text(encoding='utf8'))
    hyp.update(lr0=.005,lrf=.01,warmup_epochs=3.0,hsv_h=0.,hsv_s=.2,hsv_v=.3,degrees=0.,translate=.1,
               scale=.3,shear=0.,perspective=0.,flipud=0.,fliplr=0.,mosaic=0.,mixup=0.,copy_paste=0.)
    assert all(hyp[k]==0 for k in ['degrees','shear','perspective','flipud','fliplr','hsv_h','mosaic','mixup','copy_paste'])
    (WORK/'hyp.yaml').write_text(yaml.safe_dump(hyp,sort_keys=False),encoding='utf8')
    summary={k:{'images':len(rs),'objects':dict(Counter(o['name'] for r in rs for o in r['objects'])),
                'backgrounds':sum(not r['objects'] for r in rs)} for k,rs in split.items()}
    data={'source':str(SOURCE),'source_manifest_sha256':hashlib.sha256((SOURCE/'manifest.json').read_bytes()).hexdigest(),
          'repo_commit':subprocess.check_output(['git','-C',str(REPO),'rev-parse','HEAD'],text=True).strip(),
          'pretrained_sha256':hashlib.sha256((WORK/'yolov5s_coco_v7.pt').read_bytes()).hexdigest(),
          'policy':'new groups 3/6/11 held out; Person last51 held out, previous10 purged from training',
          'limitations':'Same venue/floor across all groups. Person validation is a temporal block of one capture group, not an independent scene. No independent test set.',
          'summary':summary,'split':split}
    (WORK/'split_manifest.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
    (WORK/'split_audit.json').write_text(json.dumps({'status':'passed','summary':summary,'stem_and_sha256_overlap':False,
                                                   'prohibited_augmentation':False,'limitations':data['limitations']},ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(summary,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
