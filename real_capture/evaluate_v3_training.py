"""Post-training holdout evaluation with per-class and fixed-threshold error reports."""
from pathlib import Path
from collections import Counter
import json,time,hashlib,math
from run_v3_training import WORK,BASE,REPO,atomic_json

def main():
    import torch,numpy as np,cv2
    import val
    from utils.callbacks import Callbacks
    from utils.general import box_iou,scale_boxes
    torch.set_num_threads(4)
    state=json.loads((WORK/'run_info.json').read_text(encoding='utf8'))
    assert state['status']=='completed',state['status']
    split=json.loads((WORK/'split_manifest.json').read_text(encoding='utf8'))
    refs={r['stem']:r for r in split['split']['val']}
    assert len(refs)==230
    dest=WORK/'evaluation';dest.mkdir(exist_ok=True)
    report={'status':'evaluating','weights':state['weights'],'weights_sha256':hashlib.sha256(Path(state['weights']).read_bytes()).hexdigest(),
            'split':split['summary'],'limitations':split['limitations'],'dataset_classes':{'0':'Person','1':'Hazardous'},
            'threshold_reports':{},'per_class':{},'predictions':[]}
    atomic_json(dest/'evaluation_status.json',{'status':'running','started_at_unix':time.time()})
    callbacks=Callbacks();per_image={}
    def batch_end(batch_i,im,targets,paths,shapes,preds):
        # YOLOv5 skips on_val_image_end when a frame has zero detections.
        # Collect every frame at batch end so clean backgrounds remain counted.
        for si,path in enumerate(paths):
            stem=Path(path).stem
            assert stem in refs,stem
            predn=preds[si].clone()
            if len(predn):scale_boxes(im[si].shape[1:],predn[:,:4],shapes[si][0],shapes[si][1])
            per_image[stem]={'path':str(path),'predictions':predn.detach().float().cpu().numpy().tolist()}
    def val_end(nt,tp,fp,p,r,f1,ap,ap50,ap_class,confusion_matrix):
        for i,c in enumerate(ap_class):
            name=['Person','Hazardous'][int(c)]
            report['per_class'][name]={'targets':int(nt[c]),'precision_at_f1_operating_point':float(p[i]),
                                     'recall_at_f1_operating_point':float(r[i]),'map50':float(ap50[i]),'map50_95':float(ap[i])}
        report['confusion_matrix']=confusion_matrix.matrix.tolist()
    callbacks.register_action('on_val_batch_end',name='collect_all_frames',callback=batch_end)
    callbacks.register_action('on_val_end',name='collect_metrics',callback=val_end)
    results,maps,times=val.run(data=str(WORK/'data.yaml'),weights=state['weights'],batch_size=16,imgsz=640,
                              device='0',workers=0,conf_thres=.001,iou_thres=.6,half=True,
                              project=str(dest),name='holdout',exist_ok=True,plots=True,verbose=True,callbacks=callbacks)
    assert len(per_image)==230,len(per_image)
    assert sum(not refs[s]['objects'] for s in per_image)==66
    report['overall']={k:float(v) for k,v in zip(['precision_at_f1_operating_point','recall_at_f1_operating_point','map50','map50_95'],results[:4])}
    report['gpu_ms_per_image']=list(times)
    error_stems=set()
    for threshold in [.25,.5]:
        counts={name:Counter(tp=0,fp=0,fn=0) for name in ['Person','Hazardous']};glyph=Counter();bg_fp=[];errors=[]
        for stem,ref in refs.items():
            preds=[p for p in per_image[stem]['predictions'] if p[4]>=threshold]
            matched=set();tp=0;fp=0
            for pred in sorted(preds,key=lambda p:p[4],reverse=True):
                cls=int(pred[5]);name=['Person','Hazardous'][cls]
                options=[]
                for j,o in enumerate(ref['objects']):
                    if j in matched or o['class_id']!=cls:continue
                    iou=float(box_iou(torch.tensor([pred[:4]]),torch.tensor([o['box_xyxy']],dtype=torch.float32))[0,0])
                    if iou>=.5:options.append((iou,j))
                if options:
                    j=max(options)[1];matched.add(j);counts[name]['tp']+=1;tp+=1
                    glyph[ref['objects'][j]['glyph']+'_tp']+=1
                else:counts[name]['fp']+=1;fp+=1
            for j,o in enumerate(ref['objects']):
                if j not in matched:counts[o['name']]['fn']+=1;glyph[o['glyph']+'_fn']+=1
            if not ref['objects'] and preds:bg_fp.append(stem)
            if fp or len(matched)<len(ref['objects']):
                errors.append({'stem':stem,'tp':tp,'fp':fp,'fn':len(ref['objects'])-len(matched)})
                error_stems.add(stem)
        summary={}
        for name,c in counts.items():
            summary[name]={**dict(c),'precision':c['tp']/max(1,c['tp']+c['fp']),'recall':c['tp']/max(1,c['tp']+c['fn'])}
        report['threshold_reports'][str(threshold)]={'iou_threshold':.5,'per_class':summary,'glyph_counts':dict(glyph),
                    'background_images':66,'background_images_with_false_detection':len(bg_fp),'false_background_stems':bg_fp,'errors':errors}
    for stem,p in per_image.items():report['predictions'].append({'stem':stem,'predictions':[v for v in p['predictions'] if v[4]>=.05]})
    # Select holdout examples and all error cases (bounded gallery), actual model outputs only.
    chosen=list(sorted(error_stems))[:16]
    for group in ['new_capture_group_03','new_capture_group_06','new_capture_group_11','person_same_floor_20261114_1404_1406']:
        for r in [r for r in refs.values() if r['capture_group']==group][:3]:
            if r['stem'] not in chosen:chosen.append(r['stem'])
    for r in [r for r in refs.values() if not r['objects']][:3]:
        if r['stem'] not in chosen:chosen.append(r['stem'])
    previews=dest/'predictions';previews.mkdir(exist_ok=True)
    sheet=np.full((((len(chosen)+3)//4)*205,4*330,3),245,np.uint8)
    for j,stem in enumerate(chosen):
        r=refs[stem];im=cv2.imdecode(np.frombuffer(Path(r['image']).read_bytes(),np.uint8),1)
        for pred in per_image[stem]['predictions']:
            if pred[4]<.5:continue
            a,b,c,d=map(round,pred[:4]);cls=int(pred[5]);color=(255,0,0) if cls==0 else (0,0,255)
            cv2.rectangle(im,(a,b),(c,d),color,2);cv2.putText(im,f"{['Person','Hazardous'][cls]} {pred[4]:.2f}",(a,max(16,b-6)),cv2.FONT_HERSHEY_SIMPLEX,.6,color,2)
        cv2.imencode('.jpg',im)[1].tofile(str(previews/(stem+'.jpg')))
        xx=j%4*330;yy=j//4*205;sheet[yy:yy+180,xx:xx+320]=cv2.resize(im,(320,180))
        cv2.putText(sheet,stem,(xx,yy+198),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,0,0),1)
    cv2.imencode('.jpg',sheet,[cv2.IMWRITE_JPEG_QUALITY,94])[1].tofile(str(dest/'prediction_examples.jpg'))
    history=[json.loads(line) for line in (WORK/'epoch_history.jsonl').read_text(encoding='utf8').splitlines()]
    report['epochs_completed']=state['completed_epoch']
    report['best_checkpoint_epoch']=max(r['epoch'] for r in history if r['fitness']==r['best_fitness'])
    report['status']='completed';atomic_json(dest/'evaluation.json',report)
    m=report['overall'];rows=[]
    for name,v in report['per_class'].items():rows.append(f"| {name} | {v['targets']} | {v['map50']:.4f} | {v['map50_95']:.4f} |")
    operating=report['threshold_reports']['0.5'];fixed=[]
    for name,v in operating['per_class'].items():fixed.append(f"| {name} | {v['tp']} | {v['fp']} | {v['fn']} | {v['precision']:.4f} | {v['recall']:.4f} |")
    doc=f'''# YOLOv5s 两类合并训练结果

训练完成{state['completed_epoch']}轮，上限200轮，patience40。最佳保存轮次{report['best_checkpoint_epoch']}。

合并已复核1242张：训练1002、验证230、Person时段边界隔离10。训练Person185、危险品581、纯背景236；验证Person51、危险品113、纯背景66。

只输出0 Person、1 Hazardous。危险品仅易燃、易爆、有毒；Interference不设类别。batch={state['batch']}，imgsz640，SGD初始lr0.005，cosine，旋转/翻转/透视/错切全0；光照、饱和度、平移及缩放轻度增强。

从官方原版YOLOv5s COCO预训练初始化；此前YOLOv8模型未续训、未覆盖。模型路径：run/weights/best.pt。

## 留出验证结果

| 类别 | 目标数 | mAP50 | mAP50-95 |
|---|---:|---:|---:|
{chr(10).join(rows)}

两类宏平均mAP50={m['map50']:.4f}，mAP50-95={m['map50_95']:.4f}。

## 固定阈值运行统计

conf=0.5，IoU匹配阈值0.5；TP/FP/FN按检测实例计数。

| 类别 | TP | FP | FN | Precision | Recall |
|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(fixed)}

66张纯背景中，{operating['background_images_with_false_detection']}张出现至少一个误检。conf=0.25的对应统计见evaluation/evaluation.json。

## 使用范围

这是同场地按整组留出的验证。Person仅一个采集组，采用连续时间块和10帧隔离，仍不能代表独立新场景精度。尚无独立测试集，实际飞行识别须补充新场景验证。GPU耗时为本机结果，不是Jetson速度。

Jetson旧模型、引擎、飞行程序均未改。新模型两类输出需重新导出引擎并适配旧飞行接口：Person0保持0，Hazardous1映射原任务Hazardous2，Interference不输出。不要直接覆盖旧引擎。

代码commit：{state['repo_commit']}；绘图保存有一处兼容修正，未改网络、损失或增强。数据划分、输入权重SHA256、环境、逐轮指标和完整日志均位于本目录。
'''
    (WORK/'训练结果说明.md').write_text(doc,encoding='utf8')
    atomic_json(dest/'evaluation_status.json',{'status':'completed','weights':state['weights'],'report':str(WORK/'训练结果说明.md'),'best_checkpoint_epoch':report['best_checkpoint_epoch']})
    print('EVALUATION_COMPLETE',json.dumps({k:report[k] for k in ['overall','per_class','epochs_completed','best_checkpoint_epoch']},ensure_ascii=False),flush=True)

if __name__=='__main__':main()
