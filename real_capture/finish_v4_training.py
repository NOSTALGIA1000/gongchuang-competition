"""Run baseline evaluation, training and final evaluation with local logs."""
from pathlib import Path
import ctypes
import hashlib
import json
import os
import subprocess
import sys
import time
import traceback

BASE = Path(__file__).resolve().parent
WORK = BASE / 'training/targets_v4_person_0p5m_yolov5s'


def state(value):
    path = WORK / 'completion_pipeline.json'
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf8')
    temp.replace(path)


def run_step(name, script, *args):
    state({'status': name, 'pid': os.getpid(), 'updated_at_unix': time.time()})
    with (WORK / (name + '_stdout.log')).open('w', encoding='utf8') as out, \
            (WORK / (name + '_stderr.log')).open('w', encoding='utf8') as err:
        subprocess.run([sys.executable, '-X', 'utf8', '-u', str(BASE / script), *args],
                       cwd=str(BASE.parent), stdout=out, stderr=err, check=True)


def main():
    run_step('baseline_evaluation', 'evaluate_v4_training.py', '--baseline')
    run_step('training', 'run_v4_training.py')
    run_step('evaluation', 'evaluate_v4_training.py')
    report = json.loads((WORK / 'evaluation/evaluation.json').read_text(encoding='utf8'))
    baseline = json.loads((WORK / 'baseline_evaluation/evaluation.json').read_text(encoding='utf8'))
    training = json.loads((WORK / 'run_info.json').read_text(encoding='utf8'))
    assert training['status'] == report['status'] == baseline['status'] == 'completed'
    best = Path(training['weights'])
    assert hashlib.sha256(best.read_bytes()).hexdigest() == report['weights_sha256']
    rows = []
    for name in ['Person', 'Hazardous']:
        old, new = baseline['per_class'][name], report['per_class'][name]
        fixed = report['threshold_reports']['0.5']['per_class'][name]
        rows.append(f"| {name} | {new['targets']} | {old['map50_95']:.4f} | "
                    f"{new['map50']:.4f} | {new['map50_95']:.4f} | "
                    f"{fixed['tp']} / {fixed['fp']} / {fixed['fn']} |")
    doc = f'''# v4：0.5米Person替换与危险品合并训练

新Person采集257张，全部复核；保留144张，排除113张残缺、遮挡或无法可靠定框的照片。原图及排除副本保留。旧Person 246张从本轮数据清单移除。

合并1140张：Person 144、危险品694、背景302。训练924张（Person107、危险品581、背景236）；验证205张（Person26、危险品113、背景66）；另11张Person时间边界隔离，不参与训练和验证。

从已完成的v3两类YOLOv5s best.pt初始化，重新建立优化器。旧Person照片不参与本轮，但初始化权重仍包含先前学到的特征。原生YOLOv5源码commit {training['repo_commit']}。

batch16，输入640，SGD lr0=0.001，cosine，上限100轮，patience20；实际{training['completed_epoch']}轮，最佳第{report['best_checkpoint_epoch']}轮，耗时{training['elapsed_seconds'] / 60:.1f}分钟。

数据增强：hsv_h=0.005、hsv_s=0.2、hsv_v=0.3、translate=0.1、scale=0.3、mosaic=0.2。随机旋转degrees=0，翻转/透视/错切/MixUp/CopyPaste均0。Mosaic可能产生训练用裁剪；人工残缺原片没有被错误作为空标签背景。

## 同一留出集：旧模型与新模型

| 类别 | 验证目标数 | v3 mAP50-95 | v4 mAP50 | v4 mAP50-95 | v4 TP / FP / FN |
|---|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

固定统计conf=0.5、IoU匹配阈值0.5。66张背景中{report['threshold_reports']['0.5']['background_images_with_false_detection']}张出现误检。完整阈值0.25/0.5统计见evaluation/evaluation.json；实际预测预览见evaluation/prediction_examples.jpg。

新Person只有同一场地连续采集；按时间块留出并隔离10秒，减少相邻帧泄漏，但不能代表独立实飞场景精度。验证结果不能证明已排除相机视野、目标尺寸或任务筛选等实飞问题。

模型：run/weights/best.pt。SHA256：{report['weights_sha256']}。只输出0 Person（预览蓝框）、1 Hazardous（预览红框），Interference不输出。若部署Jetson，需重新导出TensorRT引擎，保持危险品1→飞行任务2的接口映射；本轮没有改动板端模型、引擎或飞行程序。
'''
    (WORK / '训练结果说明.md').write_text(doc, encoding='utf8')
    invpath = BASE / 'last_photo_inventory.json'
    inv = json.loads(invpath.read_text(encoding='utf8'))
    inv['person_0p5m_batch'].update(
        trained=True, training_images=107, validation_images=26, isolation_images=11,
        training_work=str(WORK), best_weights=str(best), weights_sha256=report['weights_sha256'])
    inv['summary'].update(new_data_trained=True, trained_model=str(best),
                          training_status_date=time.strftime('%Y-%m-%d'))
    invpath.write_text(json.dumps(inv, ensure_ascii=False, indent=2), encoding='utf8')
    state({'status': 'completed', 'weights': str(best), 'overall': report['overall'],
           'epochs_completed': training['completed_epoch'],
           'best_checkpoint_epoch': report['best_checkpoint_epoch'],
           'report': str(WORK / '训练结果说明.md'), 'completed_at_unix': time.time()})


if __name__ == '__main__':
    ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)
    try:
        main()
    except BaseException as exc:
        state({'status': 'failed', 'error': repr(exc), 'traceback': traceback.format_exc()})
        raise
    finally:
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
