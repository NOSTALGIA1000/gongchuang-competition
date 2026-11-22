"""Prepare a portable, documented model handoff; ZIP only explicit deliverables."""
from pathlib import Path
import json
import shutil
import hashlib
import zipfile
import argparse

ROOT=Path('F:/工创比赛')
RUN=ROOT/'real_capture/training/person_combined_v2'
SHARE=ROOT/'队友分享'
PACKAGE=SHARE/'gongchuang_yolo_v2_person'
ZIP=SHARE/'工创视觉_三类模型v2_含Person.zip'

README='''# 工创视觉三类模型 v2（加入 Person）

## 版本与范围
2026-11-14，本机 RTX4060 Laptop 训练完成。YOLOv8n 检测模型，best.pt。
376 张真实拍摄图片、459 个目标：Person246、Interference100、Hazardous113。
本批实际图样只有：印刷人头、感叹号、有毒骷髅。
Hazardous 不代表已学会易燃易爆等所有危险品；Interference 也不代表已覆盖所有干扰项。
Person 指赛事印刷人头图片，不是通用真人行人检测。

## 类别与显示颜色（编号不可改）
0 Person：蓝色；OpenCV BGR=(255,0,0)
1 Interference：橙色；OpenCV BGR=(0,165,255)
2 Hazardous：红色；OpenCV BGR=(0,0,255)
模型和YOLO标签不包含显示颜色，predict.py已实现上述画框规则。
模型输出为标准YOLOv8检测框；示例 results.json 中 xyxy_pixels 为原始图片像素坐标。

## 使用前提
这是 PyTorch .pt 权重，不是 TensorRT .engine，也不是YOLOv5权重。
训练环境：Python3.12、Ultralytics8.4.212、PyTorch2.13.0+cu126、torchvision0.28.0+cu126。
训练环境版本供复现参考，不要求 Jetson 照搬 Windows 的 CUDA/PyTorch 包。

## 普通电脑图片演示
解压后在本文件所在目录打开终端。已有可用PyTorch环境时，安装 requirements-demo.txt。
若没有PyTorch，先按设备对应的官方方式安装；CPU也能运行演示。

    python -m pip install -r requirements-demo.txt
    python predict.py --source examples/input_person.jpg --output predictions --device cpu

有可用 NVIDIA CUDA 环境时可改为 --device 0。
--source 也支持图片文件夹；示例只处理静态图片，不操作飞控、不触发投放。
默认 conf=0.25 是演示阈值，实机须用独立数据评估后确定。

## Jetson 接入
先确认具体 Jetson 型号、JetPack、Python、PyTorch/torchvision 兼容版本。
不要在现有飞控环境盲目升级全部依赖；本包没有声称已通过Jetson测试。
先在Jetson完成 .pt 推理验证，再视需要导出ONNX/TensorRT，并在目标设备构建和测试引擎。
Jetson相机读取、通信、实时回传、连续稳定识别与投放逻辑需另行接入。

## 训练与精度边界
60轮、batch16、imgsz640，从官方YOLOv8n COCO预训练权重训练。
随机旋转 degrees=0，水平/垂直翻转均关闭；其他详细参数见 training/args.yaml。
重要：train与val使用同一批图片。指标只是训练集回测，不能作为独立验证精度或比赛成功率。
examples/pred_*.jpg 同样是训练图片上的模型预测，不是新场景测试结果。
两批均为相似场景连续拍摄，仍需补齐其它图样及独立拍摄验证/测试场景。

## 包内文件
best.pt：待接入的新模型。
classes.txt、class_mapping.json：类别与颜色。
predict.py：静态图片推理与画框示例。
requirements-demo.txt：演示依赖版本。
examples/：Person和危险品/干扰项输入及预测预览。
training/：实际训练参数、曲线、逐轮记录和版本说明。
SHA256SUMS.txt：文件哈希校验清单。
'''

def prepare():
    (PACKAGE/'examples').mkdir(parents=True,exist_ok=True)
    (PACKAGE/'training').mkdir(exist_ok=True)
    for src,dest in [
        (RUN/'run/weights/best.pt',PACKAGE/'best.pt'),
        (ROOT/'real_capture/share_predict.py',PACKAGE/'predict.py'),
        (RUN/'dataset/images/person_0001.jpg',PACKAGE/'examples/input_person.jpg'),
        (RUN/'dataset/images/capture_0002.jpg',PACKAGE/'examples/input_hazard_interference.jpg'),
        (RUN/'predictions/person_0001.jpg',PACKAGE/'examples/pred_person.jpg'),
        (RUN/'predictions/person_0183.jpg',PACKAGE/'examples/pred_person_lower_frame.jpg'),
        (RUN/'predictions/capture_0002.jpg',PACKAGE/'examples/pred_hazard_interference.jpg'),
    ]:
        shutil.copy2(src,dest)
    for name in ['args.yaml','results.csv','results.png']:
        shutil.copy2(RUN/'run'/name,PACKAGE/'training'/name)
    info=json.loads((RUN/'run_info.json').read_text(encoding='utf-8'))
    info['weights']='best.pt'
    (PACKAGE/'training/run_info.json').write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8')
    (PACKAGE/'README_先看我.md').write_text(README,encoding='utf-8-sig')
    (PACKAGE/'requirements-demo.txt').write_text('ultralytics==8.4.212\n',encoding='utf-8')
    (PACKAGE/'classes.txt').write_text('Person\nInterference\nHazardous\n',encoding='utf-8')
    mapping=[{'id':i,'name':name,'color':color,'opencv_bgr':bgr} for i,name,color,bgr in
             [(0,'Person','blue',[255,0,0]),(1,'Interference','orange',[0,165,255]),(2,'Hazardous','red',[0,0,255])]]
    (PACKAGE/'class_mapping.json').write_text(json.dumps(mapping,ensure_ascii=False,indent=2),encoding='utf-8')
    print('Prepared',PACKAGE)

def archive():
    allowed=['best.pt','predict.py','README_先看我.md','requirements-demo.txt','classes.txt','class_mapping.json']
    files=[PACKAGE/n for n in allowed]+sorted((PACKAGE/'examples').glob('*'))+sorted((PACKAGE/'training').glob('*'))
    checksum=PACKAGE/'SHA256SUMS.txt'
    checksum.write_text('\n'.join(f'{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(PACKAGE).as_posix()}' for p in files)+'\n',encoding='utf-8')
    files.append(checksum)
    with zipfile.ZipFile(ZIP,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in files:z.write(p,f'{PACKAGE.name}/{p.relative_to(PACKAGE).as_posix()}')
    with zipfile.ZipFile(ZIP) as z:
        assert z.testzip() is None
        for p in files:assert z.read(f'{PACKAGE.name}/{p.relative_to(PACKAGE).as_posix()}')==p.read_bytes()
    assert (PACKAGE/'best.pt').read_bytes()==(RUN/'run/weights/best.pt').read_bytes()
    (SHARE/'新模型v2_转发说明.txt').write_text('工创视觉新模型v2：已加入Person，并保留有毒、感叹号识别。类别固定0 Person蓝、1 Interference橙、2 Hazardous红。包内含best.pt、图片推理代码、依赖说明、样图和训练配置。解压先看README。本次376张、60轮；尚无独立验证，新场景及Jetson部署需进一步测试。',encoding='utf-8-sig')
    print(json.dumps({'zip':str(ZIP),'size_MB':round(ZIP.stat().st_size/1e6,2),'files':len(files),'verification':'PASS'},ensure_ascii=False))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--archive',action='store_true');args=parser.parse_args()
    archive() if args.archive else prepare()
