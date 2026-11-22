"""Package the reviewed YOLO samples without modifying the source dataset."""
from pathlib import Path
import json
import hashlib
import zipfile

BASE = Path('F:/工创比赛')
SOURCE = BASE / 'real_capture/datasets/camera_roll_20261114_final_v1'
OUT = BASE / '队友分享'
OUT.mkdir(exist_ok=True)
ARCHIVE = OUT / '工创视觉_YOLO已标注数据_130张_v1.zip'
PREFIX = 'gongchuang_yolo_130_v1/'
README = '''# 工创视觉 YOLO 标注数据 v1

## 本包内容
- images/：130 张真实相机图片。
- labels/：130 份同名 YOLO 检测标签。
- classes.txt：固定类别顺序。
- previews/标注示例.png：红框危险品、橙框干扰项。
- dataset_collection.yaml：采集阶段配置，不是完整训练配置。
- SHA256SUMS.txt：包内各文件的 SHA256 校验值。

## 类别与数量
0 Person：0 个；赛事人员救助目标为人头图像，不把画面里的脚或鞋标为 Person。
1 Interference：100 个；本批为感叹号标识。
2 Hazardous：113 个；本批为有毒骷髅标识。
合计 130 张、213 个目标；类别编号不要改动。

## 标签格式
每张图片对应 labels/ 中同名 .txt。每个目标占一行：
class_id x_center y_center width height
坐标按原图宽、高归一化到 0～1。框包含三角标志外缘，不包括白纸和红色任务区外框。
YOLO 标签不保存框颜色；预览图的颜色只是显示约定。
使用 LabelImg 时选择 YOLO 格式，打开 images/，保存目录设为 labels/，加载 classes.txt。

## 当前状态与训练前要求
原始 161 张完成目视复核，保留 130 张，排除 31 张，待复核 0；本包只含保留图片。
图片与标签配对、类别编号、归一化坐标和边界检查通过。尚未训练。
当前仅同一场景的两种标识；没有 Person、易燃、易爆样本，也没有独立验证/测试场景。
继续补齐真实拍摄数据，并按独立拍摄批次或场景划分训练、验证、测试集。
不要把相邻连拍随机分到训练和验证集，以免结果虚高。
采集配置故意没有 val；补齐独立验证集后再写训练 data.yaml。配置 path 应改成队友电脑解压后的绝对路径。
目视复核不等于独立标注精度评估，也不代表模型泛化精度。
'''
CONFIG = '''# Collection only: update path after extracting; add independent val before training.
path: .
train: images
names:
  0: Person
  1: Interference
  2: Hazardous
'''
files = {}
for folder in ('images', 'labels'):
    for p in sorted((SOURCE / folder).iterdir()):
        if p.is_file() and p.name != 'classes.txt':
            files[f'{folder}/{p.name}'] = p.read_bytes()
files['classes.txt'] = (SOURCE/'classes.txt').read_bytes()
files['README_先看我.md'] = README.encode('utf-8-sig')
files['dataset_collection.yaml'] = CONFIG.encode('utf-8')
files['previews/标注示例.png'] = (SOURCE/'qa/example_final_0002.png').read_bytes()
files['summary.json'] = (SOURCE/'summary.json').read_bytes()
files['SHA256SUMS.txt'] = ('\n'.join(f'{hashlib.sha256(data).hexdigest()}  {name}'
                                     for name, data in sorted(files.items()))+'\n').encode('utf-8')
with zipfile.ZipFile(ARCHIVE, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
    for name, data in files.items():
        z.writestr(PREFIX+name, data)
with zipfile.ZipFile(ARCHIVE) as z:
    assert z.testzip() is None
    names = z.namelist()
    assert sum(n.startswith(PREFIX+'images/') for n in names) == 130
    assert sum(n.startswith(PREFIX+'labels/') for n in names) == 130
    for name, data in files.items():
        assert z.read(PREFIX+name) == data
message = '''工创视觉第一批已标注数据：130 张真实拍摄图片，YOLO 格式，213 个目标（危险品 113、干扰项 100）。类别固定：0 Person、1 Interference、2 Hazardous。含原图、标签、预览与使用说明，解压先看 README。尚未训练；还需补齐 Person、易燃易爆及独立验证场景，避免把相邻连拍随机拆分训练/验证。'''
(OUT/'发给队友的说明.txt').write_text(message, encoding='utf-8-sig')
print(json.dumps({'zip':str(ARCHIVE),'size_MB':round(ARCHIVE.stat().st_size/1e6,2),
                  'images':130,'labels':130,'crc_and_content_verification':'PASS'},ensure_ascii=False))
