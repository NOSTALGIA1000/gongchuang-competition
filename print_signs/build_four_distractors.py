"""Build four text-free A4 distractor pages from published standard artwork."""
from pathlib import Path
import json
import shutil
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader

ROOT = Path('F:/工创比赛/国标标识打印包')
OUT = ROOT / '08_干扰项4张'
OUT.mkdir(exist_ok=True)
ITEMS = [
    ('注意行人_统一黄色', ROOT / '08_干扰项4张/注意行人_统一黄色.png', 'GB 5768.2-2022 警10图形；按用户要求黄色改为其他三张图的主色RGB(224,188,33)，不是源文件原色', 'https://rijbewijs-online.be/img/traffic-signs/cn/warning/warning-pedestrian-crossing.svg'),
    ('当心落物', ROOT / '02_干扰候选_非三类目标/1-14_当心落物.jpeg', 'GB 2894-2025', 'https://xcoss.henan.gov.cn/typtfile/20250701/1c8b5ad235d748d1b5437a650fb19cb4.pdf'),
    ('当心触电', ROOT / '02_干扰候选_非三类目标/1-07_当心触电.jpeg', 'GB 2894-2025', 'https://xcoss.henan.gov.cn/typtfile/20250701/1c8b5ad235d748d1b5437a650fb19cb4.pdf'),
    ('当心滑倒', ROOT / '02_干扰候选_非三类目标/1-37_当心滑倒.jpeg', 'GB 2894-2025', 'https://xcoss.henan.gov.cn/typtfile/20250701/1c8b5ad235d748d1b5437a650fb19cb4.pdf'),
]
pdf_path = ROOT / '四张干扰项_A4无文字打印.pdf'
c = canvas.Canvas(str(pdf_path), pagesize=A4, pageCompression=1)
c.setTitle('四张干扰项 A4 无文字打印')
records = []
for page, (name, source, standard, url) in enumerate(ITEMS, 1):
    dest = OUT / f'{page:02d}_{name}{source.suffix}'
    shutil.copy2(source, dest)
    img = ImageReader(str(dest))
    iw, ih = img.getSize()
    scale = min(180 * mm / iw, 180 * mm / ih)
    width, height = iw * scale, ih * scale
    c.drawImage(img, (A4[0] - width) / 2, (A4[1] - height) / 2,
                width=width, height=height, mask='auto')
    c.showPage()
    records.append(dict(page=page, name=name, file=dest.name, standard=standard, source=url))
c.save()
(OUT / '来源与页序.json').write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')
print(pdf_path)
