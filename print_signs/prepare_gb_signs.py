"""Extract unmodified official GB 2894-2025 artwork and make print sheets."""
from pathlib import Path
import re
import json
import hashlib
import fitz

ROOT = Path('F:/工创比赛/国标标识打印包')
SOURCE = ROOT / '来源/GB2894-2025_安全色和安全标志.pdf'
SOURCE_URL = 'https://xcoss.henan.gov.cn/typtfile/20250701/1c8b5ad235d748d1b5437a650fb19cb4.pdf'
TARGET = {'1-02', '1-03', '1-05'}
# Only clear mechanical/electrical/environmental pictograms enter candidate negatives.
NEGATIVE = {'1-01','1-07','1-08','1-09','1-10','1-11','1-12','1-13',
            '1-14','1-15','1-16','1-17','1-18','1-19','1-20','1-21',
            '1-22','1-24','1-25','1-26','1-29','1-30','1-31','1-32',
            '1-33','1-34','1-35','1-36','1-37','1-38','1-39','1-40',
            '1-44','1-45','1-46','1-47','1-48'}
GROUPS = {'target':'01_明确危险品目标','negative':'02_干扰候选_非三类目标',
          'review':'03_相近危险标识_待赛方确认'}

def text(page, xy, value, size=11):
    page.insert_text(xy, value, fontname='china-s', fontsize=size, color=(0,0,0))

def place(page, row, rect):
    page.insert_image(rect, filename=str(ROOT / row['file']), keep_proportion=True)

def print_book(rows, path):
    doc = fitz.open()
    mm = 72 / 25.4
    for row in rows:
        page = doc.new_page(width=210*mm, height=297*mm)
        # No class/name hints on the printed sample. Cut or mask the bottom ID.
        rect = fitz.Rect(15*mm, 50*mm, 195*mm, 230*mm)
        place(page, row, rect)
        page.insert_text((15*mm, 278*mm), 'GB 2894-2025 | '+row['code']+' | 100% / Actual size', fontsize=8)
        page.draw_line((15*mm,270*mm),(65*mm,270*mm), color=(0,0,0),width=.5)
        page.insert_text((15*mm,274*mm),'50 mm',fontsize=7)
    doc.set_metadata({'title':path.stem,'subject':'Official Chinese standard artwork; practice print sheets, not certified safety signs.'})
    doc.save(path, garbage=4, deflate=True)
    doc.close()

def main():
    source = fitz.open(SOURCE)
    rows=[]
    for pi in range(18,32):
        page=source[pi]
        ids=[]
        for b in page.get_text('blocks'):
            code=b[4].strip()
            if b[0]<110 and re.fullmatch(r'1-\d{2}',code):
                ids.append((code,(b[1]+b[3])/2))
        images=[i for i in page.get_image_info(xrefs=True)
                if i['width']>500 and 100<i['bbox'][0]<300]
        for image in images:
            rect=fitz.Rect(image['bbox'])
            choices=[(code,y) for code,y in ids if rect.y0<=y<=rect.y1]
            if len(choices)!=1:
                continue
            code,_=choices[0]
            cell=page.get_text('text',clip=fitz.Rect(305,rect.y0,393,rect.y1))
            chinese_cell=re.split(r'[A-Za-z]',cell,maxsplit=1)[0]
            name=''.join(re.findall(r'[\u4e00-\u9fff、（）()]+',chinese_cell))
            assert name, (code,cell)
            group='target' if code in TARGET else 'negative' if code in NEGATIVE else 'review'
            folder=ROOT/GROUPS[group];folder.mkdir(parents=True,exist_ok=True)
            raw=source.extract_image(image['xref'])
            out=folder/(code+'_'+name+'.'+raw['ext'])
            for previous in folder.glob(code+'_*'):
                if previous!=out and previous.is_file() and previous.read_bytes()==raw['image']:
                    previous.rename(out)
            out.write_bytes(raw['image'])
            rows.append({'code':code,'name':name,'group':group,'source_page':pi+1,
                         'width':raw['width'],'height':raw['height'],
                         'file':str(out.relative_to(ROOT)),
                         'sha256':hashlib.sha256(raw['image']).hexdigest()})
    rows.sort(key=lambda r:r['code'])
    assert len(rows)==52, (len(rows),[r['code'] for r in rows])
    assert len({r['code'] for r in rows})==52
    assert {r['code'] for r in rows if r['group']=='target'}==TARGET
    (ROOT/'图样清单.json').write_text(json.dumps({'standard':'GB 2894-2025',
        'effective_date':'2026-03-01','source':SOURCE_URL,
        'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        'artwork_processing':'Original embedded images extracted without recolouring, sharpening or redrawing.',
        'items':rows},ensure_ascii=False,indent=2),encoding='utf-8')
    print_book([r for r in rows if r['group']=='target'],ROOT/'01_危险品目标_A4打印.pdf')
    print_book([r for r in rows if r['group']=='negative'],ROOT/'02_干扰候选_A4打印.pdf')
    catalog=fitz.open()
    for offset in range(0,len(rows),8):
        page=catalog.new_page(width=595.28,height=841.89)
        text(page,(34,35),'GB 2894-2025 新国标标识索引',16)
        text(page,(34,56),'2026-03-01实施 | 图样保持官方原图 | 分类为赛训建议',9)
        for j,row in enumerate(rows[offset:offset+8]):
            x=34+(j%2)*278;y=80+(j//2)*180
            place(page,row,fitz.Rect(x+40,y,x+218,y+137))
            text(page,(x,y+153),row['code']+' '+row['name'],9)
            tag={'target':'明确目标','negative':'干扰候选（需赛方确认）','review':'相近危险标识：暂勿作为干扰'}[row['group']]
            text(page,(x,y+168),tag,8)
        page.insert_text((34,823),f'{offset//8+1} / 7',fontsize=8)
    catalog.save(ROOT/'03_全部52种_图样与分组索引.pdf',garbage=4,deflate=True)
    catalog.close()
    lines=['# GB 2894-2025 新国标实拍打印素材包','',
      '现行标准：GB 2894—2025《安全色和安全标志》，2025-05-30发布，2026-03-01实施。',
      '本包52种警告标志全部直接提取自政府网站标准原文附录C表C.1，未使用国外标识、旧版截图或AI重绘。',
      '原文来源：'+SOURCE_URL,
      '现行状态核验：https://openstd.samr.gov.cn/bzgk/std/newGbInfo?hcno=26CD7C428F2BF6AE971F84D22D7B72BB',
      '', '## 赛题与分类边界',
      '赛题第3页：两个危险品任务区均有图像，一个为易燃、易爆、有毒等危险品标识，另一个为其它图像干扰项；具体图样抽签决定，示意图不是固定图库。',
      '赛题第5页：识别正确危险品后用红框标注并向正确目标投放；人员目标用蓝框标注。',
      '明确目标：1-02 当心火灾、1-03 当心爆炸、1-05 当心中毒，训练类别为Hazardous。',
      '干扰候选为注意安全及机械、电气、跌落等非本次明确三类图样；这是赛训候选分组，不是说这些标志没有危险，也不是赛方确认的干扰清单。',
      '腐蚀、生物危害、辐射、氧化物、气瓶等保留在待确认组，不能仅凭它们不是火灾/爆炸/中毒就标成Interference。赛题中的“等”尚未封闭危险品范围。',
      '禁止烟火/禁止吸烟的含义是禁止行为，不能因为带火焰就默认等同当心火灾；本包仅收录警告标志，避免混义。',
      '', '## 打印',
      '先看03索引，再按需要打印01目标与02干扰候选。PDF一页一图，A4彩色、实际大小/100%，关闭适合页面和省墨模式，使用哑光纸。',
      '图像保持比例，放在180×180 mm区域内；这不是所有三角形外边长180 mm。下方50 mm校准线可检查打印缩放。',
      '图样底部没有附加类别提示文字；页脚是管理编号和校准线，拍摄前沿页脚上方裁掉或遮住，避免模型学到编号。',
      '400×400 mm是赛题任务区尺寸，不是要求图标尺寸；用400×400 mm哑光底板搭建任务区，具体图标实际尺寸以赛方提供为准。',
      '如需更大打印，使用各组原图交给打印店按比例排版，禁止任意拉伸、改颜色或修改图案。颜色以标准原文/色度要求为准，普通显示器和打印机不能保证认证级色度。',
      '本包是识别训练/练习打印素材，不是认证安全标牌或危险品运输标签，不用于实际危险化学品标签合规。',
      '', '## 采集与训练',
      '按用户方案，背景、距离、俯视角、光照、遮挡、运动模糊通过真实无人机相机采集；本包不生成这些人工变体。',
      '每个目标图样搭配多种干扰候选，两者同时入镜；交换左右、方位与位置，避免模型学位置。也拍单图、无人头/标识的背景，以及真实人员任务图。',
      '原始素材按拍摄日期/批次保留；同段视频和近邻帧只属于一个训练/验证/测试集合。',
      '先确认赛方目标/干扰定义，再标注Person=0、Interference=1、Hazardous=2。待确认图样不提前生成训练标签。',
      '', '## 图样清单','|编号|名称|建议分组|标准PDF页|','|---|---|---|---|']
    for r in rows:lines.append('|'+r['code']+'|'+r['name']+'|'+GROUPS[r['group']]+'|'+str(r['source_page'])+'|')
    (ROOT/'先读我_打印与分类说明.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    preview=ROOT/'预览';preview.mkdir(exist_ok=True)
    for filename,indices in [('01_危险品目标_A4打印.pdf',[0,1,2]),('02_干扰候选_A4打印.pdf',[0,10,20,36]),('03_全部52种_图样与分组索引.pdf',list(range(7)))]:
        d=fitz.open(ROOT/filename)
        for i in indices:
            if i<len(d):d[i].get_pixmap(matrix=fitz.Matrix(.85,.85)).save(str(preview/(Path(filename).stem+f'_{i+1}.png')))
    print(json.dumps({'total':len(rows),'groups':{g:sum(r['group']==g for r in rows) for g in GROUPS},'root':str(ROOT)},ensure_ascii=False))
    for r in rows: print(r['code'],r['name'],r['group'],r['width'],r['height'])

if __name__=='__main__':main()
