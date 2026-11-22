"""Collect current Chinese-standard non-target pictograms without assigning YOLO labels."""
from pathlib import Path
import fitz,re,json,hashlib

ROOT=Path('F:/工创比赛/国标标识打印包')
OUT=ROOT/'05_扩展干扰候选_新国标'
PDF=ROOT/'来源/GB2894-2025_安全色和安全标志.pdf'
SPECIAL={'2-01','2-02','2-03','2-04','2-05','2-37','2-38','2-42','2-46','2-47','2-50','3-04','3-24','3-25','4-05'}
PRIORITY=['2-02','2-03','2-04','2-05','2-37','2-38','2-46','2-47',
          '2-11','2-13','2-16','2-17','2-18','2-22','2-34','2-43',
          '3-01','3-03','3-04','3-06','3-07','3-09','4-01','4-04','4-07']

def main():
    OUT.mkdir(exist_ok=True)
    doc=fitz.open(PDF);rows=[]
    for pi in range(31,54):
        page=doc[pi];ids=[]
        for b in page.get_text('blocks'):
            if b[0]<110 and re.fullmatch(r'[234]-\d{2}',b[4].strip()):ids.append((b[4].strip(),(b[1]+b[3])/2))
        for image in page.get_image_info(xrefs=True):
            rect=fitz.Rect(image['bbox'])
            if image['width']<500 or not 100<rect.x0<300:continue
            choices=[(code,y) for code,y in ids if rect.y0-16<=y<=rect.y1+16]
            if len(choices)!=1:continue
            code=choices[0][0]
            if code=='4-01':name='安全出口'
            else:
                # New tables begin with narrower name columns on these pages.
                # Artwork width cannot identify a column: some symbols are narrower.
                left,right=(260,350) if pi in {31,44,51} and (pi!=44 or code.startswith('3-')) else (297,393)
                cell=page.get_text('text',clip=fitz.Rect(left,rect.y0,right,rect.y1))
                name=''.join(re.findall(r'[\u4e00-\u9fff、（）()]+',re.split('[A-Za-z]',cell,maxsplit=1)[0]))
            assert name,(pi,code)
            family={'2':'禁止标志','3':'指令标志','4':'提示标志'}[code[0]]
            folder=OUT/family;folder.mkdir(exist_ok=True)
            variant=sum(r['code']==code for r in rows)+1
            suffix=f'_图样{variant}' if code=='4-01' else ''
            raw=doc.extract_image(image['xref']);p=folder/(code+'_'+name+suffix+'.'+raw['ext']);p.write_bytes(raw['image'])
            for previous in folder.glob(code+'_*'):
                if previous!=p and previous.is_file() and previous.read_bytes()==raw['image'] and code!='4-01':
                    previous.unlink()
            rows.append({'code':code,'variant':variant,'name':name,'family':family,'source_page':pi+1,
                         'file':str(p.relative_to(ROOT)),'width':raw['width'],'height':raw['height'],
                         'sha256':hashlib.sha256(raw['image']).hexdigest(),
                         'label_status':'需确认场景语义' if code in SPECIAL else '干扰候选_非赛方指定',
                         'reason':'火焰、危险物或防护关联，禁止行为不等于危险品类别' if code in SPECIAL else '人形/设备/动作/颜色形状与目标可能混淆'})
    assert len({r['code'] for r in rows if r['code'].startswith('2-')})==51
    assert len({r['code'] for r in rows if r['code'].startswith('3-')})==25
    assert len({r['code'] for r in rows if r['code'].startswith('4-')})==11
    assert len(rows)==88,len(rows)
    (OUT/'扩展图样清单.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
    for filename,selected,grid in [
        ('扩展干扰候选_全部88张索引.pdf',rows,True),
        ('优先实拍_25类26张_A4打印.pdf',sorted([r for r in rows if r['code'] in PRIORITY],key=lambda r:PRIORITY.index(r['code'])),False)]:
        pdf=fitz.open()
        if grid:
            for offset in range(0,len(selected),8):
                page=pdf.new_page(width=595.28,height=841.89)
                page.insert_text((30,30),'GB 2894-2025 / Extended candidate index',fontsize=14)
                page.insert_text((30,48),'Candidate examples only; class labels require scenario review.',fontsize=9)
                for j,r in enumerate(selected[offset:offset+8]):
                    x=30+(j%2)*280;y=70+(j//2)*180
                    page.insert_image(fitz.Rect(x+35,y,x+220,y+140),filename=str(ROOT/r['file']))
                    page.insert_text((x,y+155),r['code']+' '+r['name'],fontname='china-s',fontsize=8)
                    page.insert_text((x,y+169),r['label_status'],fontname='china-s',fontsize=8)
                page.insert_text((30,823),f'{offset//8+1} / {(len(selected)+7)//8}',fontsize=8)
        else:
            mm=72/25.4
            for r in selected:
                page=pdf.new_page(width=210*mm,height=297*mm)
                page.insert_image(fitz.Rect(15*mm,50*mm,195*mm,230*mm),filename=str(ROOT/r['file']))
                page.insert_text((15*mm,278*mm),f"GB 2894-2025 | {r['code']} | 100% / Actual size",fontsize=8)
                page.draw_line((15*mm,270*mm),(65*mm,270*mm),width=.5)
                page.insert_text((15*mm,274*mm),'50 mm',fontsize=7)
        pdf.save(OUT/filename,deflate=True,garbage=4)
        for i in (range(len(pdf)) if grid else [0,8,16,22]):
            p=OUT/'预览';p.mkdir(exist_ok=True)
            pdf[i].get_pixmap(matrix=fitz.Matrix(.8,.8)).save(str(p/(('索引' if grid else '打印')+f'_{i+1}.png')))
        pdf.close()
    readme='''# 扩展干扰思路与素材

全部88张原图直接提取自GB 2894—2025附录C，来源为政府网站发布的标准PDF。原图不改色、不翻转、不重绘。

## 为什么需要这些干扰
1. 禁止烟火、禁止带火种、禁止燃放烟花爆竹等：含火焰/爆炸图形，检验模型是否同时识别红色禁止外框与斜杠，而非看见火焰就判Hazardous。这些标志本身与危险物存在关联，需先确认赛方含义，不能自动写成负样本。
2. 必须戴口罩、防毒面具、安全帽、防护面罩等：含头部/人脸，检验Person类别是否识别人员救助区的人头目标，而非所有脸部符号。
3. 禁止通行、禁止攀登、禁止乘人、安全出口、紧急集合点等：含完整人形，适合作为人头图像之外的人形干扰候选。
4. 必须戴防护手套、防护鞋、洗手等：人体局部与蓝底圆形；用于避免只学形状/颜色，全部原图已保存。
5. 安全出口、急救点、应急电话、洗眼台等：绿色提示标志，增加白底/蓝底/绿底与不同轮廓类别。
6. 之前已有37种三角警告干扰候选：与危险品三角外框、黄底完全相近，优先拍摄这组以防模型只看黄色三角形。
7. 道路交通图样还包括蓝底人行横道、黄底注意行人、注意儿童、施工、信号灯、自行车/禁行、停车让行、限速、直行/转弯、停车场等。相关标准另属GB 5768.2—2022及第1号修改单，不能拿旧图库或国外图冒充2026现行原图；当前这批88张不包含道路交通图样。
8. 非标普通图像：数字、字母、二维码、几何图、动物、卡通人头、品牌图形、空白纸也可能是赛题所说“其它图像”。这些不属于国标素材，本次不伪造国标来源。卡通头是否属于Person必须由赛题目标定义决定。

## 分组不是训练标签
所有图片都只是候选，未生成任何YOLO负标签。禁止图案不表示所在现场一定无危险；特别是禁止携带易燃易爆/有毒物品、可动火区、防毒面具等，不应未经确认混进Interference。赛方允许的目标范围优先。

## 打印与实拍
先看全部88张索引，重点打印“优先实拍_25类26张_A4打印.pdf”。25类中安全出口有官方左右两个原图，因此26页。
A4、彩色、实际大小100%、哑光纸，按50mm校准线确认比例。裁去页脚再拍，避免模型学编号。相同纸张/底板/尺寸用于目标与干扰；随机交换位置。
同框拍真实危险品标志与干扰，保持相似大小；加入仅干扰、仅目标和空背景。拍摄批次整体划分数据集，不把同视频相邻帧分进训练和测试。

## 来源
标准原文：https://xcoss.henan.gov.cn/typtfile/20250701/1c8b5ad235d748d1b5437a650fb19cb4.pdf
现行状态：https://openstd.samr.gov.cn/bzgk/std/newGbInfo?hcno=26CD7C428F2BF6AE971F84D22D7B72BB
原始文件、页码、SHA256在来源目录和扩展图样清单.json中可核对。
'''
    (OUT/'先读我_扩展干扰与标签边界.md').write_text(readme,encoding='utf-8')
    print(json.dumps({'total':len(rows),'counts':{g:sum(r['family']==g for r in rows) for g in ['禁止标志','指令标志','提示标志']},'priority':sum(r['code'] in PRIORITY for r in rows)},ensure_ascii=False))
    for r in rows:
        if r['code'] in PRIORITY:print(r['code'],r['name'])

if __name__=='__main__':main()
