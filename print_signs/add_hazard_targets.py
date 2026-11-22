"""Extract GB 190 scan artwork as-is; keep provenance and printing limits."""
from pathlib import Path
import json,hashlib,shutil
import fitz

ROOT=Path('F:/工创比赛/国标标识打印包')
OUT=ROOT/'06_危险品目标补充_中国现行标准'
SCAN=Path('F:/工创比赛/print_signs/sources/GB190_2009.pdf')
URL='https://sbc.ecust.edu.cn/_upload/article/files/0b/06/7494bf2a4c65abeec7b78ba67d5a/d701981f-41d0-428e-b5ed-a7307e75bc3e.pdf'
# Clip coordinates measured on 1.3x page renders, preserving the scan's colour.
SPECS=[
 ('易爆','EXP_01','爆炸性物质和物品_1.1至1.3_原文占位示例',151,(365,190,533,353)),
 ('易爆','EXP_02','爆炸性物质和物品_1.4_原文占位示例',151,(367,377,532,541)),
 ('易爆','EXP_03','爆炸性物质和物品_1.5_原文占位示例',151,(365,566,533,731)),
 ('易爆','EXP_04','爆炸性物质和物品_1.6_原文占位示例',151,(365,756,533,920)),
 ('易燃','FLAM_01','易燃气体_2.1_黑符号',152,(372,207,527,365)),
 ('易燃','FLAM_02','易燃气体_2.1_白符号',152,(372,400,529,558)),
 ('有毒','TOX_01','毒性气体_2.3',153,(378,194,543,356)),
 ('易燃','FLAM_03','易燃液体_3_黑符号',153,(371,390,555,571)),
 ('易燃','FLAM_04','易燃液体_3_白符号',153,(371,592,555,776)),
 ('易燃','FLAM_05','易燃固体_4.1',153,(370,799,561,987)),
 ('易燃','FLAM_06','易于自燃的物质_4.2',154,(354,195,539,383)),
 ('易燃','FLAM_07','遇水放出易燃气体的物质_4.3_黑符号',154,(363,434,525,589)),
 ('易燃','FLAM_08','遇水放出易燃气体的物质_4.3_白符号',154,(363,605,525,763)),
 ('有毒','TOX_02','毒性物质_6.1',155,(348,568,543,761)),
]

def main():
    OUT.mkdir(exist_ok=True);source=fitz.open(SCAN);rows=[]
    original=json.loads((ROOT/'图样清单.json').read_text(encoding='utf-8'))['items']
    for r in original:
        if r['group']!='target':continue
        family={'1-02':'易燃','1-03':'易爆','1-05':'有毒'}[r['code']]
        folder=OUT/'原有3张新国标警告标志';folder.mkdir(exist_ok=True)
        target=folder/Path(r['file']).name;shutil.copy2(ROOT/r['file'],target)
        rows.append({'id':r['code'],'name':r['name'],'family':family,'standard':'GB 2894-2025',
                     'file':str(target.relative_to(ROOT)),'width':r['width'],'height':r['height'],
                     'source_page':r['source_page'],'quality':'标准内嵌原图','status':'赛题明确危险属性对应警告标志'})
    for family,code,name,page_number,coords in SPECS:
        folder=OUT/('新增_'+family);folder.mkdir(exist_ok=True)
        rect=fitz.Rect([v/1.3 for v in coords]);page=source[page_number-1]
        # Render at approximately native scan density: do not fake resolution.
        matrix=fitz.Matrix(1240/596,1753/842)
        pix=page.get_pixmap(matrix=matrix,clip=rect,alpha=False)
        path=folder/(code+'_'+name+'.png');pix.save(path)
        rows.append({'id':code,'name':name,'family':family,'standard':'GB 190-2009',
          'file':str(path.relative_to(ROOT)),'width':pix.width,'height':pix.height,
          'source_page_in_ecust_collection':page_number,'source_printed_page':page_number-148,
          'clip_pdf_points':list(rect),'source':URL,'quality':'彩色扫描原样裁取，未锐化、重绘或改色',
          'status':'扩展目标候选；具体赛方图库待确认'})
    # Add high quality black symbol references from the current Chinese general rules.
    new_standard=fitz.open(ROOT/'来源/GB30000.1-2024_通则.pdf')
    refs=[]
    for code,name,rect in [('SYMBOL_FLAME','火焰',(151,102,203,174)),
                           ('SYMBOL_EXPLOSION','爆炸的炸弹',(367,105,450,174)),
                           ('SYMBOL_TOXIC','骷髅和交叉骨',(367,222,447,278))]:
        folder=OUT/'符号参考_非完整安全标签';folder.mkdir(exist_ok=True)
        path=folder/(code+'_'+name+'.png')
        new_standard[9].get_pixmap(matrix=fitz.Matrix(8.333,8.333),clip=fitz.Rect(rect)).save(path)
        refs.append({'id':code,'name':name,'standard':'GB 30000.1-2024','source_page':10,
                     'file':str(path.relative_to(ROOT)),
                     'status':'仅危险符号，不是完整红框象形图；不计入17张目标打印图'})
    for r in rows:r['sha256']=hashlib.sha256((ROOT/r['file']).read_bytes()).hexdigest()
    (OUT/'补充目标清单.json').write_text(json.dumps({'target_candidates':rows,'symbol_references':refs,
       'warning':'Official standard examples are not organizer-confirmed competition artwork.'},ensure_ascii=False,indent=2),encoding='utf-8')
    mm=72/25.4
    for name,selected,index in [('危险品目标候选_17张_A4打印.pdf',rows,False),('危险品目标候选_17张分类索引.pdf',rows,True)]:
        doc=fitz.open()
        if index:
            for offset in range(0,len(selected),6):
                p=doc.new_page(width=210*mm,height=297*mm)
                p.insert_text((28,30),'China GB / Hazard target candidates',fontsize=15)
                p.insert_text((28,48),'17 artworks: 3 warnings + 14 transport-label examples',fontsize=9)
                for j,r in enumerate(selected[offset:offset+6]):
                    x=28+(j%2)*280;y=75+(j//2)*240
                    p.insert_image(fitz.Rect(x+30,y,x+220,y+175),filename=str(ROOT/r['file']))
                    short=r['name'].replace('_原文占位示例','')
                    p.insert_textbox(fitz.Rect(x,y+182,x+255,y+218),r['id']+' '+short,fontname='china-s',fontsize=9)
                    p.insert_text((x,y+232),r['standard'],fontsize=8)
        else:
            for r in selected:
                p=doc.new_page(width=210*mm,height=297*mm)
                # Keep every candidate the same print envelope to avoid class/size hints.
                p.insert_image(fitz.Rect(45*mm,75*mm,165*mm,195*mm),filename=str(ROOT/r['file']))
                p.draw_line((15*mm,270*mm),(65*mm,270*mm),width=.5)
                p.insert_text((15*mm,274*mm),'50 mm',fontsize=7)
                p.insert_text((15*mm,278*mm),r['standard']+' | '+r['id']+' | 100% / Actual size',fontsize=8)
        doc.save(OUT/name,deflate=True,garbage=4)
        preview=OUT/'预览';preview.mkdir(exist_ok=True)
        for i in range(len(doc)):
            doc[i].get_pixmap(matrix=fitz.Matrix(.7,.7)).save(str(preview/(('索引' if index else '打印')+f'_{i+1}.png')))
        doc.close()
    notes='''# 危险品目标补充：三种危险属性，多种标准图样

本轮补充14张GB 190—2009标准标签示例，加上原有3张GB 2894—2025警告标志，共17张可打印目标候选。
新增：易燃8张、易爆4张、有毒2张。不是17个模型类别；确认属于比赛目标后统一归Hazardous。
全部原图是中国标准原文图样，不用外国标识冒充中国国标，不用AI重绘。原有2025年警告标志仍保留。

## 来源与现行状态
- GB 190—2009《危险货物包装标志》：全国标准信息系统现行。大学公开的标准合订本中，PDF第147页为封面，第151至155页含本次裁取的表2图样。扫描颜色/锐度有限。
- 合订本来源：https://sbc.ecust.edu.cn/_upload/article/files/0b/06/7494bf2a4c65abeec7b78ba67d5a/d701981f-41d0-428e-b5ed-a7307e75bc3e.pdf
- GB190现行状态：https://openstd.samr.gov.cn/bzgk/std/newGbInfo?hcno=9B64ABB5E55498AC460E240EBDC811B9
- GB 30000.1—2024《化学品分类和标签规范 第1部分：通则》2025-08-01实施，代替GB13690—2009。第10页图1有火焰、爆炸的炸弹、骷髅和交叉骨等危险符号，另存3张符号参考。这些没有红色边框，不是完整安全标签，未混入17张目标打印稿。
- 通则原文：https://chemec.lzu.edu.cn/hdupf/fil/202509/202509231758608317620626.pdf
- 通则现行状态：https://openstd.samr.gov.cn/bzgk/std/newGbInfo?hcno=C008AEDBFD9A16F3C5BEB671C20618DD
- GB 15258—2009《化学品安全标签编写规定》仍列现行，但它不是所有象形图的独立图集，修订征求意见稿不能当作已生效版本。

## 重要边界
1. GB190图样是危险货物运输包装标签，与GB2894安全警告标志是不同体系。对应危险属性明确，不代表赛方一定采用这些图库。
2. 爆炸品图中的星号是原文用于项别/配装组的占位示例，原样保留；本包不随意填造运输信息，不用于真实运输包装合规。
3. 绿色非易燃无毒气体、氧化性物质、感染性物质、放射性物质未作为本轮易燃易爆有毒目标加入，也不能据此直接标成干扰。
4. 扫描原图每张约250至310像素。打印稿用120×120mm容纳框并保留比例；放大不能增加真实细节。本包适合先行实拍参考，大幅正式打印优先换取更清晰的原始标准图样。
5. 原图只做PDF页区域裁取/正常渲染，不改色、不锐化、不去水印、不用AI补细节。PNG不是宣称高分辨率的重绘文件。

## 打印和实拍
先看17张分类索引，再选实际需要的页。A4、彩色、实际大小100%、哑光纸；按50mm线检查比例。拍摄前裁去/遮住页脚，避免学到编号。
目标与干扰都使用同样底板与接近尺寸；交换左右位置，背景、光照、俯视角、遮挡、模糊按用户计划由实际无人机相机采集。
没有生成YOLO标签，也未启动训练。训练前按赛方目标范围决定哪些扩展候选进入Hazardous，确认后统一固定类别映射。
'''
    (OUT/'先读我_来源与打印质量.md').write_text(notes,encoding='utf-8')
    # Copy the relevant source section without changing content for local audit.
    excerpt=fitz.open();excerpt.insert_pdf(source,from_page=146,to_page=163)
    excerpt.save(ROOT/'来源/GB190-2009_合订本原文节选.pdf',garbage=4,deflate=True)
    print(json.dumps({'new':14,'total_printable':len(rows),'symbol_references':len(refs),
                      'by_family':{f:sum(r['family']==f for r in rows) for f in ['易燃','易爆','有毒']}},ensure_ascii=False))

if __name__=='__main__':main()
