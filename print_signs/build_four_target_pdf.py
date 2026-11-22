"""Four A4 print pages: three GB warning signs and the original competition head."""
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont

ROOT=Path('F:/工创比赛/国标标识打印包')
OUTPUT=ROOT/'三个危险品与Person_A4打印.pdf'
ITEMS=[
 ('当心火灾',ROOT/'01_明确危险品目标/1-02_当心火灾.jpeg'),
 ('当心爆炸',ROOT/'01_明确危险品目标/1-03_当心爆炸.jpeg'),
 ('当心中毒',ROOT/'01_明确危险品目标/1-05_当心中毒.jpeg'),
 ('Person 人员救助目标',ROOT/'07_Person_赛题示例/person_from_competition_pdf.png'),
]

def main():
    pdfmetrics.registerFont(UnicodeCIDFont('STSong-Light'))
    c=canvas.Canvas(str(OUTPUT),pagesize=A4,pageCompression=1)
    c.setTitle('三个危险品与Person - A4打印')
    c.setSubject('GB 2894-2025 warning artwork and original low-resolution competition head example')
    width,height=A4
    for i,(name,path) in enumerate(ITEMS,1):
        c.bookmarkPage(f'target_{i}');c.addOutlineEntry(name,f'target_{i}',level=0)
        image=ImageReader(str(path));iw,ih=image.getSize()
        scale=min(180*mm/iw,180*mm/ih)
        dw,dh=iw*scale,ih*scale
        x,y=(width-dw)/2,(height-dh)/2
        c.saveState()
        if i==4:
            # The last source row belongs to the diagram's red dimension line.
            # Clip that row in PDF layout; retain the complete black picture border.
            clip=c.beginPath();clip.rect(x,y+dh/ih,dw,dh-dh/ih);c.clipPath(clip,stroke=0)
        c.drawImage(image,x,y,width=dw,height=dh,mask='auto')
        c.restoreState()
        c.showPage()
    c.save()
    print(OUTPUT)

if __name__=='__main__':main()
