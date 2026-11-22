"""Produce offline interactive views and an overview of the extracted STEP parts."""
from pathlib import Path
import json
import numpy as np
import plotly.graph_objects as go
from plotly.offline import get_plotlyjs
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT=Path(__file__).resolve().parent

def main():
    records=json.loads((ROOT/'candidates.json').read_text(encoding='utf-8'))
    font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
    colors=['#2878A5','#207C6B','#B06F34','#7E64AC','#9C5A73','#4B7E99']
    cards=[];scripts=[]
    fig=plt.figure(figsize=(15,11),facecolor='#f5f7fa')
    for i,(record,color) in enumerate(zip(records,colors)):
        mesh=np.load(ROOT/f'{record["id"]}.npz');v=mesh['vertices'];f=mesh['faces']
        trace=go.Mesh3d(x=v[:,0],y=v[:,1],z=v[:,2],i=f[:,0],j=f[:,1],k=f[:,2],
            color=color,flatshading=False,lighting=dict(ambient=.5,diffuse=.8,specular=.25,roughness=.7),
            lightposition=dict(x=200,y=100,z=300),hoverinfo='skip')
        layout=dict(margin=dict(l=0,r=0,t=0,b=0),paper_bgcolor='#ffffff',
            scene=dict(aspectmode='data',xaxis=dict(visible=False),yaxis=dict(visible=False),zaxis=dict(visible=False),
                       camera=dict(eye=dict(x=1.6,y=-1.8,z=1.3))),showlegend=False)
        plot=go.Figure(trace,layout=layout).to_plotly_json()
        from plotly.utils import PlotlyJSONEncoder
        size=' × '.join(f'{s:.1f}' for s in record['size_mm'])+' mm'
        cards.append(f'<section><h2>{i+1:02d} · {record["name"]}</h2><p>{size} · '+('装配体' if record['assembly'] else '零件')+f'</p><div class="model" id="{record["id"]}"></div></section>')
        scripts.append('Plotly.newPlot('+json.dumps(record['id'])+','+json.dumps(plot['data'],cls=PlotlyJSONEncoder)+','+json.dumps(layout)+', {responsive:true, displaylogo:false, scrollZoom:true});')
        ax=fig.add_subplot(2,3,i+1,projection='3d',facecolor='#ffffff')
        poly=Poly3DCollection(v[f],facecolor=color,edgecolor=(0,0,0,.1),linewidth=.12)
        ax.add_collection3d(poly)
        span=float(np.max(np.ptp(v,axis=0)))*.56
        center=(v.min(axis=0)+v.max(axis=0))/2
        ax.set_xlim(center[0]-span,center[0]+span);ax.set_ylim(center[1]-span,center[1]+span);ax.set_zlim(center[2]-span,center[2]+span)
        ax.set_box_aspect((1,1,1));ax.view_init(elev=25,azim=-55);ax.set_axis_off()
        ax.set_title(f'{i+1:02d}  {record["name"]}\n{size}',fontproperties=font,fontsize=13,pad=0,color='#182b3c')
    fig.suptitle('飞机 STEP 原始零件候选',fontproperties=font,fontsize=22,color='#182b3c')
    fig.text(.5,.035,'按原模型名称提取；待确认打印版本。尺寸为包围盒，不代表孔距或配合尺寸。',ha='center',fontproperties=font,fontsize=11,color='#526476')
    fig.subplots_adjust(top=.88,bottom=.08,wspace=.02,hspace=.15)
    fig.savefig(ROOT/'候选零件总览.png',dpi=140,facecolor=fig.get_facecolor());plt.close(fig)
    html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>飞机零件候选预览</title>
<style>*{box-sizing:border-box}body{margin:0;background:#f0f4f7;color:#182b3c;font-family:"Microsoft YaHei",sans-serif}main{max-width:1400px;margin:auto;padding:30px}h1{font-size:28px;margin:0 0 10px}header p{line-height:1.8;color:#526476}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:20px}section{background:white;border:1px solid #dbe4eb;border-radius:12px;padding:18px}h2{font-size:18px;margin:0 0 7px}section p{font-size:13px;color:#647789;margin:0}.model{height:380px}@media(max-width:850px){.grid{grid-template-columns:1fr}}</style>
<main><header><h1>飞机零件候选 · 原模型提取</h1><p>鼠标拖动旋转，滚轮缩放。双击恢复视角。<br>先确认电池仓版本与雷达保护结构；尚未生成打印文件。尺寸为包围盒，单位毫米。</p></header><div class="grid">'''+''.join(cards)+'''</div></main><script>'''+get_plotlyjs()+'''</script><script>'''+''.join(scripts)+'''</script></html>'''
    (ROOT/'候选零件预览.html').write_text(html,encoding='utf-8')
    print('PREVIEWS_READY',ROOT/'候选零件预览.html')

if __name__=='__main__':main()
