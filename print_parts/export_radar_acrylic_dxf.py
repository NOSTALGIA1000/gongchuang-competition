"""Export one original planar face as 1:1 mm laser-cut outlines, without duplicate faces."""
from pathlib import Path
import json
import math
import hashlib
import numpy as np
import ezdxf
from ezdxf import bbox as dxf_bbox
from shapely.geometry import Polygon
from OCP.TopoDS import TopoDS,TopoDS_Shape
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools,BRepTools_WireExplorer
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_WIRE,TopAbs_REVERSED
from OCP.BRepAdaptor import BRepAdaptor_Surface,BRepAdaptor_Curve
from OCP.GeomAbs import GeomAbs_Plane,GeomAbs_Circle,GeomAbs_Line
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

ROOT=Path('F:/工创比赛/print_parts/radar_plate_search')
OUT=Path('F:/工创比赛/亚克力加工')
TOL=.005

def point(p):return np.array([p.X(),p.Y()])

def main():
    OUT.mkdir(exist_ok=True)
    shape=TopoDS_Shape();assert BRepTools.Read_s(shape,str(ROOT/'candidate_05.brep'),BRep_Builder())
    candidates=[];ex=TopExp_Explorer(shape,TopAbs_FACE)
    while ex.More():
        face=TopoDS.Face_s(ex.Current());surface=BRepAdaptor_Surface(face)
        if surface.GetType()==GeomAbs_Plane and abs(surface.Plane().Axis().Direction().Z())>.999999:
            props=GProp_GProps();BRepGProp.SurfaceProperties_s(face,props)
            candidates.append((props.Mass(),props.CentreOfMass().Z(),face))
        ex.Next()
    # Top and bottom are the two large matching extrusion faces.
    candidates.sort(key=lambda x:x[0],reverse=True)
    assert len(candidates)>=2 and abs(candidates[0][0]-candidates[1][0])<.01
    area,z,face=max(candidates[:2],key=lambda x:x[1])
    thickness=abs(candidates[0][1]-candidates[1][1]);assert abs(thickness-3)<.001
    bound=Bnd_Box();BRepBndLib.AddOptimal_s(face,bound,False,False)
    b=bound.Get();origin=np.array([b[0],b[1]])
    outside=BRepTools.OuterWire_s(face)
    wires=[];ex=TopExp_Explorer(face,TopAbs_WIRE)
    while ex.More():
        wire=TopoDS.Wire_s(ex.Current());wires.append((wire.IsSame(outside),wire));ex.Next()
    doc=ezdxf.new('R2010');doc.units=ezdxf.units.MM
    doc.header['$MEASUREMENT']=1;doc.header['$INSUNITS']=4;doc.header['$LUNITS']=2;doc.header['$LUPREC']=4
    doc.layers.new('CUT_OUTER',dxfattribs={'color':7})
    doc.layers.new('CUT_INNER',dxfattribs={'color':1})
    msp=doc.modelspace();contours=[];hole_diameters=[];gap_max=0
    for is_outer,wire in wires:
        explorer=BRepTools_WireExplorer(wire,face);edges=[]
        while explorer.More():edges.append(explorer.Current());explorer.Next()
        layer='CUT_OUTER' if is_outer else 'CUT_INNER'
        curves=[BRepAdaptor_Curve(edge) for edge in edges]
        if not is_outer and all(curve.GetType()==GeomAbs_Circle for curve in curves):
            circles=[curve.Circle() for curve in curves]
            center=point(circles[0].Location())-origin;radius=circles[0].Radius()
            total=sum(abs(curve.LastParameter()-curve.FirstParameter()) for curve in curves)
            same=all(np.linalg.norm(point(circle.Location())-origin-center)<1e-5
                     and abs(circle.Radius()-radius)<1e-5 for circle in circles)
            if same and abs(total-2*math.pi)<1e-5:
                msp.add_circle(center,radius,dxfattribs={'layer':layer})
                hole_diameters.append(2*radius)
                n=max(24,int(math.ceil(2*math.pi/(2*math.acos(1-TOL/radius)))))
                angles=np.linspace(0,2*math.pi,n,endpoint=False)
                points=center+radius*np.column_stack((np.cos(angles),np.sin(angles)))
                contours.append({'outer':False,'sample':points.tolist(),'cad_edges':len(edges)})
                continue
        poly=[];sample=[];last_end=None;first_start=None
        for edge in edges:
            curve=BRepAdaptor_Curve(edge)
            lo,hi=curve.FirstParameter(),curve.LastParameter()
            reverse=edge.Orientation()==TopAbs_REVERSED
            start,end=(hi,lo) if reverse else (lo,hi)
            p0,p1=point(curve.Value(start))-origin,point(curve.Value(end))-origin
            if first_start is None:first_start=p0
            if last_end is not None:gap_max=max(gap_max,float(np.linalg.norm(p0-last_end)))
            last_end=p1
            kind=curve.GetType()
            if kind==GeomAbs_Circle:
                circle=curve.Circle();radius=circle.Radius();center=point(circle.Location())-origin
                sweep=(end-start)*(1 if circle.Axis().Direction().Z()>0 else -1)
                n=max(8,int(math.ceil(abs(end-start)/max(.0001,2*math.acos(max(-1,min(1,1-TOL/radius)))))))
                params=np.linspace(start,end,n+1)
                points=[point(curve.Value(t))-origin for t in params]
                if len(edges)==1 and abs(abs(sweep)-2*math.pi)<1e-5:
                    assert not is_outer
                    msp.add_circle(center,radius,dxfattribs={'layer':layer})
                    hole_diameters.append(2*radius)
                    sample=points
                    poly=None
                    break
                poly.append((float(p0[0]),float(p0[1]),math.tan(sweep/4)))
                sample.extend(points[:-1])
            elif kind==GeomAbs_Line:
                poly.append((float(p0[0]),float(p0[1]),0));sample.append(p0)
            else:
                discrete=GCPnts_QuasiUniformDeflection(curve,TOL,lo,hi)
                assert discrete.IsDone()
                points=[point(discrete.Value(i))-origin for i in range(1,discrete.NbPoints()+1)]
                if reverse:points.reverse()
                assert np.linalg.norm(points[0]-p0)<.001 and np.linalg.norm(points[-1]-p1)<.001
                poly.extend((float(p[0]),float(p[1]),0) for p in points[:-1])
                sample.extend(points[:-1])
        if poly is not None:
            gap_max=max(gap_max,float(np.linalg.norm(last_end-first_start)))
            assert gap_max<.001
            msp.add_lwpolyline(poly,format='xyb',close=True,dxfattribs={'layer':layer})
        pts=np.array(sample)
        contours.append({'outer':bool(is_outer),'sample':pts.tolist(),'cad_edges':len(edges)})
    assert len(contours)==18 and sum(c['outer'] for c in contours)==1
    outer=next(c['sample'] for c in contours if c['outer']);inners=[c['sample'] for c in contours if not c['outer']]
    polygon=Polygon(outer,inners)
    assert polygon.is_valid and abs(polygon.area-area)/area<.001
    path=OUT/'雷达下方亚克力板_GPS下板nano_3mm_1比1.dxf'
    doc.saveas(path)
    check=ezdxf.readfile(path);assert not check.audit().has_errors
    entities=list(check.modelspace());assert len(entities)==18
    assert sum(e.dxf.layer=='CUT_OUTER' for e in entities)==1
    assert all(e.dxftype() in ('LWPOLYLINE','CIRCLE') for e in entities)
    assert all(e.closed for e in entities if e.dxftype()=='LWPOLYLINE')
    bounds=dxf_bbox.extents(entities);width=bounds.extmax.x-bounds.extmin.x;height=bounds.extmax.y-bounds.extmin.y
    assert abs(width-(b[3]-b[0]))<.02 and abs(height-(b[4]-b[1]))<.02
    font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
    fig,ax=plt.subplots(figsize=(7,10),facecolor='white')
    outer=np.array(outer);ax.fill(outer[:,0],outer[:,1],color='#dbeff5',edgecolor='#172d40',linewidth=1.4)
    for inner in inners:
        q=np.array(inner);ax.fill(q[:,0],q[:,1],color='white',edgecolor='#172d40',linewidth=1)
    ax.set_aspect('equal');ax.set_xlim(-9,width+9);ax.set_ylim(-12,height+12);ax.axis('off')
    ax.annotate('',xy=(0,-5),xytext=(width,-5),arrowprops=dict(arrowstyle='<->',color='#467284'))
    ax.text(width/2,-8,f'{width:.2f} mm',ha='center',va='top',fontsize=12)
    ax.annotate('',xy=(width+5,0),xytext=(width+5,height),arrowprops=dict(arrowstyle='<->',color='#467284'))
    ax.text(width+7,height/2,f'{height:.2f} mm',rotation=90,ha='left',va='center',fontsize=12)
    ax.set_title('雷达下方亚克力板 · 切割轮廓预览\n原模型：GPS下板-nano.  |  板厚：3 mm',fontproperties=font,fontsize=15,pad=16)
    fig.text(.5,.03,'DXF 为 1:1 毫米轮廓，无文字或尺寸线；请按实物核对孔位与厚度。',ha='center',fontproperties=font,fontsize=10)
    fig.savefig(OUT/'雷达下方亚克力板_轮廓核对.png',dpi=170,bbox_inches='tight');plt.close(fig)
    info={'source':'380 飞机.STEP / GPS下板-nano.','photo_match':'Candidate selected by matching curved outline and mounting features; verify against physical part before machining',
          'unit':'mm','scale':'1:1','width_mm':width,'height_mm':height,'model_thickness_mm':thickness,
          'outer_contours':1,'inner_contours':17,'circular_hole_diameters_mm':sorted(hole_diameters),
          'curve_approximation_max_deflection_mm':TOL,'max_source_edge_gap_mm':gap_max,
          'cad_face_area_mm2':area,'sampled_outline_area_mm2':polygon.area,
          'dxf_audit':'PASS','all_contours_closed':True,
          'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    (OUT/'雷达下方亚克力板_导出检查.json').write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8')
    (OUT/'加工说明.txt').write_text('雷达下方透明异形亚克力板\n原STEP零件名：GPS下板-nano.\nDXF单位：毫米，比例1:1。原模型板厚约3mm，请按实物确认。\n外轮廓1条、内部切割轮廓17条（含圆孔与非圆开口）。CUT_OUTER为外形，CUT_INNER为内部轮廓。\nDXF中不含文字、标注线或第二层重复轮廓。圆孔保持真圆；圆弧保留，非圆样条按最大弦高0.005mm离散。\n未做激光刀缝补偿，切割方按机器和配合要求处理，避免重复补偿。\n请先对照照片、实物外形、孔距、板厚核对，再安排加工；文件尚未经过实物试装验证。\n',encoding='utf-8-sig')
    print(json.dumps(info,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
