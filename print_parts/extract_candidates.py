"""Extract named candidate geometry from the supplied STEP; never modify source."""
from pathlib import Path
import json
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

ROOT=Path(__file__).resolve().parent
SOURCE=Path('F:/DOWNLOAD/380 飞机.STEP')
WANTED=['新电池仓','新电池仓2.0','雷达保护罩装配','保护罩-大侧板','保护罩-小侧板-新','舱体板板_顶板']

def tessellate(shape):
    mesher=BRepMesh_IncrementalMesh(shape,.25,False,.3,True)
    mesher.Perform()
    vertices=[];faces=[]
    explorer=TopExp_Explorer(shape,TopAbs_FACE)
    while explorer.More():
        face=TopoDS.Face_s(explorer.Current());location=TopLoc_Location()
        tri=BRep_Tool.Triangulation_s(face,location)
        if tri is not None:
            offset=len(vertices)
            for i in range(1,tri.NbNodes()+1):
                p=tri.Node(i).Transformed(location.Transformation());vertices.append([p.X(),p.Y(),p.Z()])
            for i in range(1,tri.NbTriangles()+1):
                a,b,c=tri.Triangle(i).Get()
                if face.Orientation()==TopAbs_REVERSED:b,c=c,b
                faces.append([offset+a-1,offset+b-1,offset+c-1])
        explorer.Next()
    return np.array(vertices,dtype=float),np.array(faces,dtype=int)

def main():
    print('Reading STEP assembly...',flush=True)
    doc=TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
    reader=STEPCAFControl_Reader();reader.SetNameMode(True);reader.SetColorMode(False)
    assert reader.ReadFile(str(SOURCE))==IFSelect_RetDone
    print('Transferring CAD geometry...',flush=True)
    assert reader.Transfer(doc)
    tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    labels=TDF_LabelSequence();tool.GetShapes(labels)
    inventory=[];found={}
    for i in range(1,labels.Length()+1):
        label=labels.Value(i);attr=TDataStd_Name()
        if not label.FindAttribute(TDataStd_Name.GetID_s(),attr):continue
        name=attr.Get().ToExtString()
        inventory.append(name)
        if name in WANTED:found[name]=label
    (ROOT/'assembly_names.json').write_text(json.dumps(inventory,ensure_ascii=False,indent=2),encoding='utf-8')
    print('Named shapes',len(inventory),'Candidates',list(found),flush=True)
    records=[]
    for i,name in enumerate(WANTED,1):
        if name not in found:continue
        shape=tool.GetShape_s(found[name])
        box=Bnd_Box();BRepBndLib.Add_s(shape,box)
        xmin,ymin,zmin,xmax,ymax,zmax=box.Get()
        vertices,faces=tessellate(shape)
        assert len(vertices)>0 and len(faces)>0
        center=(vertices.min(axis=0)+vertices.max(axis=0))/2
        centered=vertices-center
        key=f'candidate_{i:02d}'
        np.savez_compressed(ROOT/f'{key}.npz',vertices=centered,faces=faces)
        record={'id':key,'name':name,'size_mm':[xmax-xmin,ymax-ymin,zmax-zmin],
                'vertices':len(vertices),'triangles':len(faces),'assembly':bool(tool.IsAssembly_s(found[name]))}
        records.append(record);print(json.dumps(record,ensure_ascii=False),flush=True)
    assert len(records)==len(WANTED),(len(records),WANTED)
    (ROOT/'candidates.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    print('EXTRACTION_COMPLETE',flush=True)

if __name__=='__main__':main()
