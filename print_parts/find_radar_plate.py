"""Inspect other original STEP plate parts to match the user's physical photo."""
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
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepTools import BRepTools
from extract_candidates import tessellate

ROOT=Path(__file__).resolve().parent/'radar_plate_search'
ROOT.mkdir(exist_ok=True)
WANTED=['中间板板','零件1','零件1 - 副本','机体上仓','GPS下板-nano.','侧板板']
INSPECT=WANTED+['舱体板板','飞控PX4','mid360激光雷达.','PX4分电板']

def main():
    doc=TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
    reader=STEPCAFControl_Reader();reader.SetNameMode(True);reader.SetColorMode(False)
    print('Reading original assembly...',flush=True)
    assert reader.ReadFile('F:/DOWNLOAD/380 飞机.STEP')==IFSelect_RetDone
    assert reader.Transfer(doc)
    tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());labels=TDF_LabelSequence();tool.GetShapes(labels)
    found={};inventory=[]
    for i in range(1,labels.Length()+1):
        label=labels.Value(i);attr=TDataStd_Name()
        if not label.FindAttribute(TDataStd_Name.GetID_s(),attr):continue
        name=attr.Get().ToExtString()
        if name not in INSPECT:continue
        shape=tool.GetShape_s(label);box=Bnd_Box();BRepBndLib.AddOptimal_s(shape,box,False,False)
        coords=box.Get();dims=[coords[i+3]-coords[i] for i in range(3)]
        print(json.dumps({'name':name,'dims':dims,'bounds':coords,'assembly':tool.IsAssembly_s(label)},ensure_ascii=False),flush=True)
        found[name]=shape;inventory.append({'name':name,'dims':dims,'bounds':coords})
    records=[]
    for i,name in enumerate(WANTED,1):
        if name not in found:continue
        shape=found[name]
        key=f'candidate_{i:02d}'
        BRepTools.Write_s(shape,str(ROOT/f'{key}.brep'))
        v,f=tessellate(shape);center=(v.min(axis=0)+v.max(axis=0))/2
        np.savez_compressed(ROOT/f'{key}.npz',vertices=v-center,faces=f)
        record={'id':key,'name':name,'size_mm':np.ptp(v,axis=0).tolist(),'vertices':len(v),'triangles':len(f),'assembly':False}
        records.append(record)
    (ROOT/'candidates.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    (ROOT/'inspection.json').write_text(json.dumps(inventory,ensure_ascii=False,indent=2),encoding='utf-8')
    print('PLATES_EXTRACTED',flush=True)

if __name__=='__main__':main()
