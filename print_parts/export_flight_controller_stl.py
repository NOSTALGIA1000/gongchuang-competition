"""Export the exact named flight-controller housing as a fine, validated binary STL in mm."""
from pathlib import Path
import json
import hashlib
import sys
import numpy as np
import trimesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepTools import BRepTools
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from extract_candidates import tessellate

SOURCE=Path('F:/DOWNLOAD/380 飞机.STEP')
OUT=Path('F:/工创比赛/3D打印')

def main():
    OUT.mkdir(exist_ok=True)
    print('Reading original STEP...',flush=True)
    doc=TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
    reader=STEPCAFControl_Reader();reader.SetNameMode(True);reader.SetColorMode(False)
    assert reader.ReadFile(str(SOURCE))==IFSelect_RetDone
    assert reader.Transfer(doc)
    tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    labels=TDF_LabelSequence();tool.GetShapes(labels)
    selected=[]
    for i in range(1,labels.Length()+1):
        label=labels.Value(i);attr=TDataStd_Name()
        if label.FindAttribute(TDataStd_Name.GetID_s(),attr) and attr.Get().ToExtString()=='机体上仓':
            selected.append(tool.GetShape_s(label))
    assert len(selected)==1
    shape=selected[0]
    assert BRepCheck_Analyzer(shape).IsValid(),'Original CAD shape invalid'
    props=GProp_GProps();BRepGProp.VolumeProperties_s(shape,props)
    cad_volume=props.Mass()
    box=Bnd_Box();BRepBndLib.AddOptimal_s(shape,box,False,False)
    bounds=np.array(box.Get());cad_extents=bounds[3:]-bounds[:3]
    print('Meshing at 0.05 mm deviation...',flush=True)
    BRepTools.Clean_s(shape)
    fine=BRepMesh_IncrementalMesh(shape,.05,False,.15,True);fine.Perform()
    # Existing finer triangulation is retained by the shared extraction helper.
    vertices,faces=tessellate(shape)
    raw_faces=len(faces)
    mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=True)
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    assert mesh.is_watertight and mesh.is_winding_consistent and mesh.is_volume
    assert mesh.body_count==1
    assert np.allclose(mesh.extents,cad_extents,atol=.06)
    relative_volume_error=abs(mesh.volume-cad_volume)/cad_volume
    assert relative_volume_error<.005
    # Translation only: preserve original shape, orientation and millimeter scale.
    lo,hi=mesh.bounds
    mesh.apply_translation([-float((lo[0]+hi[0])/2),-float((lo[1]+hi[1])/2),-float(lo[2])])
    target=OUT/'飞控仓_机体上仓_mm.stl'
    target.write_bytes(mesh.export(file_type='stl'))
    check=trimesh.load_mesh(target,process=True)
    assert check.is_watertight and check.is_winding_consistent and check.is_volume
    assert check.body_count==1
    assert np.allclose(check.extents,[134,39.8,92],atol=.06)
    report={'part':'机体上仓','source':str(SOURCE),'file':str(target),
            'coordinate_unit':'mm; STL itself has no unit metadata',
            'scale':1,'rotation_applied':False,'translation':'XY centered; minimum Z=0',
            'dimensions_mm':check.extents.tolist(),'triangles':len(check.faces),
            'watertight':bool(check.is_watertight),'winding_consistent':bool(check.is_winding_consistent),
            'solid_bodies':check.body_count,'cad_valid':True,
            'cad_volume_mm3':cad_volume,'mesh_volume_mm3':check.volume,
            'relative_volume_error':relative_volume_error,
            'raw_triangles':raw_faces,'removed_invalid_or_duplicate_triangles':raw_faces-len(mesh.faces),
            'bytes':target.stat().st_size,'sha256':hashlib.sha256(target.read_bytes()).hexdigest()}
    (OUT/'机体上仓_导出检查.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (OUT/'机体上仓_打印说明.txt').write_text('机体上仓：从380 飞机.STEP中同名独立零件导出。\n导入切片软件按毫米（mm），100%缩放；外包尺寸约134 × 39.8 × 92 mm。\nSTL已验证闭合、面方向一致、单一实体；仅平移到台面，未修改设计尺寸或旋转。\n切片前核对飞控仓尺寸、安装孔位、打印机空间与耗材；网格检查通过不等于已验证配合、强度或飞行适用性。\n打印方向、支撑、壁数和填充需按实际设备与材料确定；先试装再上机。\n',encoding='utf-8-sig')
    print(json.dumps(report,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
