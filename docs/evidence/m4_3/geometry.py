"""Inspect actual sensor-visible COLLADA/STL warehouse geometry, no AMCL references."""
import xml.etree.ElementTree as E
from pathlib import Path
import numpy as np,json,math
ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
def dae_triangles(path):
 root=E.parse(path).getroot();geometries={}
 for g in root.findall('.//c:geometry',ns):
  mesh=g.find('c:mesh',ns);positions={s.get('id'):np.fromstring(s.find('c:float_array',ns).text,sep=' ').reshape(-1,3) for s in mesh.findall('c:source',ns) if s.find('c:technique_common/c:accessor',ns).get('stride')=='3'}
  vertices={v.get('id'):v.find("c:input[@semantic='POSITION']",ns).get('source')[1:] for v in mesh.findall('c:vertices',ns)}
  triangles=[]
  for t in mesh.findall('c:triangles',ns):
   inputs=t.findall('c:input',ns);stride=max(int(i.get('offset')) for i in inputs)+1
   vertex=next(i for i in inputs if i.get('semantic')=='VERTEX');indexes=np.fromstring(t.find('c:p',ns).text,sep=' ',dtype=int).reshape(-1,stride)[:,int(vertex.get('offset'))]
   triangles.append(positions[vertices[vertex.get('source')[1:]]][indexes].reshape(-1,3,3))
  geometries[g.get('id')]=np.concatenate(triangles)
 all_triangles=[]
 for node in root.findall('.//c:visual_scene/c:node',ns):
  transform=node.find('c:matrix',ns);matrix=np.fromstring(transform.text,sep=' ').reshape(4,4) if transform is not None else np.eye(4)
  for inst in node.findall('c:instance_geometry',ns):
   tris=geometries[inst.get('url')[1:]];pts=tris.reshape(-1,3)
   pts=np.column_stack((pts,np.ones(len(pts))))@matrix.T;all_triangles.append(pts[:,:3].reshape(-1,3,3))
 return np.concatenate(all_triangles)
def section(tris,z):
 segments=[]
 for t in tris:
  if t[:,2].min()>z or t[:,2].max()<z:continue
  hits=[]
  for a,b in ((t[0],t[1]),(t[1],t[2]),(t[2],t[0])):
   if (a[2]-z)*(b[2]-z)<0:
    hits.append((a+(z-a[2])/(b[2]-a[2])*(b-a))[:2])
  if len(hits)==2 and np.linalg.norm(hits[0]-hits[1])>1e-8:segments.append(hits)
 return np.array(segments)
root=Path('/root/.ignition/fuel/fuel.gazebosim.org/movai/models/shelf_big/2')
tris=dae_triangles(root/'meshes/shelf_big_movai.dae');segs=section(tris,.19)
# SDF visual yaw=pi/2, link x=-.5; world instances then applied.
rot=np.array([[0.,-1.],[1.,0.]])
segs=segs@rot.T+np.array([-.5,0.])
print('visible slice bbox',segs.reshape(-1,2).min(0),segs.reshape(-1,2).max(0))
np.savez_compressed('/tmp/m4_3/visible_shelves.npz',segments=segs)
