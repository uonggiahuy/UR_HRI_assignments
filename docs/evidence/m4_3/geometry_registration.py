"""Independent mesh-to-occupancy fit; reserve whole source shelf for validation."""
import numpy as np,json,math
from PIL import Image
from scipy.ndimage import distance_transform_edt,map_coordinates
from scipy.optimize import differential_evolution,minimize
from pathlib import Path
m=np.flipud(np.array(Image.open('/root/ros2_ws/src/turtlebot4/maps/map.pgm')));dist=distance_transform_edt(m!=0)*.05
base=np.load('/tmp/m4_3/visible_shelves.npz')['segments']
def sample(seg):
 p=[]
 for a,b in seg:
  n=max(2,int(np.linalg.norm(b-a)/.08)+1);p.extend(a+(b-a)*t for t in np.linspace(0,1,n))
 return np.array(p)
points=sample(base);world=[points+np.array([x,-13]) for x in (-8.5,-1.5,6.5)]
train=np.vstack((world[0],world[2]));test=world[1]
def residual(p,xy):
 tx,ty,angle=p;c,s=math.cos(angle),math.sin(angle);new=xy@np.array([[c,s],[-s,c]])+p[:2]
 return map_coordinates(dist,np.stack(((new[:,1]+25.1)/.05-.5,(new[:,0]+15)/.05-.5)),order=1,mode='constant',cval=2.)
def fun(p):
 r=residual(p,train)
 return np.mean(np.minimum(r,.30)**2)
r=differential_evolution(fun,[(-.4,.4),(-.3,.3),(-.02,.02)],seed=5,tol=1e-8,popsize=12,maxiter=80,polish=True)
tr=residual(r.x,train);te=residual(r.x,test)
out={'method':'COLLADA visible slice full mesh surface to occupancy distance, two outside shelves fit and central shelf held out', 'transform':r.x.tolist(),'training':{'points':len(tr),'median':float(np.median(tr)),'p90':float(np.percentile(tr,90)),'p95':float(np.percentile(tr,95)),'max':float(tr.max())},'heldout':{'points':len(te),'median':float(np.median(te)),'p90':float(np.percentile(te,90)),'p95':float(np.percentile(te,95)),'max':float(te.max())},'verified':False}
Path('/tmp/m4_3/mesh_registration.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
