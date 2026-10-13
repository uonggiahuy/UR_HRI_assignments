import numpy as np,json,math
from PIL import Image
from pathlib import Path
from scipy.ndimage import distance_transform_edt
m=np.flipud(np.array(Image.open('/root/ros2_ws/src/turtlebot4/maps/map.pgm')))
y,x=np.where(m==0);occupied=np.column_stack((-15+(x+.5)*.05,-25.1+(y+.5)*.05))
world=[];mapped=[];boxes=[]
# Independently inspected SDF: shelf_big poses -8.5,-1.5,6.5 at y=-13;
# link offset x=-0.5, collision box 2.1 x 18 m, zero yaw.
for center in (-9.,-2.,6.):
 expected=(center-1.05,center+1.05,-22.,-4.)
 region=occupied[(occupied[:,0]>expected[0]-.35)&(occupied[:,0]<expected[1]+.35)&(occupied[:,1]>-22.35)&(occupied[:,1]<-3.65)]
 observed=[region[:,0].min(),region[:,0].max(),region[:,1].min(),region[:,1].max()]
 boxes.append({'world':expected,'observed_map':observed})
 for i,j in ((0,2),(0,3),(1,2),(1,3)):
  world.append([expected[i],expected[j]]);mapped.append([observed[i],observed[j]])
w=np.array(world);v=np.array(mapped)
u,s,vt=np.linalg.svd((w-w.mean(0)).T@(v-v.mean(0)));rot=vt.T@u.T
assert np.linalg.det(rot)>0
trans=v.mean(0)-rot@w.mean(0);pred=w@rot.T+trans;res=np.linalg.norm(pred-v,axis=1)
scan=np.load('/tmp/m4_2/scan_map.npz');pts=(scan['points']+scan['truth'])@rot.T+trans
ix=np.floor((pts[:,0]+15)/.05).astype(int);iy=np.floor((pts[:,1]+25.1)/.05).astype(int)
d=distance_transform_edt(m!=0)[iy,ix]*.05
out={'method':'SDF shelf collision rectangle corners to independently observed occupancy rectangle bounds; rigid least-squares, held-out stationary scan', 'source_models':'shelf_big_0/1/2, model.sdf link x=-0.5, box 2.1x18; world y=-13', 'world_frame':'warehouse','map_frame':'map','world_points':world,'map_points':mapped,'boxes':boxes,'translation':trans.tolist(),'yaw':math.atan2(rot[1,0],rot[0,0]),'rms_residual_m':float(np.sqrt(np.mean(res**2))),'max_residual_m':float(res.max()),'heldout_scan_median_m':float(np.median(d)),'heldout_scan_p90_m':float(np.percentile(d,90)),'heldout_scan_fraction_within_0_15m':float(np.mean(d<=.15)), 'reference_uncertainty_m':.15,'scope':'local spawn and three mapped long shelf rectangles; full warehouse drift not established'}
out['verified']=out['max_residual_m']<.15 and out['heldout_scan_fraction_within_0_15m']>.95
Path('/tmp/m4_2/registration.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
