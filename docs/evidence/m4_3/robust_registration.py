"""Robust visible shelf-edge reference with spatially held-out shelves."""
import numpy as np,json,math
from pathlib import Path
from PIL import Image
from scipy.optimize import least_squares
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
m=np.flipud(np.array(Image.open('/root/ros2_ws/src/turtlebot4/maps/map.pgm')))
y,x=np.where(m==0);pts=np.column_stack((-15+(x+.5)*.05,-25.1+(y+.5)*.05))
segs=np.load('/tmp/m4_3/visible_shelves.npz')['segments'];lo=segs.reshape(-1,2).min(0);hi=segs.reshape(-1,2).max(0)
# Robust medians in per-metre slices, avoid extrema and mirrored ghost tails.
lines=[];world=[];mapped=[]
for sx in (-8.5,-1.5,6.5):
 xs=[sx+lo[0],sx+hi[0]];ends=[]
 for edge in xs:
  data=pts[(abs(pts[:,0]-edge)<.5)&(pts[:,1]>-21.5)&(pts[:,1]<-4.5)]
  bins=np.round(data[:,0]/.05)*.05;vals,counts=np.unique(bins,return_counts=True);peak=vals[np.argmax(counts)]
  data=data[abs(data[:,0]-peak)<.11]
  chunks=[]
  for yr in np.arange(-21.5,-4.5,1):
   strip=data[(data[:,1]>=yr)&(data[:,1]<yr+1)]
   if len(strip)>=4:chunks.append(np.median(strip,axis=0))
  chunks=np.array(chunks)
  fit=least_squares(lambda p:p[0]*chunks[:,1]+p[1]-chunks[:,0],[0,peak],loss='soft_l1',f_scale=.03)
  slope,intercept=fit.x;ends.append([slope,intercept]);lines.append({'world_x':edge,'slope':slope,'intercept':intercept,'n_strips':len(chunks),'residual_p95_m':float(np.percentile(abs(fit.fun),95)), 'peak_x':float(peak)})
 # Ends identified by cross-edge occupancy transverse histogram, excluding sidewalls.
 end_y=[]
 for expected in (-13+lo[1],-13+hi[1]):
  data=pts[(pts[:,0]>xs[0]+.15)&(pts[:,0]<xs[1]-.15)&(abs(pts[:,1]-expected)<.4)]
  vals,counts=np.unique(np.round(data[:,1]/.05)*.05,return_counts=True);peak=vals[np.argmax(counts)]
  end_y.append(float(np.median(data[abs(data[:,1]-peak)<.06,1])))
 for wi,(slope,intercept) in enumerate(ends):
  for yi in range(2):world.append([xs[wi],-13+(lo[1] if yi==0 else hi[1])]);mapped.append([slope*end_y[yi]+intercept,end_y[yi]])
w=np.array(world);v=np.array(mapped)
# Fit shelves 0 and 2; independently hold out central shelf 1.
train=np.array([0,1,2,3,8,9,10,11]);test=np.array([4,5,6,7])
u,s,vt=np.linalg.svd((w[train]-w[train].mean(0)).T@(v[train]-v[train].mean(0)));r=vt.T@u.T;t=v[train].mean(0)-r@w[train].mean(0);angle=math.atan2(r[1,0],r[0,0]);res=np.linalg.norm(w@r.T+t-v,axis=1)
out={'method':'sensor-visible COLLADA slice at z=0.19; dominant occupancy edge strip medians and robust linear fits; outer shelves train, central shelf held out','world_frame':'warehouse','map_frame':'map','translation':t.tolist(),'yaw':angle,'world_points':w.tolist(),'map_points':v.tolist(),'training_indices':train.tolist(),'heldout_indices':test.tolist(),'residuals_m':res.tolist(),'train_rms_m':float(np.sqrt(np.mean(res[train]**2))),'heldout_max_m':float(res[test].max()),'max_residual_m':float(res.max()),'edge_fits':lines,'reference_uncertainty_m':.15,'verified':False}
Path('/tmp/m4_3/registration_candidate.json').write_text(json.dumps(out,indent=2))
fig,ax=plt.subplots(figsize=(9,12));ax.imshow(np.flipud(m),cmap='gray',extent=(-15,15.15,-25.1,25),vmin=0,vmax=255);ax.scatter(*v.T,c='r',label='robust map features');pred=w@r.T+t;ax.scatter(*pred.T,c='b',marker='+',label='registered visible-mesh corners');ax.legend();ax.set_aspect('equal');fig.savefig('/tmp/m4_3/registration_features.png')
print(json.dumps({k:v for k,v in out.items() if k not in ('world_points','map_points','edge_fits')},indent=2))
