import sqlite3,pathlib,struct,cv2,numpy as np,argparse
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
parser=argparse.ArgumentParser(description='Reconstruct ECHO D8 retained RGB-D observations (RTAB-Map 0.23.14).')
parser.add_argument('database',type=pathlib.Path)
parser.add_argument('--output',type=pathlib.Path,required=True,help='New output directory; must not exist')
args=parser.parse_args()
db=args.database.resolve(strict=True); root=args.output.resolve()
root.mkdir(parents=True,exist_ok=False)
c=sqlite3.connect(db.as_uri()+'?mode=ro',uri=True)
assert c.execute('PRAGMA integrity_check').fetchone()[0]=='ok'
print('Node weights:',c.execute('SELECT weight,count(*) FROM Node GROUP BY weight').fetchall())
print('Stored scan rows:',c.execute('SELECT count(*) FROM Data WHERE length(scan)>0').fetchone()[0])
def models(b):
 assert len(b)==1488
 for j in range(4):
  a=b[j*372:(j+1)*372]; assert struct.unpack('<11i',a[:44])==(0,23,14,0,640,480,9,5,9,12,12)
  K=np.frombuffer(a[44:116],'<f8').reshape(3,3); P=np.frombuffer(a[228:324],'<f8').reshape(3,4); T=np.frombuffer(a[324:],'<f4').reshape(3,4)
  assert np.allclose(K,P[:,:3]); assert np.allclose(T[:,:3]@T[:,:3].T,np.eye(3),atol=1e-5)
  yield K,T
intr=[]; ext=[]
for (b,) in c.execute('select calibration from Data'):
 for K,T in models(b): intr.append([K[0,0],K[1,1],K[0,2],K[1,2]]);ext.append(T[:,3].tolist())
print('intrinsics range',np.min(intr,0),np.max(intr,0),'models',len(intr))
print('Retained node IDs:',[r[0] for r in c.execute('SELECT id FROM Node WHERE weight>=0 ORDER BY id')])
coords=[];colors=[];sides=[];counts=np.zeros(4,int); centers=[]
v,u=np.mgrid[0:480:4,0:640:4]
for id,p,img,dep,cal in c.execute('select Node.id,pose,image,depth,calibration from Node join Data using(id) where weight>=0 order by Node.id'):
 pose=np.frombuffer(p,'<f4').reshape(3,4); centers.append(pose[:,3]); rgb=cv2.imdecode(np.frombuffer(img,np.uint8),1)[:,:,::-1]; raw=cv2.imdecode(np.frombuffer(dep,np.uint8),-1); assert raw.shape==(480,2560,4) and rgb.shape==(480,2560,3); depth=np.ascontiguousarray(raw).view('<f4').reshape(480,2560)
 for j,(K,T) in enumerate(models(cal)):
  z=depth[::4,j*640:(j+1)*640:4]; good=np.isfinite(z)&(z>0)&(z<=10); z=z[good]; xyz=np.stack([(u[good]-K[0,2])*z/K[0,0],(v[good]-K[1,2])*z/K[1,1],z],1); xyz=xyz@T[:,:3].T+T[:,3]; xyz=xyz@pose[:,:3].T+pose[:,3]; col=rgb[::4,j*640:(j+1)*640:4][good]; coords.append(xyz);colors.append(col);sides.append(np.full(len(z),j,np.uint8)); counts[j]+=len(z)
xyz=np.concatenate(coords); rgb=np.concatenate(colors); side=np.concatenate(sides); centers=np.array(centers);print('points',len(xyz),counts,'bounds',xyz.min(0),xyz.max(0));print('pose span',np.linalg.norm(centers[-1,:2]-centers[0,:2]))
# PLY point samples in database odometry frame, not an optimized RTAB-Map export.
p=root/'echo_d8_rgbd_reconstruction.ply'; arr=np.empty(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')]);
for i,k in enumerate('xyz'):arr[k]=xyz[:,i]
for i,k in enumerate(['red','green','blue']):arr[k]=rgb[:,i]
with p.open('wb') as f:
 f.write(('ply\nformat binary_little_endian 1.0\ncomment ECHO D8 retained RGBD observations, odometry poses, stride 4, depth <= 10m\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'%len(arr)).encode());arr.tofile(f)
# Equal aspect orthographic projection, depth sorted; no geometric smoothing.
fig,axs=plt.subplots(1,2,figsize=(15,7),facecolor='#111827',layout='constrained')
for ax,mode in zip(axs,['top','iso']):
 ax.set_facecolor('#111827');
 if mode=='top':
  X,Y,Z=xyz.T; px,py=X,Y; order=np.argsort(Z)[::-1]; route=centers[:,:2]; title='Top view · looking down'
 else:
  az=np.deg2rad(40); el=np.deg2rad(29); right=np.array([np.cos(az),-np.sin(az),0]); up=np.array([-np.sin(az)*np.sin(el),-np.cos(az)*np.sin(el),np.cos(el)]); toward=np.cross(right,up); px=xyz@right;py=xyz@up; order=np.argsort(xyz@toward);route=np.column_stack([centers@right,centers@up]);title='Oblique view · RGB-D point cloud'
 ax.scatter(px[order],py[order],c=rgb[order]/255.,s=.75,linewidths=0,rasterized=True);ax.plot(route[:,0],route[:,1],'-o',color='#26e5ff',lw=1.5,ms=3,label='Retained robot poses');ax.set_aspect('equal');ax.set_title(title,color='white',fontsize=14);ax.tick_params(colors='#cbd5e1');ax.set_xlabel('m',color='#cbd5e1');ax.set_ylabel('m',color='#cbd5e1');ax.grid(alpha=.12);ax.legend(loc='lower right',facecolor='#111827',labelcolor='white',framealpha=.8)
fig.suptitle('ECHO S03-D8 | four-camera reconstruction from the saved database',color='white',fontsize=17);fig.savefig(root/'echo_d8_map_preview.png',dpi=150,facecolor=fig.get_facecolor());print('saved',p)
