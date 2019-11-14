import numpy as np
import os
from mayavi import mlab

foldername = '/home/umamaheswaran/Desktop/major_project/kitti_tracking/data_tracking_velodyne/training/velodyne/0001/'
frameId = 0

filename = os.path.join(foldername, '%06d.bin' % frameId)
data = np.fromfile(filename, np.float32)
data = np.array(data.reshape(data.shape[0] // 4, 4))

x =data[:,0]
y =data[:,1]
z =data[:,2]

mlab.figure(figure="LidarData", size=(1920,1080),bgcolor=(0.0,0.0,0.0), fgcolor=(0.,1.0,0.))
plt = mlab.points3d(x, y, z,mode="point",colormap="copper", scale_factor=100)


#below '@' stuff is known as a 'decorator' in python
@mlab.animate(delay=100)
def anim():
	for i in range(447):
		print("file number = "+str(i))
		filename = os.path.join(foldername, '%06d.bin' % i)
		data = np.fromfile(filename, np.float32)
		data = np.array(data.reshape(data.shape[0] // 4, 4))

		x_ = data[:,0]
		y_ = data[:,1]
		z_ = data[:,2]
		
		#'reset' coz size of z,y,z change with file
		plt.mlab_source.reset(x=x_, y=y_, z=z_)
		yield

anim()
mlab.show()
