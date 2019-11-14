import numpy as np
import os
from mayavi import mlab
import random

foldername = '/home/umamaheswaran/Desktop/major_project/kitti_tracking/data_tracking_velodyne/training/velodyne/0001/'
frameId = 200

filename = os.path.join(foldername, '%06d.bin' % frameId)
data = np.fromfile(filename, np.float32)
data = np.array(data.reshape(data.shape[0] // 4, 4))

X =data[:,0]
Y =data[:,1]
Z =data[:,2]

mlab.figure(figure="LidarData", size=(1920,1080))
plt = mlab.points3d(X, Y, Z,mode="point",colormap="spectral", scale_factor=100)
mlab.show()