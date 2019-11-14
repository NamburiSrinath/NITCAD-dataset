import numpy as np
import os
from mayavi import mlab
import random

foldername = '/home/umamaheswaran/Desktop/major_project/kitti_tracking/data_tracking_velodyne/training/velodyne/0000/'
frameId = 20

filename = os.path.join(foldername, '%06d.bin' % frameId)
data = np.fromfile(filename, np.float32)
data = np.array(data.reshape(data.shape[0] // 4, 4))
# print data

X =data[:,0]
Y =data[:,1]
Z =data[:,2]



#Uncomment below lines for printing the lidar data

# for i in range(len(X)):
# 	print Y[i]
# 	print str(data[i][0])+"  "+str(data[i][1])+"  "+str(data[i][2])+"  "+str(data[i][3]) + ';'

# x = np.array([0,2,2,0,0])
# y = np.array([0,0,2,2,0])
# z = np.array([0,0,0,0,0])

fig = mlab.figure(figure="LidarData", size=(1920,1080))

mlab.points3d(X, Y, Z,mode="point",colormap="spectral", scale_factor=100,figure = fig)
# mlab.plot3d(x,y,z,color = (0,1,0),line_width=1, colormap='Spectral', figure=fig)
mlab.show()