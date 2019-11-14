import numpy as np
import os
from mayavi import mlab

#=============================================================================================
def readCalib(calib_dir,seq_idx):
	fid = open(os.path.join(calib_dir,'%04d.txt'%seq_idx),"r")
	lines = fid.readlines()
	temp_line = lines[5]

	temp_file = open('temp.txt','w')
	temp_file.write(temp_line)
	temp_file.close()

	temp = open('temp.txt','r')

	#For accessing the file as 2D matrix.loadtxt() stores the .txt file word by word 
	#into the 2D matrix


	tr_velo_cam = np.loadtxt(temp, delimiter = ' ',dtype={'names': ('col1', 'col2', 'col3', 'col4', 'col5', 'col6','col7', 'col8', 'col9', 'col10', 'col11', 'col12', 'col13'),
		'formats': ('S11', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float')})
	# print tr_velo_cam['col5']

	rotation = np.zeros((3,3))
	translation = np.zeros((3,1))

	var = 2
	for i in range(3):
		for j in range(3):
			rotation[i][j] = tr_velo_cam['col'+str(var)]
			var = var + 1

	for i in range(3):
		translation[i] = tr_velo_cam['col'+str(var)]
		var = var + 1

	
	# print rotation
	# print translation

	return translation, rotation

		
#=============================================================================================
def readLabels(label_dir, seq_idx, nimages):
	labelfile = os.path.join(label_dir,'%04d.txt'%seq_idx)
	fid = open(labelfile,"r")

	lines = fid.readlines()
	line_0_stripped = lines[0].strip()

	# No idea why +2 is working. When we tried +1 the last coloumn of the file was'nt getting read..!!! 
	
	num_cols =  line_0_stripped.count(' ') + 2
	#print 'Num of cols' + str(num_cols)

	fid.close()

	fid = open(labelfile,"r")

	try:
		if num_cols == 17:
			C = np.loadtxt(fid, delimiter = ' ',dtype={'names': ('col1', 'col2', 'col3', 'col4', 'col5', 'col6','col7', 'col8', 'col9', 'col10', 'col11', 'col12', 'col13''col14', 'col15', 'col16', 'col17'),
				'formats': ('float', 'float', 'S10', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float')})


		elif num_cols == 18:
			C = np.loadtxt(fid, delimiter = ' ',dtype={'names': ('col1', 'col2', 'col3', 'col4', 'col5', 'col6','col7', 'col8', 'col9', 'col10', 'col11', 'col12', 'col13''col14', 'col15', 'col16', 'col17', 'col18'),
				'formats': ('float', 'float', 'S10', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float')})


		else:
			print 'Else : This file is not in KITTI tracking format.'

	except:
		print 'Except : This file is not in KITTI tracking format.'

	fid.close()

	# for future references for finding nimages without passing as param
	#print int(max(zip(*C)[0]))+1

	unzipped_version = zip(*C)	
	# print unzipped_version[2]
	h = unzipped_version[10]
	w = unzipped_version[11]
	l = unzipped_version[12]
	x = unzipped_version[13]
	y = unzipped_version[14]
	z = unzipped_version[15]
	
	theta = unzipped_version[16]
	# print theta[4]

	dim = np.array([h,w,l])
	unzip_dim = zip(*dim)	
	# print unzip_dim[4]

	coord = np.array([x,y,z])
	unzip_coord = zip(*coord)	
	# print unzip_coord[4]	

	return unzip_coord, unzip_dim, theta

#=============================================================================================
def find_camera_spatialcoord(obj_coord, obj_dim, obj_angle):
	# face_idx = [[1,2,6,5],[2,3,7,6],[3,4,8,7],[4,1,5,8]]

	# R = np.zeros((len(obj_angle),1))
	# for i in range(len(obj_angle)):
	# 	R[i] = [[+np.cos(theta[16]), 0, +np.sin(theta[16])],[0,1,0],[-np.sin(theta[16]), 0, +np.cos(theta[16])]]

	unzipdim =  zip(*obj_dim)
	l = unzipdim[0]
	w = unzipdim[1]
	h = unzipdim[2]

	unzipcoord =  zip(*obj_dim)
	x_ = unzipcoord[0]
	y_ = unzipcoord[1]
	z_ = unzipcoord[2]

	# print len(x)


	R  = [[[0 for x in range(3)] for y in range(3)]] 
	final_coord  = [[[0 for x in range(8)] for y in range(3)] for z in range(len(obj_angle))]
	final_cam_coord  = [[[0 for x in range(3)] for y in range(8)] for z in range(len(obj_angle))]
	corners_3D  = [[[0 for x in range(8)] for y in range(3)] for z in range(len(obj_angle))]
	
	for i in range(len(obj_angle)):
		theta = obj_angle[i]
		R = [[+np.cos(theta), 0, +np.sin(theta)],[0,1,0],[-np.sin(theta), 0, +np.cos(theta)]]

		corners = np.array([[l[i]/2, l[i]/2, -l[i]/2, -l[i]/2, l[i]/2, l[i]/2, -l[i]/2, -l[i]/2],
					[0,0,0,0,-h[i],-h[i],-h[i],-h[i]],
					[w[i]/2, -w[i]/2, -w[i]/2, w[i]/2, w[i]/2, -w[i]/2, -w[i]/2, w[i]/2]])

		corners_3D[i] = np.matmul(R,corners)

		final_coord[i][0] = corners_3D[i][0] + x_[i]
		final_coord[i][1] = corners_3D[i][1] + y_[i] 
		final_coord[i][2] = corners_3D[i][2] + z_[i]

		# print final_coord[i]
		final_cam_coord[i] = zip(*final_coord[i])
		

	# print final_cam_coord[100]
	return final_cam_coord


#=============================================================================================
def camera_spatial_to_lidar_points(camera_coord,translate,rotate,length):
	print "Rotation Matrix"
	print "----------------"
	print rotate


	# print translate[:][0]

	# THEORY
	# L = inv(R)[C - T]

	#finding (C - T)
	diff_coord  = [[[0 for x in range(3)] for y in range(8)] for z in range(length)]
	lidar_coord  = [[[0 for x in range(3)] for y in range(8)] for z in range(length)]
	

	# print translation
	# rot_inv = np.linalg.inv(rotate)
	#print np.matmul(rot_inv,rotate)

	# for i in range(length):
	# 	for j in range(8):
	# 		diff_coord[i][j] = camera_coord[i][j] - translation
	# 		# print diff_coord[i]
	# 		# lidar_coord[i][j] = np.matmul(rot_inv,diff_coord[i][j])
	# 	lidar_coord[i] = np.matmul(rot_inv,np.transpose(diff_coord[i]))	
	# 	print lidar_coord[i]

	# print np.array(lidar_coord[1])	
	# y = np.array(diff_coord[2])		
	# print y[1].size
	# print camera_coord[7][1] - translation		
	# print diff_coord

	# print "camera coord = " + str(np.transpose(camera_coord[99]))
	# print "T = " + str(translation)

	# diff = camera_coord[99] - np.array([translation,translation,translation,translation,translation,translation,translation,translation])
	# print "diff coord = " + str(diff)

	# print " inv mat = " + str(rot_inv)
	# lidar = np.matmul(rot_inv,diff)
	# print "lidar = " + str(lidar)

	#4x4 transformation matrix

	#########################################################################

	translation = np.zeros((3,1))
	translation = (np.transpose(translate))[0] 

	print "Translation Vector"
	print "------------------"	
	print translation

	inv_transf_mat = np.zeros((4,4))
	inv_rotmat = np.zeros((3,3))
	minus_rotinv_trans = np.zeros((3,1))

	inv_rotmat = np.linalg.inv(rotate)

	minus_rotinv_trans = -1*np.matmul(inv_rotmat,translation)

	print "Inverse Rotation"
	print "-----------------"
	print inv_rotmat

	print "Minus Inverse Rotation x Translation"
	print "-------------------------------"
	print minus_rotinv_trans

	for i in range(3):
		for j in range(3):
			inv_transf_mat[i][j] = inv_rotmat[i][j]
	for i in range(3):
		inv_transf_mat[i][3] = minus_rotinv_trans[i]
		inv_transf_mat[3][i] = 0
	inv_transf_mat[3][3] = 1

	print "Inverse Affine"
	print "-------------------------------"
	print inv_transf_mat
	
	# # inv_transfmat = np.linalg.inv(transf_mat)
	# # print trans_mat[1][2]

	ptlidar = np.array([0.0 for x in range(4)]) 
	ptcam = np.array([0.0 for x in range(4)]) 
	# # print ptcam

	for i in range(3):
		ptcam[i] = camera_coord[29][2][i]
		# print camera_coord[99][2][2]

	ptcam[3] = 1.0
	print "Input Point in Camera Frame"
	print ptcam

	ptlidar = np.matmul(inv_transf_mat,ptcam)
	print ptlidar

	return 0
#=============================================================================================

foldername = '/home/umamaheswaran/Desktop/major_project/kitti_tracking/data_tracking_velodyne/training/velodyne/0000/'
seq_idx = 0

frameId = 0
filename = os.path.join(foldername, '%06d.bin' % frameId)
data = np.fromfile(filename, np.float32)
data = np.array(data.reshape(data.shape[0] // 4, 4))

# print data

x =data[:,0]
y =data[:,1]
z =data[:,2]


root_dir   = '/home/umamaheswaran/Desktop/major_project/kitti_tracking/data_tracking_image_2'
data_set   = 'training'
train__dir = 'image_02'
label_dir = os.path.join(root_dir,data_set,'label_02')
calib_dir = os.path.join(root_dir,data_set, 'calib')

translate, rotate = readCalib(calib_dir,seq_idx)
# print rotate

nimages = 153
obj_coord, obj_dim, obj_angle = readLabels(label_dir, seq_idx, nimages)
# xbox, ybox, zbox = find_camera_spatialcoord(obj_coord, obj_dim, obj_angle)
# unzip =  zip(*obj_dim)
# print obj_angle[8]
camera_coord_3D = find_camera_spatialcoord(obj_coord, obj_dim, obj_angle)
print camera_coord_3D[98]
# lidar_bbox_pts = camera_spatial_to_lidar_points(camera_coord_3D,translate,rotate,len(obj_angle))
# print camera_coord_3D
#=============================================================================================

# mlab.figure(figure="LidarData", size=(1920,1080),bgcolor=(0.898,0.803,1.0), fgcolor=(0.,0.333,0.498))
# plt = mlab.points3d(x, y, z,mode="point",colormap="copper", scale_factor=100)


# #below '@' stuff is known as a 'decorator' in python
# @mlab.animate(delay=100)
# def anim():
# 	for i in range(153):
# 		print("file number = "+str(i))
# 		filename = os.path.join(foldername, '%06d.bin' % i)
# 		data = np.fromfile(filename, np.float32)
# 		data = np.array(data.reshape(data.shape[0] // 4, 4))

# 		x_ = data[:,0]
# 		y_ = data[:,1]
# 		z_ = data[:,2]
		
# 		#'reset' coz size of z,y,z change with file
# 		plt.mlab_source.reset(x=x_, y=y_, z=z_)
# 		yield

# anim()
# mlab.show()

#=============================================================================================
