# Speed estimation on KITTI Dataset

import cv2
import os 
import numpy as np
from matplotlib import pyplot as plt

#=========================================================================================

def find_count(passed_tracklets,idx):

	#...This finds the number of lablled object in an image
	count = 0
	for i in range(40):
	 	#print tracklets[1][i][2]
	 	#3rd argument is t3e labelled object name
	 	if passed_tracklets[idx][i][2] != 0:
	 		count = count + 1

	return count	 		

#=========================================================================================

def readCalibration(calib_dir,seq_idx,cam):
	fid = open(os.path.join(calib_dir,'%04d.txt'%seq_idx),"r")

	temp_file = open('temp.txt','w')
	lines = fid.readlines()
	for var in range(4):
		temp_file.write(lines[var] + '\n')  #go to next line after writing a line

	temp_file.close()
	temp = open('temp.txt','r')

	C = np.loadtxt(temp, delimiter = ' ',dtype={'names': ('col1', 'col2', 'col3', 'col4', 'col5', 'col6','col7', 'col8', 'col9', 'col10', 'col11', 'col12', 'col13'),
		'formats': ('S3', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float', 'float')})

	#print C

	#Since we are using only the second camera we only need the P2 matrix
	#typecasting first index of P to int is necesssary, else that is float and error happens..BEWARE..!!

	P = np.zeros((3,4))
	for i in range(12):
		P[int(np.floor(i / 4))][np.mod(i,4)] = C[cam][i+1]

	#delete the temporary file	

	os.remove('temp.txt')
	fid.close()
	return P

#===========================================================================================

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

	unzipped_version = zip(*C)	
	np_array = np.array(unzipped_version[0])


	# The number 40 is selected considering that number of labelled objects in an image would
	# above 40. TRADING SPACE FOR TIME...NOT REALLY TECHNICAL..!!!

	final_data  = [[[0 for x in range(num_cols)] for y in range(40)] for z in range(nimages)]
	# print array(final_data).shape

	# In the thrid loop (num_cols - 1) is used because num_cols is 18 , but as much as we are concerned
	# there are only 17 columns present and when we print also only 17 are getting printed...STRANGE.!! 

	for img in range(nimages):
		index, = np.where(np_array == img)    # COMMA IS PUT DELIBERATELY TO DIRECTLY GET INDICES
		for i in range(len(index)):
			annotated_obj = index[i]
			# print annotated_obj
			#Infact annotated_obj varies from 0 to 4270
		 	for j in range(num_cols - 1):
		 	 	final_data[img][i][j] = unzipped_version[j][annotated_obj]
			
	return final_data


#=========================================================================================================

def drawBox2D(passed_object, count_label):

	if passed_object[2] == 'Car':
		cv2.rectangle(image,(int(passed_object[6]),int(passed_object[7])), (int(passed_object[8]),int(passed_object[9])),(0,0,255),2)
	elif passed_object[2] == 'Van':
		cv2.rectangle(image,(int(passed_object[6]),int(passed_object[7])), (int(passed_object[8]),int(passed_object[9])),(255,0,0),2)	
	elif passed_object[2] == 'Truck':
		cv2.rectangle(image,(int(passed_object[6]),int(passed_object[7])), (int(passed_object[8]),int(passed_object[9])),(255,255,0),2)
	elif passed_object[2] == 'Pedestrian':
		cv2.rectangle(image,(int(passed_object[6]),int(passed_object[7])), (int(passed_object[8]),int(passed_object[9])),(0,255,0),2)	
	elif passed_object[2] == 'Person_sitting':
		cv2.rectangle(image,(int(passed_object[6]),int(passed_object[7])), (int(passed_object[8]),int(passed_object[9])),(0,255,255),2)
	elif passed_object[2] == 'Cyclist':
		cv2.rectangle(image,(int(passed_object[6]),int(passed_object[7])), (int(passed_object[8]),int(passed_object[9])),(255,0,255),2)	
	elif passed_object[2] == 'Tram':
		cv2.rectangle(image,(int(passed_object[6]),int(passed_object[7])), (int(passed_object[8]),int(passed_object[9])),(155,155,155),2)
	elif passed_object[2] == 'Misc':
		cv2.rectangle(image,(int(passed_object[6]),int(passed_object[7])), (int(passed_object[8]),int(passed_object[9])),(128,105,255),2)				

	#cv2.imshow('figure_test_display',image)
	return image


#==========================================================================================================

def projectToImage(passed_corners_3D, passed_P):
	np_passed = array(passed_corners_3D)
	num_cols_passed_corners_3D = np_passed[1].size

	pts = np.append(np_passed,[np.ones(num_cols_passed_corners_3D)],axis=0)
	pts_2D = np.matmul(passed_P,pts)

	#print 'size' + str(pts_2D[1].size)


	for i in range(3):
		pts_2D[0][i] = pts_2D[0][i] / pts_2D[2][i]
		pts_2D[1][i] = pts_2D[1][i] / pts_2D[2][i]

	np_pts_2D = array(pts_2D)
	ret_pts_2D = np.delete(np_pts_2D,(2),axis = 0)

	return ret_pts_2D

#============================================================================================
def find_featuresize(img1, img2, image, image_tp1, tracked, tracked_tp1):

	blank_image = np.zeros((image.shape[0],image.shape[1],3), np.uint8)
	blank_image_tp1 = np.zeros((image_tp1.shape[0],image_tp1.shape[1],3), np.uint8)

	blank_image[int(tracked[7]):int(tracked[9]),int(tracked[6]):int(tracked[8]),:] = img1
	blank_image_tp1[int(tracked_tp1[7]):int(tracked_tp1[9]),int(tracked_tp1[6]):int(tracked_tp1[8]),:] = img2

	# small_blank = cv2.resize(blank_image, (0,0), fx=0.5, fy=0.5)
	# small_blank_tp1 = cv2.resize(blank_image_tp1, (0,0), fx=0.5, fy=0.5)
	# small_image = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
	# small_image_tp1 = cv2.resize(image_tp1, (0,0), fx=0.5, fy=0.5)

	sift = cv2.xfeatures2d.SIFT_create()

	# find the keypoints and descriptors with SIFT
	kp1, des1 = sift.detectAndCompute(blank_image,None)
	kp2, des2 = sift.detectAndCompute(blank_image_tp1,None)

	# FLANN parameters
	FLANN_INDEX_KDTREE = 0
	index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
	search_params = dict(checks=50)   # or pass empty dictionary

	flann = cv2.FlannBasedMatcher(index_params,search_params)

	matches = flann.knnMatch(des1,des2,k=2)

	# Need to draw only good matches, so create a mask
	matchesMask = [[0,0] for i in xrange(len(matches))]

	# ratio test as per Lowe's paper
	for i,(m,n) in enumerate(matches):
		if m.distance < 0.7*n.distance:
			matchesMask[i]=[1,0]

	matchesMask = np.array(matchesMask)
	matchesMask_x = matchesMask[:,0]
	cnt_match = 0
	for i in range(len(matchesMask_x)):
		if matchesMask_x[i] == 1:
			cnt_match += 1

	draw_params = dict(matchColor = (0,255,0),
					singlePointColor = (255,0,0),
					matchesMask = matchesMask,
					flags = 0)

	img3 = cv2.drawMatchesKnn(image,kp1,image_tp1,kp2,matches,None,**draw_params)
	#cv2.imshow("test", img3)

	plt.imshow(img3,),plt.show()
	
	return cnt_match, img3

#============================================================================================

def func_disparity(frame_l,frame_r):
	
	stereo = cv2.StereoBM_create(numDisparities=32, blockSize=21)
	disparity = stereo.compute(frame_l,frame_r)

	min_ = disparity.min()
	max_ = disparity.max()

	disparity_scaled = np.uint8(255 * (disparity - min_) / (max_ - min_))
	dispcolormap = cv2.applyColorMap(disparity_scaled, cv2.COLORMAP_SUMMER)

	return disparity,dispcolormap	

#============================================================================================


print "======= KITTI Development Kit Demo ======="

root_dir        =   '/home/makka/kitti_test/data_tracking_image_2'
root_dir_right  =   '/home/makka/kitti_test/data_tracking_image_3'
data_set        =   'training'
train__dir      =   'image_02'

# set camera
cam = 2; # 2 = left color camera

full_path = os.path.join(root_dir,data_set,train__dir)
sub_dir = os.listdir(full_path)

num_sequences = len(sub_dir) - 2

seq_idx = 1
image_dir	  = os.path.join(root_dir,data_set,'image_02/0001')
right_imgdir  = os.path.join(root_dir_right,data_set,'image_03/0001')

label_dir 	  = os.path.join(root_dir,data_set,'label_02')
calib_dir 	  = os.path.join(root_dir,data_set, 'calib')

P = readCalibration(calib_dir,seq_idx,cam)

# print 'P is' + str(P)

nimages = len(os.listdir(image_dir))

tracklets = readLabels(label_dir, seq_idx,nimages)

THRESH_FEATURE_CNT = 30
FRAMES_PER_SEC     = 30

centroid_t   = [0,0]
centroid_tp1 = [0,0]

for img_idx in range(nimages-1):
	image = cv2.imread(os.path.join(image_dir,'%06d.png'%(img_idx)))
	image_tp1 = cv2.imread(os.path.join(image_dir,'%06d.png'%(img_idx+1)))

	image_right     = cv2.imread(os.path.join(right_imgdir,'%06d.png'%(img_idx)))
	image_right_tp1 = cv2.imread(os.path.join(right_imgdir,'%06d.png'%(img_idx+1)))	

	"""YOLO OBJECT DETECTION COMES HERE AND RETURNS THE TRACKELTS ACTUALLY"""

	count_t   = find_count(tracklets,img_idx)       #Num of detected objects @ time = t
	count_tp1 = find_count(tracklets,img_idx+1)	    #Num of detected objects @ time = t+1

	"""USE MC-CNN OR OPENCV FOR FINDING THE DISPARITY MAPS HERE"""

	# left_grey_t  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	# right_grey_t = cv2.cvtColor(image_right, cv2.COLOR_BGR2GRAY)

	# left_grey_tp1  = cv2.cvtColor(image_tp1, cv2.COLOR_BGR2GRAY)
	# right_grey_tp1 = cv2.cvtColor(image_right_tp1, cv2.COLOR_BGR2GRAY)

	#Find disparity map at time = t and time = t+1

	# disp_t, dispimage_t     = func_disparity(left_grey_t,right_grey_t)
	# disp_tm1, dispimage_tm1 = func_disparity(left_grey_tp1,right_grey_tp1)

	#cv2.imshow('DISPARITY', dispimage_t)


	for obj_idx_t in range(count_t):
		tracked = tracklets[img_idx][obj_idx_t]
		#print int(tracked[7]), int(tracked[9]), int(tracked[6]), int(tracked[8])
		cropped_t = image[int(tracked[7]):int(tracked[9]),int(tracked[6]):int(tracked[8]),:]

		if int(tracked[9])-int(tracked[7]) > 90 and int(tracked[8])-int(tracked[6]) > 90:	

			centroid_t = [(int(tracked[6])+int(tracked[8]))/2, (int(tracked[7])+int(tracked[9]))/2]	
			# print centroid_t

			"""Trying to find the probable objects in frame tp1  that corresponds to 
			   obj_idx_t"""

			currmax_matches  = 0

			for obj_idx_tp1 in range(count_tp1):

				tracked_tp1 = tracklets[img_idx+1][obj_idx_tp1]
				#print int(tracked_tp1[7]), int(tracked_tp1[9]), int(tracked_tp1[6]), int(tracked_tp1[8])
				cropped_tp1 = image_tp1[int(tracked_tp1[7]):int(tracked_tp1[9]),int(tracked_tp1[6]):int(tracked_tp1[8]),:]

				if int(tracked_tp1[9])-int(tracked_tp1[7]) > 90 and int(tracked_tp1[8])-int(tracked_tp1[6]) > 90:
					num_matches, imgx = find_featuresize(cropped_t,cropped_tp1, image, image_tp1, tracked, tracked_tp1)
					# print num_matches	

					"""Setting the threshold to be considered for finding maximum match at 30 features"""
					"""The centroid updates when num_matches is greater than a threshold as well the currmax_matches"""

					if num_matches > THRESH_FEATURE_CNT and num_matches > currmax_matches:

						centroid_tp1 = [(int(tracked_tp1[6])+int(tracked_tp1[8]))/2, (int(tracked_tp1[7])+int(tracked_tp1[9]))/2]
						currmax_matches = num_matches


			"""DISPARITY TO DEPTH COMES HERE D_t AND D_tp1"""
			"""D_tp1 MINUS D_t distance moved in 1/FPS sec => Velocity = FPS*(D_tp1 MINUS D_t)"""

			# print 'centroid_t = ',centroid_t, 'centroid_tp1 = ',centroid_tp1
			# disp = disp_t[int(centroid_t[1]),int(centroid_t[0])]
			# disp_tp1 = disp_tm1[int(centroid_tp1[1]),int(centroid_tp1[0])]
			# depth_t = 0.5*680/abs(disp)
			# depth_tp1 = 0.5*680/abs(disp_tp1)
			# inst_depth = depth_tp1-depth_t
			# velocity = inst_depth*45  
			# print velocity
		    #draw = drawBox2D(tracklets[img_idx][obj_idx_t], count_t)
			#cv2.putText(draw, str(velocity), ((int(tracked[6])+int(tracked[8]))/2, int(tracked[7])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
		    #cv2.imshow("test", draw)
	cv2.waitKey(50)


if(cv2.waitKey(0) & 0xFF == ord('q')):
	cv2.destroyAllWindows()

#=============================================================================================
