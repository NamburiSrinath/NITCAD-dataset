# Speed estimation on KITTI Dataset

import cv2
import os 
import numpy as np

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

def drawBox2D(image, passed_object):
	image_copy = image.copy()
	for i in range(len(passed_object)):
		cv2.rectangle(image_copy,(int(passed_object[i][0]),int(passed_object[i][1])), (int(passed_object[i][2]),int(passed_object[i][3])),(0,0,255),2)
	return image_copy


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
def find_featuresize(img1, img2):

	sift = cv2.xfeatures2d.SIFT_create()
	kp1, des1 = sift.detectAndCompute(img1,None)
	kp2, des2 = sift.detectAndCompute(img2,None)

	# print 'pts1 = ' + str(len(kp1))
	# print 'pts2 = ' + str(len(kp2))

	# FLANN parameters
	FLANN_INDEX_KDTREE = 0
	index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
	search_params = dict(checks=50)   # or pass empty dictionary

	flann = cv2.FlannBasedMatcher(index_params,search_params)

	matches = flann.knnMatch(des1,des2,k=2)
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
	
	return cnt_match

#============================================================================================

def func_disparity(frame_l,frame_r):
	
	stereo = cv2.StereoBM_create(numDisparities=128, blockSize=15)
	disparity = stereo.compute(frame_l,frame_r)

	min_ = disparity.min()
	max_ = disparity.max()

	disparity_scaled = np.uint8(255 * (disparity - min_) / (max_ - min_))
	dispcolormap = cv2.applyColorMap(disparity_scaled, cv2.COLORMAP_SUMMER)

	return disparity,dispcolormap	

#============================================================================================

print "======= KITTI Development Kit Demo ======="

root_dir        =   '/home/umamaheswaran/major_project/datasets/kitti_tracking/data_tracking_image_2'
root_dir_right  =   '/home/umamaheswaran/major_project/datasets/kitti_tracking/data_tracking_image_3'
data_set        =   'training'
train__dir      =   'image_02'

# set camera
cam = 2; # 2 = left color camera

full_path = os.path.join(root_dir,data_set,train__dir)
sub_dir = os.listdir(full_path)

num_sequences = len(sub_dir) - 2

seq_idx = 3
image_dir	  = os.path.join(root_dir,data_set,'image_02/0003')
right_imgdir  = os.path.join(root_dir_right,data_set,'image_03/0003')

label_dir 	  = os.path.join(root_dir,data_set,'label_02')
calib_dir 	  = os.path.join(root_dir,data_set, 'calib')

P = readCalibration(calib_dir,seq_idx,cam)
# print 'P is' + str(P)

nimages = len(os.listdir(image_dir))

tracklets = readLabels(label_dir, seq_idx,nimages)

THRESH_FEATURE_CNT = 5
FRAMES_PER_SEC     = 30

centroid_t   = [0,0]
centroid_tp1 = [0,0]

#-------------------------------------------------------------------

image_lt    =	cv2.imread(os.path.join(image_dir,'000000.png'))
image_lr	=	cv2.imread(os.path.join(right_imgdir,'000000.png'))

image_ltp1	=	cv2.imread(os.path.join(image_dir,'000001.png'))
image_lrp1	=	cv2.imread(os.path.join(right_imgdir,'000001.png'))

detectedimage = drawBox2D()

#-------------------------------------------------------------------

'''image at instant t = 0'''

car1_lt = [776.295323, 167.346734, 1241.0, 374.0]
car2_lt = [716.495068, 179.216697, 856.320367, 270.111097]
car3_lt = [687.58362, 178.796339, 758.801387, 236.853238]
car4_lt = [386.049683, 192.243034, 463.188613, 244.957603]
car5_lt = [496.36026, 189.120921, 527.364453, 212.930471]
car6_lt = [637.23824, 179.19725, 665.906205, 202.155555]
car7_lt = [508.387993, 187.659658, 536.923249, 208.082142]

'''image at instant t = 1'''

car1_ltp1 = [ 798.330176, 170.197116, 1241.0, 374.0]
car2_ltp1 = [ 725.209266, 180.465334, 883.545546, 281.312638]
car3_ltp1 = [692.497122, 178.298959, 768.905263, 240.251872]
car4_ltp1 = [373.958976, 194.370949, 456.390299, 250.280839]
car5_ltp1 = [493.195888, 189.539251, 525.160431, 213.985418]
car6_ltp1 = [637.383502, 180.686978, 666.738985, 204.213968]
car7_ltp1 = [505.665889, 190.415353, 535.000602, 211.524857]

#-------------------------------------------------------------------


Q =	[[   1.0,     0.0  ,         0.0        ,-468.91947365],
 [   0.0    ,     1.0  ,         0.0        ,-699.2122196 ],
 [   0.0    ,     0.0  ,         0.0        , 680.05186262],
 [   0.0    ,     0.0  ,        -1.87703644 , 25.00423929]]

#-------------------------------------------------------------------

label0_lt = [car1_lt,car2_lt,car3_lt,car4_lt,car5_lt,car6_lt,car7_lt]
label0_ltp1 = [car1_ltp1,car2_ltp1,car3_ltp1,car4_ltp1,car5_ltp1,car6_ltp1,car7_ltp1]


visual_imglt = drawBox2D(image_lt, label0_lt)
visual_imgltp1 = drawBox2D(image_ltp1, label0_ltp1)

#-------------------------------------------------------------------

image_ltgrey      = cv2.cvtColor(image_lt, cv2.COLOR_BGR2GRAY)
image_lrgrey      = cv2.cvtColor(image_lr, cv2.COLOR_BGR2GRAY)

image_ltp1grey      = cv2.cvtColor(image_ltp1, cv2.COLOR_BGR2GRAY)
image_lrp1grey      = cv2.cvtColor(image_lrp1, cv2.COLOR_BGR2GRAY)

# disp_t, dispimage_t     = func_disparity(image_ltgrey,image_lrgrey)
# disp_tp1, dispimage_tp1 = func_disparity(image_ltp1grey,image_lrp1grey)

disp_t   = cv2.imread('d1_kitti.png')
disp_tp1 = cv2.imread('d2_kitti.png')

# depth_t = cv2.reprojectImageTo3D(disp_t,np.array(Q))
# depth_tp1 = cv2.reprojectImageTo3D(disp_tp1,np.array(Q))

cv2.imshow('disp image_t',disp_t)	
draw = cv2.imread('d1_kitti.png')

#-------------------------------------------------------------------

draw = drawBox2D(image_lt, label0_lt)
for idx_t in range(len(label0_lt)):
	cropped_t = image_lt[int(label0_lt[idx_t][1]):int(label0_lt[idx_t][3]),int(label0_lt[idx_t][0]):int(label0_lt[idx_t][2]),:]
	# cv2.imshow('cropped'+str(idx_t), cropped_t)

	if int(label0_lt[idx_t][3])-int(label0_lt[idx_t][1]) > 90 and int(label0_lt[idx_t][2])-int(label0_lt[idx_t][0]) > 90:
		#cv2.imshow('cropped t '+str(idx_t), cropped_t)
		centroid_t = [(int(label0_lt[idx_t][2])+int(label0_lt[idx_t][0]))/2,(int(label0_lt[idx_t][3])+int(label0_lt[idx_t][1]))/2]
		# print centroid_t
		# cv2.circle(visual_imglt,(centroid_t[0],centroid_t[1]), 10, (0,255,0), -1 )

		currmax_matches  = 0

		for idx_tp1 in range(len(label0_ltp1)):
			cropped_tp1 = image_ltp1[int(label0_ltp1[idx_tp1][1]):int(label0_ltp1[idx_tp1][3]),int(label0_ltp1[idx_tp1][0]):int(label0_ltp1[idx_tp1][2]),:]
			# cv2.imshow('cropped'+str(idx_tp1), cropped_tp1)

			if int(label0_ltp1[idx_tp1][3])-int(label0_ltp1[idx_tp1][1]) > 90 and int(label0_ltp1[idx_tp1][2])-int(label0_ltp1[idx_tp1][0]) > 90:
				#cv2.imshow('cropped tp1 '+str(idx_tp1), cropped_tp1)

				num_matches = find_featuresize(cropped_t,cropped_tp1)
				# print idx_t, idx_tp1, num_matches

				if num_matches > THRESH_FEATURE_CNT and num_matches > currmax_matches:

					centroid_tp1 = [(int(label0_ltp1[idx_tp1][2])+int(label0_ltp1[idx_tp1][0]))/2,(int(label0_ltp1[idx_tp1][3])+int(label0_ltp1[idx_tp1][1]))/2]
					currmax_matches = num_matches

		print 'centroid_t = ',centroid_t, 'centroid_tp1 = ',centroid_tp1
		# print 'depth_t of centroid = ',depth_t[centroid_t[1]][centroid_t[0]],'depth_tp1 of centroid',depth_tp1[centroid_tp1[1]][centroid_tp1[0]]
		print 'disp_t = ',disp_t[centroid_t[1]][centroid_t[0]][0],'disp_tp1 = ', disp_tp1[centroid_tp1[1]][centroid_tp1[0]][0]


		depth_t   = 0.5*680/(disp_t[centroid_t[1]][centroid_t[0]][0])
		depth_tp1 = 0.5*680/(disp_tp1[centroid_tp1[1]][centroid_tp1[0]][0])

		print depth_t,depth_tp1

		inst_depth = depth_tp1 - depth_t
		velocity = inst_depth*45*18/5

		print 'velocity = ',velocity 

		
		cv2.putText(draw, '%.2f'%abs(velocity)+' kmph', ((int(label0_lt[idx_t][0])+int(label0_lt[idx_t][2]))/2, int(label0_lt[idx_t][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
		cv2.imshow("test", draw)




#cv2.imshow('display t',visual_imglt)
#cv2.imshow('display tp1',visual_imgltp1)

if(cv2.waitKey(0) & 0xFF == ord('q')):
	cv2.destroyAllWindows()
