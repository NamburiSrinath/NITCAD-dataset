# KITTI TRACKING BENCHMARK DEMONSTRATION

# This tool displays the images and the object labels for the benchmark. 
# Before running this tool, set root_dir to the directory where you have
# downloaded the dataset. 'root_dir' must contain the subdirectory
# 'training', which in turn contains 'image_02', 'label_02' and 'calib'.


#  Usage:
#    SPACE: next frame
#    '-':   last frame
#    'x':   +50 frames
#    'y':   -50 frames
#    'c':   previous sequence
#    'v':   next sequence
#     q:     quit


import numpy as np
from numpy import array 
import cv2 
import os


#=============================================================================================
	#...This finds the number of lablled object in an image
	#...here we are considering 1st image

	#...............FORMAT OF TRACKLETS.......................#
	#.....tracklets[A][B][C]
	#.....A is the image number
	#.....B is infact for looping through all the 40 (assumed upper limit for labels)
	#.....C is for accesing the field :'type'   

def find_count(passed_tracklets,idx):
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
	
	#Uncomment the below two lines to print the file contents on terminal 

	#contents = fid.read()
	#print contents

	#...........................................................................#
	
	#    We need to access only the first 4 lines in calib files. So create a file temp.txt 
	#    to store the first 4 lines. readlines() read the file line by line and store each 
	#    line as an element of a list.As we need just the first 4 line we run the loop four 
	#    time and write each line into the temporary text file.Now close the temporary file 
	#    and open it again but in the read mode .

	temp_file = open('temp.txt','w')
	lines = fid.readlines()
	for var in range(4):
		temp_file.write(lines[var] + '\n')  #go to next line after writing a line

	temp_file.close()
	temp = open('temp.txt','r')

	#for printing contents of temp file (first 4 lines)

	#temp_contents = temp.read()
	#print temp_contents	

	#............................................................................#

	#For accessing the file as 2D matrix.loadtxt() stores the .txt file word by word 
	#into the 2D matrix


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

	# for future references for finding nimages without passing as param
	#print int(max(zip(*C)[0]))+1

	unzipped_version = zip(*C)
	#print unzipped_version

	#array_version = array(unzipped_version)
	#print array_version.shape
	#shape of array_version is (16,4271)
	
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


#============================================================================================

# def visualization(mode, image_dir, *args):
# 	if mode == 'init':
# 		print 'Now the argument is init'
# 		cv2.namedWindow('Figure 1')
# 		try:
# 			img = cv2.imread(os.path.join(image_dir,'000000.png'))				
# 			cv2.imshow('Figure 1',img)	
# 			cv2.waitKey(0)

# 		except:
# 			print 'No image file found for initialization. Exiting..!!'
# 			return

		
		
# 	elif mode == 'update':
# 		print 'Now the argument is update'

		

#============================================================================================

def drawBox2D(passed_object,count_label,img_idx,results_dir):
	# print 'top-left ='+str(passed_object[6])+' '+str(passed_object[7])+\
	# '    ''bottom right = '+str(passed_object[8])+' '+str(passed_object[9])
	if passed_object[2] == 'DontCare':
		cv2.rectangle(image,(int(passed_object[6]),int(passed_object[7])), (int(passed_object[8]),int(passed_object[9])),(103,192,255),2)
	elif passed_object[2] == 'Car':
		cv2.rectangle(image,(int(passed_object[6]),int(passed_object[7])), (int(passed_object[8]),int(passed_object[9])),(0,0,255),3)
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

	cv2.imshow('figure_test_display',image)
	#cv2.imwrite(os.path.join(results_dir,'%06d.png'%img_idx),image)

#============================================================================================

def computeBox3D(passed_tracklets,P):
	face_idx = [[1,2,6,5],[2,3,7,6],[3,4,8,7],[4,1,5,8]]
	R = [[+np.cos(passed_tracklets[16]), 0, +np.sin(passed_tracklets[16])],[0,1,0],[-np.sin(passed_tracklets[16]), 0, +np.cos(passed_tracklets[16])]]

	l = passed_tracklets[12]
	w = passed_tracklets[11]
	h = passed_tracklets[10]

	corners = [[l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2],
				[0,0,0,0,-h,-h,-h,-h],
				[w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]]

	corners_3D = np.matmul(R,corners)

	corners_3D[0] = corners_3D[0] +  passed_tracklets[13]
	corners_3D[1] = corners_3D[1] +  passed_tracklets[14]
	corners_3D[2] = corners_3D[2] +  passed_tracklets[15]

	for i in range(3):
		if corners_3D[2][i] < 0.1:
			return None,None

	# print corners_3D[2]
	corners_2D = projectToImage(corners_3D, P)	
	#print corners_2D

	return corners_2D,face_idx


#============================================================================================	

def projectToImage(passed_corners_3D, passed_P):
	np_passed = array(passed_corners_3D)
	num_cols_passed_corners_3D = np_passed[1].size

	pts = np.append(np_passed,[np.ones(num_cols_passed_corners_3D)],axis=0)
	pts_2D = np.matmul(passed_P,pts)

	#print 'size' + str(pts_2D[1].size)


	for i in range(num_cols_passed_corners_3D):
		pts_2D[0][i] = pts_2D[0][i] / pts_2D[2][i]
		pts_2D[1][i] = pts_2D[1][i] / pts_2D[2][i]

	np_pts_2D = array(pts_2D)
	ret_pts_2D = np.delete(np_pts_2D,(2),axis = 0)

	#print ret_pts_2D
	#ret_pts_2D = array(ret_pts_2D)
	#np.ceil(ret_pts_2D)
	return ret_pts_2D
	#return 0
#============================================================================================	
def computeOrientation3D(passed_tracklets,passed_P):
	R = [[+np.cos(passed_tracklets[16]), 0, +np.sin(passed_tracklets[16])],[0,1,0],[-np.sin(passed_tracklets[16]), 0, +np.cos(passed_tracklets[16])]]
	orientation_3D =  [[0.0,passed_tracklets[12]],[0.0,0.0],[0.0,0.0]]

	orientation_3D = np.matmul(R,orientation_3D)

	orientation_3D[0] = orientation_3D[0] + passed_tracklets[13]
	orientation_3D[1] = orientation_3D[1] + passed_tracklets[14]
	orientation_3D[2] = orientation_3D[2] + passed_tracklets[15]

	for i in range(2):
		if orientation_3D[2][i] < 0.1:
			return None

	orientation_2D = projectToImage(orientation_3D, P)	
	#print orientation_2D	
	return orientation_2D

#============================================================================================
def drawBox3D(passed_object,passed_corners,img_idx,results_dir):
	if passed_corners is not None:
		np_corners = np.array(passed_corners)
		np_corners = np.transpose(np_corners)
		y = np_corners.astype(int)
		#print y
		#print "============================="
	 	if passed_object[2] == 'DontCare':
			cv2.line(image,tuple(y[0]),tuple(y[1]),(103,192,255),2)#line(1 to 2)
			cv2.line(image,tuple(y[1]),tuple(y[2]),(103,192,255),2)#line(2 to 3)
			cv2.line(image,tuple(y[2]),tuple(y[3]),(103,192,255),2)#line(3 to 4)
			cv2.line(image,tuple(y[3]),tuple(y[0]),(103,192,255),2)#line(4 to 1)
			cv2.line(image,tuple(y[4]),tuple(y[5]),(103,192,255),2)#line(5 to 6)
			cv2.line(image,tuple(y[5]),tuple(y[6]),(103,192,255),2)#line(6 to 7)
			cv2.line(image,tuple(y[6]),tuple(y[7]),(103,192,255),2)#line(7 to 8)
			cv2.line(image,tuple(y[7]),tuple(y[4]),(103,192,255),2)#line(8 to 5)
			cv2.line(image,tuple(y[0]),tuple(y[4]),(103,192,255),2)#line(1 to 5)
			cv2.line(image,tuple(y[1]),tuple(y[5]),(103,192,255),2)#line(2 to 6)
			cv2.line(image,tuple(y[2]),tuple(y[6]),(103,192,255),2)#line(3 to 7)
			cv2.line(image,tuple(y[3]),tuple(y[7]),(103,192,255),2)#line(4 to 8)
		elif passed_object[2] == 'Car':
			cv2.line(image,tuple(y[0]),tuple(y[1]),(0,0,255),2)#line(1 to 2)
			cv2.line(image,tuple(y[1]),tuple(y[2]),(0,0,255),2)#line(2 to 3)
			cv2.line(image,tuple(y[2]),tuple(y[3]),(0,0,255),2)#line(3 to 4)
			cv2.line(image,tuple(y[3]),tuple(y[0]),(0,0,255),2)#line(4 to 1)
			cv2.line(image,tuple(y[4]),tuple(y[5]),(0,0,255),2)#line(5 to 6)
			cv2.line(image,tuple(y[5]),tuple(y[6]),(0,0,255),2)#line(6 to 7)
			cv2.line(image,tuple(y[6]),tuple(y[7]),(0,0,255),2)#line(7 to 8)
			cv2.line(image,tuple(y[7]),tuple(y[4]),(0,0,255),2)#line(8 to 5)
			cv2.line(image,tuple(y[0]),tuple(y[4]),(0,0,255),2)#line(1 to 5)
			cv2.line(image,tuple(y[1]),tuple(y[5]),(0,0,255),2)#line(2 to 6)
			cv2.line(image,tuple(y[2]),tuple(y[6]),(0,0,255),2)#line(3 to 7)
			cv2.line(image,tuple(y[3]),tuple(y[7]),(0,0,255),2)#line(4 to 8)
		elif passed_object[2] == 'Van':
			cv2.line(image,tuple(y[0]),tuple(y[1]),(255,0,0),2)#line(1 to 2)
			cv2.line(image,tuple(y[1]),tuple(y[2]),(255,0,0),2)#line(2 to 3)
			cv2.line(image,tuple(y[2]),tuple(y[3]),(255,0,0),2)#line(3 to 4)
			cv2.line(image,tuple(y[3]),tuple(y[0]),(255,0,0),2)#line(4 to 1)
			cv2.line(image,tuple(y[4]),tuple(y[5]),(255,0,0),2)#line(5 to 6)
			cv2.line(image,tuple(y[5]),tuple(y[6]),(255,0,0),2)#line(6 to 7)
			cv2.line(image,tuple(y[6]),tuple(y[7]),(255,0,0),2)#line(7 to 8)
			cv2.line(image,tuple(y[7]),tuple(y[4]),(255,0,0),2)#line(8 to 5)
			cv2.line(image,tuple(y[0]),tuple(y[4]),(255,0,0),2)#line(1 to 5)
			cv2.line(image,tuple(y[1]),tuple(y[5]),(255,0,0),2)#line(2 to 6)
			cv2.line(image,tuple(y[2]),tuple(y[6]),(255,0,0),2)#line(3 to 7)
			cv2.line(image,tuple(y[3]),tuple(y[7]),(255,0,0),2)#line(4 to 8)	
		elif passed_object[2] == 'Truck':
			cv2.line(image,tuple(y[0]),tuple(y[1]),(255,255,0),2)#line(1 to 2)
			cv2.line(image,tuple(y[1]),tuple(y[2]),(255,255,0),2)#line(2 to 3)
			cv2.line(image,tuple(y[2]),tuple(y[3]),(255,255,0),2)#line(3 to 4)
			cv2.line(image,tuple(y[3]),tuple(y[0]),(255,255,0),2)#line(4 to 1)
			cv2.line(image,tuple(y[4]),tuple(y[5]),(255,255,0),2)#line(5 to 6)
			cv2.line(image,tuple(y[5]),tuple(y[6]),(255,255,0),2)#line(6 to 7)
			cv2.line(image,tuple(y[6]),tuple(y[7]),(255,255,0),2)#line(7 to 8)
			cv2.line(image,tuple(y[7]),tuple(y[4]),(255,255,0),2)#line(8 to 5)
			cv2.line(image,tuple(y[0]),tuple(y[4]),(255,255,0),2)#line(1 to 5)
			cv2.line(image,tuple(y[1]),tuple(y[5]),(255,255,0),2)#line(2 to 6)
			cv2.line(image,tuple(y[2]),tuple(y[6]),(255,255,0),2)#line(3 to 7)
			cv2.line(image,tuple(y[3]),tuple(y[7]),(255,255,0),2)#line(4 to 8)
		elif passed_object[2] == 'Pedestrian':
			cv2.line(image,tuple(y[0]),tuple(y[1]),(0,255,0),2)#line(1 to 2)
			cv2.line(image,tuple(y[1]),tuple(y[2]),(0,255,0),2)#line(2 to 3)
			cv2.line(image,tuple(y[2]),tuple(y[3]),(0,255,0),2)#line(3 to 4)
			cv2.line(image,tuple(y[3]),tuple(y[0]),(0,255,0),2)#line(4 to 1)
			cv2.line(image,tuple(y[4]),tuple(y[5]),(0.255,0),2)#line(5 to 6)
			cv2.line(image,tuple(y[5]),tuple(y[6]),(0,255,0),2)#line(6 to 7)
			cv2.line(image,tuple(y[6]),tuple(y[7]),(0,255,0),2)#line(7 to 8)
			cv2.line(image,tuple(y[7]),tuple(y[4]),(0,255,0),2)#line(8 to 5)
			cv2.line(image,tuple(y[0]),tuple(y[4]),(0,255,0),2)#line(1 to 5)
			cv2.line(image,tuple(y[1]),tuple(y[5]),(0,255,0),2)#line(2 to 6)
			cv2.line(image,tuple(y[2]),tuple(y[6]),(0,255,0),2)#line(3 to 7)
			cv2.line(image,tuple(y[3]),tuple(y[7]),(0,255,0),2)#line(4 to 8)
		elif passed_object[2] == 'Person_sitting':
			cv2.line(image,tuple(y[0]),tuple(y[1]),(0,255,255),2)#line(1 to 2)
			cv2.line(image,tuple(y[1]),tuple(y[2]),(0,255,255),2)#line(2 to 3)
			cv2.line(image,tuple(y[2]),tuple(y[3]),(0,255,255),2)#line(3 to 4)
			cv2.line(image,tuple(y[3]),tuple(y[0]),(0,255,255),2)#line(4 to 1)
			cv2.line(image,tuple(y[4]),tuple(y[5]),(0,255,255),2)#line(5 to 6)
			cv2.line(image,tuple(y[5]),tuple(y[6]),(0,255,255),2)#line(6 to 7)
			cv2.line(image,tuple(y[6]),tuple(y[7]),(0,255,255),2)#line(7 to 8)
			cv2.line(image,tuple(y[7]),tuple(y[4]),(0,255,255),2)#line(8 to 5)
			cv2.line(image,tuple(y[0]),tuple(y[4]),(0,255,255),2)#line(1 to 5)
			cv2.line(image,tuple(y[1]),tuple(y[5]),(0,255,255),2)#line(2 to 6)
			cv2.line(image,tuple(y[2]),tuple(y[6]),(0,255,255),2)#line(3 to 7)
			cv2.line(image,tuple(y[3]),tuple(y[7]),(0,255,255),2)#line(4 to 8)			
		elif passed_object[2] == 'Cyclist':
			cv2.line(image,tuple(y[0]),tuple(y[1]),(255,0,255),2)#line(1 to 2)
			cv2.line(image,tuple(y[1]),tuple(y[2]),(255,0,255),2)#line(2 to 3)
			cv2.line(image,tuple(y[2]),tuple(y[3]),(255,0,255),2)#line(3 to 4)
			cv2.line(image,tuple(y[3]),tuple(y[0]),(255,0,255),2)#line(4 to 1)
			cv2.line(image,tuple(y[4]),tuple(y[5]),(255,0,255),2)#line(5 to 6)
			cv2.line(image,tuple(y[5]),tuple(y[6]),(255,0,255),2)#line(6 to 7)
			cv2.line(image,tuple(y[6]),tuple(y[7]),(255,0,255),2)#line(7 to 8)
			cv2.line(image,tuple(y[7]),tuple(y[4]),(255,0,255),2)#line(8 to 5)
			cv2.line(image,tuple(y[0]),tuple(y[4]),(255,0,255),2)#line(1 to 5)
			cv2.line(image,tuple(y[1]),tuple(y[5]),(255,0,255),2)#line(2 to 6)
			cv2.line(image,tuple(y[2]),tuple(y[6]),(255,0,255),2)#line(3 to 7)
			cv2.line(image,tuple(y[3]),tuple(y[7]),(255,0,255),2)#line(4 to 8)				
		elif passed_object[2] == 'Tram':
			cv2.line(image,tuple(y[0]),tuple(y[1]),(155,155,155),2)#line(1 to 2)
			cv2.line(image,tuple(y[1]),tuple(y[2]),(155,155,155),2)#line(2 to 3)
			cv2.line(image,tuple(y[2]),tuple(y[3]),(155,155,155),2)#line(3 to 4)
			cv2.line(image,tuple(y[3]),tuple(y[0]),(155,155,155),2)#line(4 to 1)
			cv2.line(image,tuple(y[4]),tuple(y[5]),(155,155,155),2)#line(5 to 6)
			cv2.line(image,tuple(y[5]),tuple(y[6]),(155,155,155),2)#line(6 to 7)
			cv2.line(image,tuple(y[6]),tuple(y[7]),(155,155,155),2)#line(7 to 8)
			cv2.line(image,tuple(y[7]),tuple(y[4]),(155,155,155),2)#line(8 to 5)
			cv2.line(image,tuple(y[0]),tuple(y[4]),(155,155,155),2)#line(1 to 5)
			cv2.line(image,tuple(y[1]),tuple(y[5]),(155,155,155),2)#line(2 to 6)
			cv2.line(image,tuple(y[2]),tuple(y[6]),(155,155,155),2)#line(3 to 7)
			cv2.line(image,tuple(y[3]),tuple(y[7]),(155,155,155),2)#line(4 to 8)
		elif passed_object[2] == 'Misc':
			cv2.line(image,tuple(y[0]),tuple(y[1]),(128,105,255),2)#line(1 to 2)
			cv2.line(image,tuple(y[1]),tuple(y[2]),(128,105,255),2)#line(2 to 3)
			cv2.line(image,tuple(y[2]),tuple(y[3]),(128,105,255),2)#line(3 to 4)
			cv2.line(image,tuple(y[3]),tuple(y[0]),(128,105,255),2)#line(4 to 1)
			cv2.line(image,tuple(y[4]),tuple(y[5]),(128,105,255),2)#line(5 to 6)
			cv2.line(image,tuple(y[5]),tuple(y[6]),(128,105,255),2)#line(6 to 7)
			cv2.line(image,tuple(y[6]),tuple(y[7]),(128,105,255),2)#line(7 to 8)
			cv2.line(image,tuple(y[7]),tuple(y[4]),(128,105,255),2)#line(8 to 5)
			cv2.line(image,tuple(y[0]),tuple(y[4]),(128,105,255),2)#line(1 to 5)
			cv2.line(image,tuple(y[1]),tuple(y[5]),(128,105,255),2)#line(2 to 6)
			cv2.line(image,tuple(y[2]),tuple(y[6]),(128,105,255),2)#line(3 to 7)
			cv2.line(image,tuple(y[3]),tuple(y[7]),(128,105,255),2)#line(4 to 8)
	cv2.imshow('figure_test_display',image)
	#cv2.imwrite(os.path.join(results_dir,'%06d.png'%img_idx),image)

	return 0
#============================================================================================
print "======= KITTI Development Kit Demo ======="

root_dir   = '/home/zac/Desktop/Major/data_tracking_image_2'
data_set   = 'training'
train__dir = 'image_02'

# set camera
cam = 2; # 2 = left color camera

full_path = os.path.join(root_dir,data_set,train__dir)
#print full_path

sub_dir = os.listdir(full_path)

#for files in sub_dir:
#	print files

num_sequences = len(sub_dir) - 2
#print 'Num of sequences '+ str(num_sequences)

seq_idx = 7
image_dir = os.path.join(root_dir,data_set,'image_02/0007')
label_dir = os.path.join(root_dir,data_set,'label_02')
calib_dir = os.path.join(root_dir,data_set, 'calib')
results_dir = '/home/zac/Desktop/Major/visualization/results'

P = readCalibration(calib_dir,seq_idx,cam)
print 'P is' + str(P)

nimages = len(os.listdir(image_dir))
#print "Total num of images = " + str(nimages)

tracklets = readLabels(label_dir, seq_idx,nimages)

# for accessing image details, last index is image num and second index is annotated obj num 

# for spec in range(17):
#	print tracklets[446][0]

#Set up figure
#h = visualization('init',image_dir)

cv2.namedWindow('figure_test_display')
#while(1):
for img_idx in range(nimages):
	count = find_count(tracklets,img_idx)
	#print count
	image = cv2.imread(os.path.join(image_dir,'%06d.png'%img_idx))
	cv2.imshow('figure_test_display',image)	
	for obj_idx in range(count):
		#for drawing 2D bounding boxes
		# drawBox2D(tracklets[img_idx][obj_idx],count,img_idx,results_dir)
		
		#for drawing 3D bounding boxes
		corners, face_idx = computeBox3D(tracklets[img_idx][obj_idx],P)
		#orientation = computeOrientation3D(tracklets[img_idx][obj_idx],P)
		drawBox3D(tracklets[img_idx][obj_idx],corners,img_idx,results_dir)

	cv2.waitKey(50)



# For accessing the whole dataset uncomment below lines

# for i in range(nimages):
# 	count = find_count(tracklets,i)
# 	for j in range(count):
# 		print tracklets[i][j]


if(cv2.waitKey(0) & 0xFF == ord('q')):
	cv2.destroyAllWindows()

#=============================================================================================
