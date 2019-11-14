# Monocular visual odometry on NITCOD Dataset

import cv2
import os 
import numpy as np

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

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

	#Since we are using only the second camera we only need the P2 matrix
	#typecasting first index of P to int is necesssary, else that is float and error happens..BEWARE..!!

	P = np.zeros((3,4))
	for i in range(12):
		P[int(np.floor(i / 4))][np.mod(i,4)] = C[cam][i+1]

	os.remove('temp.txt')
	fid.close()
	return P


#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

def detect_features(frame):
	fast = cv2.FastFeatureDetector_create()
	fast.setThreshold(20)

	#Using 'fast' algorithm for feature detection
	kp = fast.detect(frame,None)

	#  Converting from Keypoint structure to Point structure
	pts = np.asarray([[[np.float32(p.pt[0]), np.float32(p.pt[1])]] for p in kp])
	return pts


#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

def track_features(frame1,frame2,p0):   
    lk_params = dict( winSize  = (21,21),
              maxLevel = 3,
              criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),flags=0,minEigThreshold=0.001)


    #  Tracking features in frame1 to frame2
    p1, status, err = cv2.calcOpticalFlowPyrLK(frame1, frame2, p0, None, **lk_params)


    for i in range(len(status)):
        if((p1[i][0][0]<0) or (p1[i][0][1]<0)):
            status[i][0] = 0

    # Removing points which failed to be tracked : trcaking failed => status == 0  

    good_p0 = p0[status == 1]
    good_p1 = p1[status == 1]

    return good_p0,good_p1

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

print "======= Monocular Visual Odometry on NITCOD Dataset ======="

seq_id = 2
root_dir        = '/home/umamaheswaran/major_project/datasets/nitcod_dataset/testing/'
root_calib        = '/home/umamaheswaran/major_project/datasets/kitti_tracking/data_tracking_image_2/training/'
calib_dir       =  os.path.join(root_calib, 'calib')
imgdir   = os.path.join(root_dir,'%02d'%seq_id)


P = readCalibration(calib_dir,seq_id,2) #left color cam
D = cv2.decomposeProjectionMatrix(P)
cam_mat = D[0]

focal      = cam_mat[0][0]
pp         = (cam_mat[0][2],cam_mat[1][2])

# focal = 721.5377  printcipal_pt = (609.5593, 172.854)  FOR SEQUENCE 0001 

nimages = len(os.listdir(imgdir))

color = np.random.randint(0,255,(50000,3))

img1_col  = cv2.imread(os.path.join(imgdir,'0000.png'))
img1      = cv2.cvtColor(img1_col, cv2.COLOR_BGR2GRAY)

img2_col  = cv2.imread(os.path.join(imgdir,'0001.png'))
img2      = cv2.cvtColor(img2_col, cv2.COLOR_BGR2GRAY)

# cv2.imshow('Display Output',img2)

p0 = detect_features(img1)
mask = np.zeros_like(img1_col)

good_old, good_new = track_features(img1, img2, p0)
E, mask = cv2.findEssentialMat(good_old,good_new, focal, pp, cv2.RANSAC, 0.999, 1.0)
points, R, t, mask = cv2.recoverPose(E,good_old, good_new, focal=focal, pp = pp)

# print E

previmg  = img2
prevfeatures = good_new

scale = 1.0

R_f = R
t_f = t

# traj = np.zeros((600,600,3), np.uint8)
# maskimg = np.zeros_like(img1_col)

traj = np.zeros((720,800,3), np.uint8)
maskimg = np.zeros_like(img1_col)

cv2.namedWindow('Output Window',cv2.WINDOW_NORMAL)
cv2.resizeWindow('Output Window', 2080,720)


for idx in range(2,nimages):

	# Triggering re-detection of features if number of features falls below 2000
	if (len(prevfeatures) < 2000 ):
		print 'Caution..Feature shortage!!!'
		prevfeatures = detect_features(previmg)
		maskimg = np.zeros_like(img1_col)


	currimg_color = cv2.imread(os.path.join(imgdir,'%04d.png'%idx))
	currimg       = cv2.cvtColor(currimg_color, cv2.COLOR_BGR2GRAY)
	# cv2.imshow('Input Video Sequence',currimg_color)

	#........................................................................

	prevfeatures,currfeatures = track_features(previmg, currimg, prevfeatures.reshape(-1,1,2))

	E, mask = cv2.findEssentialMat(prevfeatures,currfeatures, focal, pp, cv2.RANSAC, 0.999, 1.0)
	points, R, t, mask = cv2.recoverPose(E, prevfeatures, currfeatures, focal=focal, pp = pp)

	
	#  Estimstion rotation matrix and translation vetor
	t_f = t_f + scale*(np.matmul(R_f,t))
	R_f = np.matmul(R,R_f)

    # draw the tracks
	for i,(new,old) in enumerate(zip(currfeatures, prevfeatures)):
		a,b = new.ravel()
		c,d = old.ravel()
		maskimg = cv2.line(maskimg, (a,b),(c,d), color[i].tolist(), 2)
	    
	imgadd = cv2.add(currimg_color,maskimg)
	# cv2.imshow('Tracked Features',imgadd)

	previmg = currimg
	prevfeatures = currfeatures

	#........................................................................

	# Pixel co-ordinates for drawing the trajectory
	x = int(t_f[0][0]) + 400
	y = int(t_f[2][0]) + 500

	cv2.circle(traj, (x, y),1, (0,255,0), 2)
	# cv2.imshow('Trajectory',traj)

	small1 = cv2.resize(currimg_color, (0,0), fx=0.7, fy=0.5)
	small2 = cv2.resize(imgadd, (0,0), fx=0.7, fy=0.5) 

	imagec1 = np.concatenate((small1,small2), axis=0)
	imagec2 = np.concatenate((imagec1,traj), axis=1)

	# cv2.rectangle(imagec2, (1292, 10), (1792, 60), (0,0,0), cv2.FILLED)
	text = " -------TRAJECTORY--------"
	cv2.putText(imagec2, text, (1150,50), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,255), 1, 8)
	cv2.imshow('Output Window',imagec2)
	# cv2.imwrite("image_"+"%04d.jpg"%(idx-1),imagec2)
	cv2.waitKey(30)	


if(cv2.waitKey(0) & 0xFF == ord('q')):
	cv2.destroyAllWindows()	

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

