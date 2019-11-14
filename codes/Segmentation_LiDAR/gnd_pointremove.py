import numpy as np
import os
from mayavi import mlab
import math
#************************************************************************

def my_invtan(y,x):
	th = math.atan2(abs(y),abs(x))
	angle = 0
	if(y>0 and x>0):
		angle = 1.5*math.pi + (0.5*math.pi - th)
	elif(y>0 and x<0):
		angle = math.pi + th
	elif(y<0 and x>0):
		angle = th
	elif(y<0 and x<0):
		angle = 0.5*math.pi + (0.5*math.pi - th)

	return angle	

def findslope(pt_one, pt_two):
	if(pt_two[0] == pt_one[0]):
		return 500
	else:
		return (pt_two[1] - pt_one[1])/(pt_two[0] - pt_one[0])

def findintercept(pt_one, pt_two):
	if(pt_two[0] == pt_one[0]):
		return 500
	else:	
		return ((pt_two[0]*pt_one[1]) - (pt_one[0]*pt_two[1])) / (pt_two[0] - pt_one[0])

def eucleandist(pt_one, pt_two):
	return math.sqrt((pt_one[0] - pt_two[0])**2 + (pt_one[1] - pt_two[1])**2 + (pt_one[2] - pt_two[2])**2 )

#*************************************************************************

foldername = '/home/umamaheswaran/Desktop/major_project/kitti_tracking/data_tracking_velodyne/training/velodyne/0000/'
frameId = 20

filename = os.path.join(foldername, '%06d.bin' % frameId)
data = np.fromfile(filename, np.float32)
data = np.array(data.reshape(data.shape[0] // 4, 4))


X =data[:,0]
Y =data[:,1]
Z =data[:,2]


"""Uncomment below lines for printing the lidar data"""

# for i in range(len(X)):
# 	print str(data[i][0])+"  "+str(data[i][1])+"  "+str(data[i][2])+"  "+str(data[i][3]) + ';'

num_segments = 720
delta_alpha  = 2*math.pi / num_segments
# print math.degrees(delta_alpha) 

"""Below code segment maps each point to the segment to which it belongs"""

segment_mapper = [[] for x in range(num_segments)]
for i in range(len(X)):
	# print math.degrees(my_invtan(X[i],Y[i]))
	index = int(my_invtan(Y[i],X[i]) / delta_alpha)
	# print index
	segment_mapper[index].append([X[i],Y[i],Z[i]])

print "DONE SEGMENT MAPPING !!"	

"""Uncomment below segment to print points of a particular segment and seg no."""

# for i in range(len(segment_mapper[2])):
# 	print segment_mapper[2][i], int(my_invtan(segment_mapper[2][i][1],segment_mapper[2][i][0]) / delta_alpha)

max_range = 30
num_bins  = 3000 # divide 30m into bins of 1cm

bin_size = float(max_range) / num_bins
# print bin_size

bin_mapper = [[[] for x in range(num_bins)] for y in range(num_segments)]
for seg in range(num_segments):
	for pt in range(len(segment_mapper[seg])):
		# print segment_mapper[seg][pt]
		dist = math.sqrt(segment_mapper[seg][pt][0]**2 + segment_mapper[seg][pt][1]**2)
		bin_index = int(dist*100)
		# print bin_index
		if(bin_index < num_bins):
			# print bin_index
			bin_mapper[seg][bin_index].append([segment_mapper[seg][pt][0],segment_mapper[seg][pt][1],segment_mapper[seg][pt][2]])
			# print segment_mapper[seg][pt][0],segment_mapper[seg][pt][1],segment_mapper[seg][pt][2]


print "DONE BIN MAPPING !!"

# for i in range(3000):
# 	print len(bin_mapper[10][i])

new_2D_pt = [[[] for x in range(num_bins)] for y in range(num_segments)]
for seg in range(num_segments):
	for bin in range(num_bins):
		if(len(bin_mapper[seg][bin]) > 0):
			for pt in range(len(bin_mapper[seg][bin])):
				new_2D_pt[seg][bin].append([math.sqrt(bin_mapper[seg][bin][pt][0]**2 + bin_mapper[seg][bin][pt][1]**2),bin_mapper[seg][bin][pt][2]])


print "DONE TRANSFORMING TO 2D"
# print new_2D_pt

"""Finding the prototype point in non empty bins"""
prototype_2D_pt = [[[] for x in range(num_bins)] for y in range(num_segments)]
prototype_3D_pt = [[[] for x in range(num_bins)] for y in range(num_segments)]
for seg in range(num_segments):
	for bin in range(num_bins):
		if(len(bin_mapper[seg][bin]) > 0):
			proto_pt = new_2D_pt[seg][bin][0] 
			proto_3d = bin_mapper[seg][bin][0] 
			for pt in range(len(bin_mapper[seg][bin])):
				if(new_2D_pt[seg][bin][pt][1] < proto_pt[1]):
					proto_pt = new_2D_pt[seg][bin][pt]
					proto_3d = bin_mapper[seg][bin][pt]
			prototype_2D_pt[seg][bin].append(proto_pt)
			prototype_3D_pt[seg][bin].append(proto_3d)		

print "DONE FINDING PROTOTYPE POINTS"

# for i in range(num_bins):
# 	print i , prototype_3D_pt[1][i][:][:]

plot_points_proto = []
for seg in range(num_segments):
	for i in range(num_bins):
		if(len(prototype_3D_pt[seg][i]) > 0):
			plot_points_proto.append(prototype_3D_pt[seg][i])			

x_proto = zip(*zip(*plot_points_proto)[0])[0]
y_proto = zip(*zip(*plot_points_proto)[0])[1]
z_proto = zip(*zip(*plot_points_proto)[0])[2]

# print len(x_proto), len(plot_points_proto)
# print "len of proto pt = " + str(len(new_2D_pt[30])) 

"""Uncomment below print statement to see all the X coordinates"""
# print zip(*zip(*plot_points_proto)[0])[0]

fig = mlab.figure(figure="Prototype points", size=(960,1080))
mlab.points3d(x_proto, y_proto, z_proto,mode="point",colormap="spectral", scale_factor=100,figure = fig)

"""Below segment is for groud point extraction"""

set_gndpts = [[] for x in range(num_segments)]
set_3Dgndpts = [[[] for x in range(num_bins)] for y in range(num_segments)]
plot_points_gnd = []
slope_thresh = 1.732 # tan(30) = 0.577, tan(60) = 1.732,  tan(20) = 0.3639
slope_min    = 0.176  # tan(5) = 0.087 tan(10) = 0.176
threshz_intercept =  -1.4
slope = 0
intercept = 0
ptbegin = [0.0, 0.0]

for seg in range(num_segments):
	for bin in range(num_bins):
		if(len(prototype_2D_pt[seg][bin]) > 0):
			ptbegin = prototype_2D_pt[seg][bin][0]
			break

	# print ptbegin		
	for bin in range(num_bins):
		if(len(prototype_2D_pt[seg][bin]) > 0):
			slope = findslope(ptbegin, prototype_2D_pt[seg][bin][0])
			intercept = findintercept(ptbegin, prototype_2D_pt[seg][bin][0])
			# print slope, intercept
			if(((abs(slope) < slope_thresh) and (abs(slope) > slope_min)) or ((abs(slope) < slope_min) and (intercept < threshz_intercept))):
				set_gndpts[seg].append(prototype_2D_pt[seg][bin][0])
				set_3Dgndpts[seg][bin].append(prototype_3D_pt[seg][bin][0])
				plot_points_gnd.append(prototype_3D_pt[seg][bin][0])
				ptbegin = prototype_2D_pt[seg][bin][0]

x_gnd = zip(*plot_points_gnd)[0]
y_gnd = zip(*plot_points_gnd)[1]
z_gnd = zip(*plot_points_gnd)[2]	

print "DONE EXTRACTING GROUND POINTS..!!"

fig = mlab.figure(figure="Ground points", size=(960,1080))
mlab.points3d(x_gnd, y_gnd, z_gnd,mode="point",colormap="spectral", scale_factor=100,figure = fig)

plot_nongndpts = []
for seg in range(num_segments):
	for bin in range(num_bins):
		if(len(set_3Dgndpts[seg][bin]) > 0 and (bin>5 and bin < 2994)):
				for ranbin in range(bin-5,bin+5):
					for pts in range(len(bin_mapper[seg][ranbin])):
						if(len(bin_mapper[seg][ranbin]) > 0):
							dist = eucleandist(set_3Dgndpts[seg][bin][0],bin_mapper[seg][ranbin][pts])
							if(dist > 0.2):
								appendpt = bin_mapper[seg][ranbin][pts]	
								plot_nongndpts.append(appendpt)	

print "DONE REMOVING GROUND POINTS..!!"

x_nongnd = zip(*plot_nongndpts)[0]
y_nongnd = zip(*plot_nongndpts)[1]
z_nongnd = zip(*plot_nongndpts)[2]				

fig = mlab.figure(figure="Non Ground Points", size=(960,1080))
mlab.points3d(x_nongnd, y_nongnd, z_nongnd,mode="point",colormap="spectral", scale_factor=100,figure = fig)


"""Uncomment below lins for displaying point cloud"""

fig = mlab.figure(figure="LidarData", size=(960,1080))
mlab.points3d(X, Y, Z,mode="point",colormap="spectral", scale_factor=100,figure = fig)
mlab.show()

#*************************************************************************-