import numpy as np
import os
from mayavi import mlab
import math
import sys
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

max_range = 30
num_bins  = 3000 # divide 30m into bins of 1cm

bin_size = float(max_range) / num_bins
# print bin_size

bin_mapper = [[[] for x in range(num_bins)] for y in range(num_segments)]
for seg in range(num_segments):
	for pt in range(len(segment_mapper[seg])):
		dist = math.sqrt(segment_mapper[seg][pt][0]**2 + segment_mapper[seg][pt][1]**2)
		bin_index = int(dist*100)
		if(bin_index < num_bins):
			bin_mapper[seg][bin_index].append([segment_mapper[seg][pt][0],segment_mapper[seg][pt][1],segment_mapper[seg][pt][2]])
			# print segment_mapper[seg][pt][0],segment_mapper[seg][pt][1],segment_mapper[seg][pt][2]


print "DONE BIN MAPPING !!"

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

"""Below segment is for groud point extraction"""

set_3Dgndpts = [[[] for x in range(num_bins)] for y in range(num_segments)]
slope_thresh = 0.577 # tan(30) = 0.577, tan(60) = 1.732
slope_min    = 0.087  # tan(5) = 0.087
threshz_intercept =  -1.4
slope = 0
intercept = 0
ptbegin = [0.0, 0.0]

for seg in range(num_segments):
	for bin in range(num_bins):
		if(len(prototype_2D_pt[seg][bin]) > 0):
			ptbegin = prototype_2D_pt[seg][bin][0]
			break
	for bin in range(num_bins):
		if(len(prototype_2D_pt[seg][bin]) > 0):
			slope = findslope(ptbegin, prototype_2D_pt[seg][bin][0])
			intercept = findintercept(ptbegin, prototype_2D_pt[seg][bin][0])
			if(((abs(slope) < slope_thresh) and (abs(slope) > slope_min)) or ((abs(slope) < slope_min) and (intercept < threshz_intercept))):
				set_3Dgndpts[seg][bin].append(prototype_3D_pt[seg][bin][0])
				ptbegin = prototype_2D_pt[seg][bin][0]

print "DONE EXTRACTING GROUND POINTS..!!"

plot_nongndpts = []
plot_mappednongndpts = [[[] for x in range(num_bins)] for y in range(num_segments)]
binary_graph = [[0 for x in range(num_bins)] for y in range(num_segments)]
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
								plot_mappednongndpts[seg][ranbin].append(appendpt)
								binary_graph[seg][ranbin] = 1

print "DONE REMOVING GROUND POINTS..!!"

"""Next part of the code is for segmentation based on connected components labelling"""

dx = [-1,0,1,1,1,0,-1,-1,-2,-1,0,1,2,-2,-2,-2,-2,-1,0,1,2,2,2,2,-3,-3,-3,-3,-3,-3,-3,3,3,3,3,3,3,3,-2,-1,0,1,2,-2,-1,0,1,2] 
dy = [1,1,1,0,-1,-1,-1,0,2,2,2,2,2,1,0,-1,-2,-2,-2,-2,-2,1,0,-1,3,2,1,0,-1,-2,-3,3,2,1,0,-1,-2,-3,3,3,3,3,3,-3,-3,-3,-3,-3]

# dx = [-1,0,1,1,1,0,-1,-1] 
# dy = [1,1,1,0,-1,-1,-1,0]

# dx = [-1,0,1,1,0,-2,-1,0,1,2,-2,-2,2,2,-3,-3,-3,-3,3,3,3,3,-2,-1,0,1] 
# dy = [1,1,1,0,0,2,2,2,2,2,1,0,1,0,3,2,1,0,2,1,0,3,3,3,3,3]

n_len = len(dx)
next_label = 1


label_graph = [[0 for x in range(num_bins)] for y in range(num_segments)]

merge_labels = []
min_neigh = 0
for seg in range(num_segments-3):
	for bin in range(num_bins-3):
		neighbour = []
		if(binary_graph[seg][bin] == 1):
			for i in range(n_len):
				nx = seg + dx[i]
				ny = bin + dy[i]

				if(binary_graph[nx][ny] != 0):
					neighbour.append(label_graph[nx][ny])

			if(len(neighbour) > 0):		
				min_neigh = min(neighbour)

			if(len((set(neighbour)|set([0])) -set([0])) > 1):
				merge_labels.append(list((set(neighbour) | set([0])) - set([0])))


			if(min_neigh == 0):
				label_graph[seg][bin] = next_label
				next_label = next_label + 1
			else:
				label_graph[seg][bin] = min_neigh

#print len(merge_labels)

set_relation = [set() for x in range(len(merge_labels))]
for i in range(len(merge_labels)):
	set_relation[i] = set(merge_labels[i])


for i in range(len(set_relation)):
	for j in range(i+1,len(set_relation)):
		if(set_relation[i]&set_relation[j] != set()):
			set_relation[i] |= set_relation[j]
			set_relation[j] = set()


mergeset = filter(lambda a: a != set(), set_relation)
mergelist = [[] for x in range(len(mergeset))]

for i in range(len(mergeset)):
	mergelist[i] = list(mergeset[i])

# print mergelist

minlist = [0 for x in range(len(mergeset))]
for i in range(len(mergeset)):
	minlist[i] = min(mergelist[i])

# print minlist

for seg in range(num_segments-1):
	for bin in range(num_bins-1):
		if(binary_graph[seg][bin] == 1):
			for i in range(len(minlist)):
				if(set([label_graph[seg][bin]]) & mergeset[i] != set()):
					label_graph[seg][bin] = minlist[i]



# print label_graph
pts_withlabel = []
pts_color = []
for seg in range(num_segments):
	for bin in range(num_bins):
		for pt in range(len(bin_mapper[seg][bin])):
			pts_withlabel.append(bin_mapper[seg][bin][pt])
			pts_color.append(1.0*((label_graph[seg][bin] + 254) % 255)/255)


print "Done labelling !!"

x_ = zip(*pts_withlabel)[0]
y_ = zip(*pts_withlabel)[1]
z_ = zip(*pts_withlabel)[2]		


fig = mlab.figure(figure="Segmented Frame", size=(960,1080))
nodes = mlab.points3d(x_, y_, z_,mode="point",colormap="spectral", scale_factor=100,figure = fig)
nodes.mlab_source.dataset.point_data.scalars = pts_color
mlab.show()
#*************************************************************************-