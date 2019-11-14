from __future__ import print_function
import os
import torch
import numpy as np
import os
import random as rand
from mayavi import mlab
import math
import pyro
import pyro.contrib.gp as gp
import pyro.distributions as dist
from pyro.infer import SVI, Trace_ELBO
from pyro.optim import Adam

smoke_test = ('CI' in os.environ)  # ignore; used to check code integrity in the Pyro repo
pyro.enable_validation(True)       # can help with debugging
pyro.set_rng_seed(0)

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

foldername = '/home/umamaheswaran/Desktop/major_project/kitti_tracking/data_tracking_velodyne/training/velodyne/0020/'
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

print("DONE SEGMENT MAPPING !!")	

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


print ("DONE BIN MAPPING !!")

r = 0
new_2D_pt = [[[] for x in range(num_bins)] for y in range(num_segments)]
update_3d_new = []
new_3D_pt = [[[] for x in range(num_bins)] for y in range(num_segments)]
for seg in range(num_segments):
	for bin in range(num_bins):
		if(len(bin_mapper[seg][bin]) > 0):
			for pt in range(len(bin_mapper[seg][bin])):
				new_2D_pt[seg][bin].append([math.sqrt(bin_mapper[seg][bin][pt][0]**2 + bin_mapper[seg][bin][pt][1]**2),bin_mapper[seg][bin][pt][2]])
				new_3D_pt[seg][bin].append([bin_mapper[seg][bin][pt][0],bin_mapper[seg][bin][pt][1],bin_mapper[seg][bin][pt][2]])
				update_3d_new.append([bin_mapper[seg][bin][pt][0],bin_mapper[seg][bin][pt][1],bin_mapper[seg][bin][pt][2]])
				r = r+1

print ("DONE TRANSFORMING TO 2D")
#print(update_3d_new[3874][2] < -1.6)

m = 0
temp = 0
update_new_2D_pt_inputs = [[] for x in range(num_bins*num_segments)]
update_new_2D_pt_outputs = [[] for x in range(num_bins*num_segments)]

for seg in range(num_segments):
	for bin in range(num_bins):
		if(len(new_2D_pt[seg][bin]) != 0):
			temp = len(new_2D_pt[seg][bin])
			for d in range (temp):
				update_new_2D_pt_inputs[m].append(new_2D_pt[seg][bin][d][0])
				update_new_2D_pt_outputs[m].append(new_2D_pt[seg][bin][d][1])
				#print(seg,bin,d)
				m = m+1

			
#print(update_new_2D_pt_inputs[0:m])
#print(new_2D_pt[300][1400])
#print(m,r)

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
			#print(proto_pt)
print ("DONE FINDING PROTOTYPE POINTS")
#print(prototype_2D_pt[20])


k = 0
#prototype_2D_inputs    = [[[] for x in range(num_bins)] for y in range(num_segments)]
#prototype_2D_outputs   = [[[] for x in range(num_bins)] for y in range(num_segments)]


sigma_n = 0.3
t_model = 0.05
t_data = 0.5
plot_gnd = []
plot_finalgnd = []
plot_obstacle = []
q = 0
g = 0
f = 0
min_r = 0
max_r = 0
x = 0
ppp = 0
qqq = 0
for seg in range (num_segments):
	k = 0
	r = 0
	
	groundseed_inputs    = [[] for x in range(num_bins*num_segments)]
	groundseed_outputs   = [[] for x in range(num_bins*num_segments)]
	test_inputs    = [[] for x in range(num_bins*num_segments)]
	test_outputs   = [[] for x in range(num_bins*num_segments)]
	groundseed_3d = []
	test_3d = []
	#print("Start seg")
	for bin in range (num_bins):

		if(prototype_2D_pt[seg][bin] != [] and prototype_2D_pt[seg][bin][0][1] < -1):
			groundseed_inputs[k].append(prototype_2D_pt[seg][bin][0][0])
			groundseed_outputs[k].append(prototype_2D_pt[seg][bin][0][1])
			groundseed_3d.append([bin_mapper[seg][bin][0][0],bin_mapper[seg][bin][0][1],bin_mapper[seg][bin][0][2]])
			k = k + 1
			
			
		elif(prototype_2D_pt[seg][bin] != [] and prototype_2D_pt[seg][bin][0][1] >= -1):
			test_inputs[r].append(prototype_2D_pt[seg][bin][0][0])
			test_outputs[r].append(prototype_2D_pt[seg][bin][0][1])
			test_3d.append([bin_mapper[seg][bin][0][0],bin_mapper[seg][bin][0][1],bin_mapper[seg][bin][0][2]])
			r = r + 1

	#print("inputs")
	#print(groundseed_inputs[0:k])
	#print("outputs")
	#print(groundseed_outputs[0:k])
	#print("3d")
	#print(test_3d[0:r])
	#print(k,r,test_outputs[0:r],groundseed_outputs[0:k])

	groundseed_inputs1  = np.asarray(groundseed_inputs[0:k])
	groundseed_inputs1 = torch.from_numpy(groundseed_inputs1)
	groundseed_reshaped_inputs = torch.reshape(groundseed_inputs1, (-1,)).float()

	groundseed_outputs1  = np.asarray(groundseed_outputs[0:k])
	groundseed_outputs1 = torch.from_numpy(groundseed_outputs1)
	groundseed_reshaped_outputs = torch.reshape(groundseed_outputs1, (-1,)).float()

	test_inputs1  = np.asarray(test_inputs[0:r])
	test_inputs1 = torch.from_numpy(test_inputs1)
	test_reshaped_inputs = torch.reshape(test_inputs1, (-1,)).float()
	
	#print("GPR done")
	#print(test_reshaped_inputs[0:r],k)
	
	if(r > 0 and k > 0):
		kernel = gp.kernels.RBF(input_dim=1, variance=torch.tensor(1.3298),lengthscale=torch.tensor(0.3))
		gpr = gp.models.GPRegression(groundseed_reshaped_inputs,groundseed_reshaped_outputs, kernel, noise=torch.tensor(1.))
		f_loc, f_cov = gpr(test_reshaped_inputs, full_cov=True , noiseless = True)
		f_loc = f_loc.data.numpy()
		f_cov = f_cov.data.numpy()
		#print(f_loc,f_cov[0][0])
		for i in range (r):
			f = f + 1
			Z = test_outputs[i]
			den = math.sqrt((sigma_n**2) + f_cov[i][i])
			#print((f_cov[i][i],(Z-f_loc[i])/den))
			if((f_cov[i][i] < t_model and (Z-f_loc[i])/den) < t_data ):
				plot_gnd.append(test_3d[i])
				q = q + 1
		#print(plot_gnd)

	for bin in range(num_bins):
		temp = len(new_2D_pt[seg][bin])
		for d in range (temp):
			min_r = new_2D_pt[seg][bin][0][0]
			max_r = new_2D_pt[seg][bin][0][0]
			if(new_2D_pt[seg][bin][d][0] < min_r):
				min_r = new_2D_pt[seg][bin][d][0]
			if(new_2D_pt[seg][bin][d][0] > max_r):
				max_r = new_2D_pt[seg][bin][d][0]
		#if(min_r == max_r):
		#	x = x + 1

		radial = (min_r + max_r)/2
		radial1 = np.asarray(radial)
		radial1 = torch.from_numpy(radial1)
		radial1 = torch.reshape(radial1, (-1,)).float()
		#print(radial1)
		f_loc, f_cov = gpr(radial1, full_cov=True , noiseless = True)
		f_loc = f_loc.data.numpy()
		temp = len(new_2D_pt[seg][bin])
		for d in range (temp):
			if(abs(new_2D_pt[seg][bin][d][1] - f_loc) < 0.3):
				plot_finalgnd.append(new_3D_pt[seg][bin][d])
				qqq = qqq + 1
			else:
				plot_obstacle.append(new_3D_pt[seg][bin][d])
				ppp = ppp + 1


	#print(min_r,max_r)



	print("Segment number",seg)
		
print("non ground--->",ppp,"ground---",qqq,"total--->",m)


"""
x_gnd = zip(*plot_gnd)[0]
y_gnd = zip(*plot_gnd)[1]
z_gnd = zip(*plot_gnd)[2]				


fig = mlab.figure(figure="Ground Points", size=(960,1080))
mlab.points3d(x_gnd, y_gnd, z_gnd,mode="point",colormap="spectral", scale_factor=100,figure = fig)
mlab.show()
"""
x_finalgnd = zip(*plot_finalgnd)[0]
y_finalgnd = zip(*plot_finalgnd)[1]
z_finalgnd = zip(*plot_finalgnd)[2]				


fig = mlab.figure(figure="Final Ground Points", size=(960,1080))
mlab.points3d(x_finalgnd, y_finalgnd, z_finalgnd,mode="point",colormap="spectral", scale_factor=100,figure = fig)
mlab.show()

x_obstacle = zip(*plot_obstacle)[0]
y_obstacle = zip(*plot_obstacle)[1]
z_obstacle = zip(*plot_obstacle)[2]				


fig = mlab.figure(figure="Obstacle Points", size=(960,1080))
mlab.points3d(x_obstacle, y_obstacle, z_obstacle,mode="point",colormap="spectral", scale_factor=100,figure = fig)
mlab.show()