import sys 
import numpy as np
from open3d import *
import pcl
import math

#######################
#loading point cloud
#######################
cloud = pcl.PointCloud()
cloud =  pcl.load("pcd/projected_plane0.pcd")
boundary_points = np.zeros((cloud.size,3),dtype = np.float32)
count = 0
boundaries = pcl.PointCloud()
pcd = read_point_cloud("pcd/projected_plane0.pcd")
#draw_geometries([pcd])

for loc in range(0,cloud.size):

	#####################
	#Setting search point
	#####################
	searchPoint = pcl.PointCloud()
	searchPoints = np.zeros((1, 3), dtype=np.float32)
	searchPoints[0] = cloud[loc] 
	searchPoint.from_array(searchPoints)
	

	#####################
	#kdtree search	
	#####################
	pcd_tree = KDTreeFlann(pcd)
	[k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[loc], 30)
	nearest_neighbours = np.zeros((k,3),dtype=np.float32)
	nearest_neighbour = pcl.PointCloud()
	for i in range(0, k):
		nearest_neighbours[i]=cloud[idx[i]]
	nearest_neighbour.from_array(nearest_neighbours)
	n = np.array([0,0,1])				#Change  normal to the normal of your Plane

	########################
	#Calculating Angular gap
	########################
	print loc
	k_min = 500
	k_list = list()
	theta1 = list()
	k_theta = {}
	theta_min=3.14
	for K in range (0,k):
		if K != 1 and K!=0:		
			a = nearest_neighbours[1] - searchPoints[0]
			b = nearest_neighbours[K] - searchPoints[0]
			theta = np.arccos(np.dot(a,b)/(math.sqrt(np.dot(a,a))*math.sqrt(np.dot(b,b))))
			ab = np.cross(a,b)
			if np.dot(ab,n) >= 0 :
				theta = theta
			else :
				theta = 6.28 - theta
			k_theta[str(K)]=theta
	k_theta_list = sorted(k_theta.iteritems(), key=lambda (ky,v): (v,ky))
	theta_diff = list()
	for j in range(0,len(k_theta_list)):
		if j == 0:
			theta_diff.append(k_theta_list[j][1])
		elif j!=len(k_theta_list)-1:
			theta_diff.append(k_theta_list[j][1]-k_theta_list[j-1][1])
		else:
			theta_diff.append(6.28 - k_theta_list[j][1])
	G_theta = np.nanmax(theta_diff)
	print("G_theta : " + str(round(G_theta,2)))
	if round(G_theta,2) >1.57 :
		boundary_points[count] = cloud[loc]
		count = count+1
		
boundaries.from_array(boundary_points)	
pcl.save(boundaries,'temp.pcd')
print(boundaries)
pcd = read_point_cloud("temp.pcd")
draw_geometries([pcd])
