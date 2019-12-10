# imports
import math
from math import cos, sin, atan2, radians, degrees, sqrt, hypot
import random
import time
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle, Wedge, Polygon

# Circle class
class Circle:
	def __init__(self, loc, rad):
		self.x = loc[0]
		self.y = loc[1]
		self.r = rad


def plotRect(rectLists, ax):
	for rectList in rectLists:
		x = rectList[0]
		y = rectList[1]
		w = rectList[2]
		h = rectList[3]
		rect = plt.Rectangle((x,y),w,h,alpha=.7, color = (0.5,0.5,0.5))
		ax.add_artist(rect)

def check_vert(x,y, r, rect):
	rect_x = rect[0]
	rect_y = rect[1]
	w,h = rect[2], rect[3]
	d1,_ = distance((rect_x,rect_y), (x,y))
	d2,_ = distance((rect_x + w, rect_y), (x,y))
	d3,_ = distance((rect_x, rect_y + h), (x,y))
	d4,_ = distance((rect_x + w, rect_y + h), (x,y))

	if d1 <= r or d2 <= r or d3 <= r or d4 <= r:
		return True

	return False

def no_collision(px,py,rad,point_lists=[]):
	# rad = 70
	for point_list in point_lists:
		x = point_list[0] #Rectangle bottom left x
		y = point_list[1] #Rectangle bottom left y
		w = point_list[2] #Rectangle width
		h = point_list[3] #Rectangle height

		new_x = x  - rad
		new_y = y - rad
		w = rad * 2 + w
		h = rad * 2 + h

		if (px >= new_x and px <= new_x + w) and ( py >= new_y and py <= new_y + h ):
			return False
	return True

# Distance between 2 points
def distance(loc1, loc2):
	x1,y1 = loc1
	x2,y2 = loc2

	print("loc1:", x1,y1)
	print("loc2:", x2,y2)
	x_dis = x2 - x1
	y_dis = y2 - y1

	print("x_dis", x_dis, "y_dis", y_dis)

	theta = atan2(y_dis, x_dis) 
	mag = sqrt(x_dis**2 + y_dis**2)
	print("magnitude of dist",mag)

	return mag,theta

def circ_intersection(circ1, circ2):
	print("Circle 1 info", circ1.r, circ1.x, circ1.y)
	print("Circle 2 info", circ2.r, circ2.x, circ2.y)
	dis = hypot(circ2.x - circ1.x, circ2.y - circ1.y)

	e_x = (circ2.x - circ1.x) / dis
	e_y = (circ2.y - circ1.y) / dis

	x = (circ1.r * circ1.r - circ2.r * circ2.r + dis * dis) / (2 * dis)
	y = sqrt(circ1.r * circ1.r - x * x)

	p1 = (circ1.x + x * e_x - y * e_y, circ1.y + x * e_y + y * e_x)
	p2 = (circ1.x + x * e_x + y * e_y, circ1.y + x * e_y - y * e_x )

	return p1,p2

def dot_prod(vector1, vector2):
	pass

def integrate(loc, velocity, timestep = 1):
	mag, theta = velocity
	x,y = loc
	x_speed = mag * cos(theta)
	y_speed = mag * sin(theta)

	# print("x_speed and y_speed", x_speed, y_speed)

	new_x = x + x_speed * timestep
	new_y = y + y_speed * timestep

	# print(new_x,new_y)
	return new_x, new_y


def getRVO(robot_a, robot_b):

	alpha = 0.5
	# Find the first triangle
	A = robot_a.get_loc()
	Ax, Ay = A

	# Find the the other 2 points of triangle. 
	# Find point where tangent line from robot A intersect markovski sun of robotA and robotB
	mksvi_radius = robot_b.get_radius() * 2
	Cx, Cy = robot_b.get_loc()
	AO = sqrt((Cx - Ax)**2 + (Cy - Ay)**2)
	print("robot_a", robot_a.get_loc(), "robot_b", robot_b.get_loc())
	print("Ao", AO, "radius", mksvi_radius)
	AP = sqrt((AO**2) - (mksvi_radius**2))

	P1,P2 = circ_intersection(Circle(robot_b.get_loc(), mksvi_radius),
		Circle(robot_a.get_loc(), AP))

	# Translate the points across velocity of a robotB
	mag_b, theta_b = robot_b.get_vel()
	mag_a, theta_a = robot_a.get_vel()
	d_x = mag_b * cos(theta_b)
	d_y = mag_b * sin(theta_b)

	P1x, P1y = P1
	P2x, P2y = P2


	# update triangle points
	A_new = Ax + d_x, Ay + d_y
	P1_new = P1x + d_x, P1y + d_y
	P2_new = P2x + d_x, P2y + d_y

	# Extend size of triangle
	mag, theta = distance (A_new,P1_new)
	P1_new = integrate(A_new, (2000, theta))
	mag, theta = distance (A_new,P2_new)
	P2_new = integrate(A_new, (2000, theta))

	# update triangle points again

	# change from VO to RVO
	mag_b, theta_b = robot_b.get_vel()
	mag_a, theta_a = robot_a.get_vel()
	xa, ya = cos(theta_a) * alpha * mag_a, sin(theta_a) * alpha * mag_a
	xb, yb = cos(theta_b) * alpha * mag_b, sin(theta_b) * alpha * mag_b

	x_sum, y_sum = xa + xb, ya + yb
	x_sum_a, y_sum_a = Ax + x_sum, Ay + y_sum
	vect_mag, vect_theta = distance(A_new, (x_sum_a, y_sum_a))

	d_x = vect_mag * cos(vect_theta)
	d_y = vect_mag * sin(vect_theta)
	
	A_new = A_new[0]+ d_x, A_new[1] + d_y
	P1_new = P1_new[0] + d_x, P1_new[1] + d_y
	P2_new = P2_new[0] + d_x, P2_new[1] + d_y

	return [A_new, P1_new, P2_new]

def form_roadmap():

	return [[(10,-10), (5,-10), (0,-10), (0,12), (-5,12), (-15,15), (-10,-10)],
			[(10,10), (0,0), (-5,0), (-10,0), (-10,10)]]

def plot_roadmap(ax, road_maps):

	lines = []

	for road_map in road_maps:
		x = []
		y = []
		for points in road_map:
			x.append(points[0])
			y.append(points[1])

		line, = ax.plot([], [], lw=2)
		line.set_data([], [])
		lines.append(line)
		line.set_data(x, y)

def line_inter(A,B):
	tol = 0.0002
	A1x, A1y = A[0]
	A2x, A2y = A[1]

	B1x, B1y = B[0]
	B2x, B2y = B[1]

	# slope
	ma = (A2y - A1y) / (A2x - A1x) 
	mb = (B2y - B1y) / (B2x - B1x) 

	# intersecpts
	ba = A1y - ma * A1x
	bb = B1y - mb * B1x

	print("ma", ma, mb)
	print("ba", ba, bb)

	# If lines intersect give a point half way between both lines
	if abs(ma - mb) < tol and abs(bb - ba) < tol:
		d, mag = distance(A[0], B[0])
		mid_point = integrate(A[0], d/2) 
		print("same line")
		return mid_point

	x = (ba - bb)/ (mb - ma)
	y = (ma*bb - mb*ba) / (ma - mb)

	return (x,y)


def consequence(vel1, vel2, loc1, loc2):
	# if magnitude of velocity is 0 there is no consequence
	if vel1[0] == 0 or vel2[0] == 0:
		return 0

	A1 = loc1
	A2 = integrate(A1, vel1)

	B1 = loc2
	B2 = integrate(B1, vel2)

	print("second point A", A2)
	print("second part B", B2)

	int_point = line_inter((A1, A2),(B1, B2))

	if int_point is None:  # no intersection
		return 0

	# distance between robot A to int point is the consequence
	dist,_ = distance(A1, int_point)
	return dist

def readPaths(directory):
	road_maps = []
	filenames = []

	print(directory)

	for filename in os.listdir(directory):
	     # filename = os.fsdecode(file)
		if filename.endswith("txt"): 
			full_file_name = os.path.join(directory, filename)
			print(full_file_name)
			filenames.append(full_file_name)
		else:
			continue

	for filename in filenames:
		lines = [line.rstrip() for line in open(filename) if len(line.rstrip()) > 0]

		if len(lines) == 0:
		    print("That files empty")
		    sys.exit(1)

		cspace = lines[0].strip()
		if (cspace != 'R2' and cspace != 'SE2' and cspace!= 'Weird'):
		    print ("Unknown C-space Identifier: ", cspace)
		    sys.exit(1)

		data = [[float(x) for x in line.split(' ')] for line in lines[1:]]
		road_maps.append(data)
	return road_maps

def plotRing(small_r, big_x, big_y, num_circles, big_rad):
	phi = 2 * math.pi
	xvar = 2/num_circles
	start =  []
	
	for i in range(0,num_circles):
		x = big_x + (big_rad * math.cos(phi + i * xvar * math.pi )) #x coordinate of small circle
		y = big_y + (big_rad * math.sin(phi + i * xvar * math.pi )) #y coordinate of small circle
		start.append((x,y))

	nn = int(num_circles / 2)
	print(nn)
	first_half = start[:nn]
	second_half = start[nn:]
	goal = second_half + first_half

	return start, goal


def inter_point(A,B,C,D):
	a1 = B[1] - A[1]
	b1 = A[0] - B[0]

	c1 = a1*A[0] + b1*A[1]

	a2 = D[1] - C[1]
	b2 = C[0] - D[0]
	c2 = a2*C[0] + b2*C[1]

	determinant = a1*b2 - a2*b1

	if (determinant == 0):
		return False, (0,0)
	
	x = (b2*c1 - b1*c2)/determinant
	y = (a1*c2 - a2*c1)/determinant

	return True, (x,y)

def get_penalty(robot1_loc, vel1, robot2_loc, vel2):
	A = robot1_loc
	B = integrate(A,vel1)
	C  = robot2_loc
	D = integrate(C, vel2)

	collision, int_point = inter_point(A,B,C,D)
	print("A {}, B {}, C {}, D {}".format(A,B,C,D))
	if collision:
		print("A is {} and int point is {}".format(A,int_point))
		dist, theta = distance(A, int_point)

		print("dist is: {}".format(dist))
		t = dist/vel1[0]
		penalty = 1/t

		return penalty

	return 0


if __name__ == "__main__":	
	# vel1 = (1,radians(90))
	# vel2 = (1, radians(90+180))
	# loc1 = (0,0)
	# loc2 = (0,2)
	# print("consequences", consequence(vel1, vel2, loc1, loc2))
	# radius = 2
	# big_x, big_y = 0,0
	# num_circles = 6
	# big_rad = 6
	# starts, goals = plotRing(radius, big_x,big_y,num_circles,big_rad)
	# print(starts)
	# print()
	# print(goals)
	pen = penalty((0,0),(sqrt(2),radians(45)), (3,1), (sqrt(2),radians(90+45)))
	print("Penalty id {}".format(pen))