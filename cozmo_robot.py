import asyncio
import cozmo
import cozmo.util
from cozmo.util import distance_mm, speed_mmps

from math import cos, sin, atan2, radians, degrees, sqrt, hypot, pi
import random
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle, Wedge, Polygon
import threading

import funcs
from funcs import plotRect, check_vert, getRVO, no_collision, distance, \
					circ_intersection, dot_prod, integrate, form_roadmap, plot_roadmap, get_penalty


time_step = 1.2#secs
N_dist = 600 #mm
radius = 70 #mm
Max_vel = 50 #mm/sec
rectList = []
goal_radius = 50 #mm
retries = 500

rectList = [[200,-210,200,150],[-320,100,200,150]]

class Cozmo:
	def __init__(self, robot, cid, goal, queue, loc, ax, rects):
		self.robot = robot
		self.ax = ax
		self.robots = None
		self.vel = (0,0)
		self.man_loc = loc
		self.loc = None
		self.id = cid
		self.queue = queue
		# self.thread = threading.Thread(target=self.update_loc, args=[])
		self.cozmos = []
		self.at_goal = False
		self.neighbours = []
		self.goal = goal
		self.RVOs = []
		self.RVO_patches = []
		self.rad = radius
		self.rects = rects


		# self.thread.start()
		# time.sleep(6)

	def update_loc(self):
		# print("in thread")
		while not self.is_at_goal():
			# print("in update loop")
			if not self.queue.empty():
				print("in update loop but nothing")
				self.loc = self.queue.get()

	def get_radius(self):
		return self.rad

	def get_id(self):
		return self.id

	def get_loc(self):
		self.loc = self.man_loc.value[self.id]
		return (self.loc[0], self.loc[1])

	def set_loc(loaction):
		self.loc = location

	def get_vel(self):
		return self.vel

	def set_vel(self, velocity):
		self.vel = velocity

	def get_pref_vel(self, goal=None):
		self.loc = self.man_loc.value[self.id]
		if goal is None:
			goal = self.goal
			print("goal is", self.goal)
		mag,theta = distance((self.loc[0],self.loc[1]), goal)
		if mag > Max_vel:
			mag = Max_vel

		print("::::::::::::::::::::::::::::")
		print("get pref velocity, mag: {} theta: {}".format(mag,degrees(theta)))
		print("goal: {}, location: {}".format(self.goal, (self.loc[0],self.loc[1])))
		print("::::::::::::::::::::::::::::")

		# if theta < 0:
		# 	theta = 2*math.pi - self.loc[2]

		return mag, theta

	def get_ideal_velocity(self):
		print()
		print()
		print()
		r_range = 140

		pref = self.get_pref_vel()

		pref_mag, pref_theta = pref
		pref_theta = degrees(pref_theta)

		if pref_theta < 0 and (-1*pref_theta) > 180:
			pref_theta = 360 + pref_theta
		elif pref_theta >= 0 and (pref_theta) > 180:
			pref_theta = pref_theta - 360
		
		print("pref mag", pref_mag, "pref theta", pref_theta, "goal is ", self.goal)
		self.loc = self.man_loc.value[self.id]
		A = (self.loc[0], self.loc[1])
		B = self.new_loc((Max_vel, (radians(pref_theta) - radians(r_range))))
		C = self.new_loc((Max_vel, (radians(pref_theta) + radians(r_range))))

		# Check if preferred velocity intersects with any robots velocity obstacle or RVO
		valid = self.no_rov_in((pref_mag, radians(pref_theta)))
		if valid:
			px,py = self.new_loc((pref_mag, radians(pref_theta)))
			valid = no_collision(px,py, radius, self.rects)
			if not valid: print("collision with obstacle","px", px, "py", py, "radius", self.get_radius())
		print()
		if valid:
			print("Valid with velocity", pref[0], pref_theta)
			# self.move_robot(pref)
			chosen_vel = (pref[0], radians(pref_theta))
			
		else:
			print("Trying retries, InValid")
			for i in range(0, retries):
				# sample  velocities out of available velocities
				mag  = random.uniform(0, Max_vel)
				theta = random.uniform(radians(pref_theta) - radians(r_range), radians(pref_theta) + radians(r_range))

				valid = self.no_rov_in((mag, theta))
				# Check if velocities intersect obstacles
				if valid:
					px,py = self.new_loc((mag, theta))
				
					if not no_collision(px,py,radius, self.rects):
						valid = False
						continue


				if valid:
					# self.move_robot((mag, theta))
					# return mag, theta
					chosen_vel = (mag, theta)
					print("First loop found valid")
					break

				print("Invalid velocity", mag, degrees(theta))
				print("InValid")
			if not valid:
				print("Second retry loop")
				for i in range(0, retries):
					# sample  velocities out of available velocities
					mag  = random.uniform(0/2, Max_vel)
					theta = random.uniform(radians(pref_theta) - radians(r_range), radians(pref_theta) + radians(r_range))

					valid = self.no_rov_in((mag, theta))
					# Check if velocities intersect obstacles
					if valid:
						px,py = self.new_loc((mag, theta))

						if not no_collision(px,py,radius, self.rects):
							
							chosen_vel = (mag, theta)
							valid = False
							continue
						# valid = True

					if valid:
						# self.move_robot((mag, theta))
						# return mag, theta
						chosen_vel = (mag, theta)
						break

					print("Invalid velocity", mag, degrees(theta))
					print("InValid")

				# if not valid: chosen_vel = (0,0)

			av_velocities = []
			vel_penalities = []
			if not valid:
				print("third retry loop")
				for i in range(0, retries):
					# sample  velocities out of available velocities
					mag  = random.uniform(0, Max_vel)
					theta = random.uniform(radians(pref_theta) - radians(r_range+10), radians(pref_theta) + radians(r_range))

					valid = self.no_rov_in((mag, theta))
					# Check if velocities intersect obstacles
					# if valid:
					# 	px,py = self.new_loc((mag, theta))
					# 	# valid = no_collision(px,py,radius, rectList)
					# 	valid = True

					if valid:
						px,py = self.new_loc((mag, theta))
						valid = no_collision(px,py,radius, self.rects)

					if valid:
						chosen_vel = (mag, theta)
						break

					penaltys = []
					for nb in self.neighbours:
						# print("I HAVE A NEIGHBOUR")
						penaltys.append(get_penalty(self.get_loc(), (mag,theta), nb.get_loc(), nb.get_vel()))

					# print("penalties",penaltys)
					if len(penaltys) > 0:
						penalty  = max(penaltys)

					else:
						penalty = 0
					
					prob_vel = (mag, theta)
					px,py = self.new_loc((mag, theta))
					if no_collision(px,py,radius, self.rects):
						av_velocities.append(prob_vel)
						vel_penalities.append(penalty)

					else:
						valid = False
						continue
						av_velocities.append(prob_vel)
						vel_penalities.append(1000000000)
						

					print("Invalid velocity", mag, degrees(theta))
					print("InValid")

				# if not valid: chosen_vel = (0,0)

			if not valid: 
				p_loc = vel_penalities.index(min(vel_penalities))
				chosen_vel = av_velocities[p_loc]

				
			# return (0,0)

		print("location is",self.loc)
		axle_len = 0.07
		
		mag, theta = chosen_vel

		pp = self.new_loc((mag, theta))
		goal = plt.Circle(pp,20,  alpha = 0.3, color=(1,0,0))
		self.ax.add_patch(goal)

		px,py = self.new_loc((mag, theta))
		print("::::::::::::::::::::::::::::::::::")
		print(":::::::: RECT LIST: {}, radius {}, px,py {},{}::::::::::::".format(self.rects,radius,px,py))
		print("::::::::::::::::::::::::::::")
		if no_collision(px,py,radius, self.rects):
			print("::::::::::::::::::::::::::::::::::")
			print(":::::::: NO COLLISION ::::::::::::")
			print("::::::::::::::::::::::::::::")
		else:
			print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
			print("!!!!!COLLLLLIIISSIOOOON!!!!!!!!!!!!!")
			print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

		theta = degrees(theta)

		curr_theta = degrees(self.loc[2])

		first_turn_angle = int(curr_theta)

		print("first_turn_angle/ orientation", first_turn_angle)
		print("theta", theta)

		rect_angle = theta + first_turn_angle

		if rect_angle < 0 and (-1*rect_angle) > 180:
			rect_angle = 360 + rect_angle
		elif rect_angle >= 0 and (rect_angle) > 180:
			rect_angle = rect_angle - 360

		print("prefered velocity {} {}".format(mag, rect_angle))
		return (mag, radians(rect_angle)), (mag,radians(theta))

	def update_rov_obstacles(self):
		self.RVOs = []
		

		self.RVO_patches = []
		for neighbour in self.neighbours:
			r = radius
			RVO = getRVO(self, neighbour)
			self.RVOs.append(RVO)
			print("ROV is", RVO)

			

	def update_neighbours(self):
		self.neighbours = []
		self.loc = self.man_loc.value[self.id]
		print()
		print("update_neighbours", self.robots[self.id])
		for k,robot in self.robots.items():
			r_loc = robot.get_loc()

			if (self.id != robot.get_id()):
				mag,_ = distance((self.loc[0], self.loc[1]), (r_loc[0], r_loc[1]))
				if mag < N_dist:	
					self.neighbours.append(robot)

	def no_rov_in(self, vel):
		for ROV in self.RVOs:
			Px,Py = self.new_loc(vel)
			A,B,C = ROV[0], ROV[1], ROV[2]
			Ax,Ay = A
			Bx,By = B
			Cx,Cy = C

			numerator = Ax * (Cy - Ay) + (Py - Ay) * (Cx - Ax) -  Px * (Cy - Ay)
			denomenator = (By - Ay) * (Cx - Ax) - (Bx - Ax) * (Cy - Ay)

			w1 = numerator / denomenator

			numerator = Py - Ay - w1 * (By - Ay)
			denomenator = Cy - Ay

			w2 = numerator / denomenator
			
			if w1 >= 0 and w2 >= 0 and (w1 + w2) <= 1: 
				print("Not in rec")
				return False

		return True

	def new_loc(self,velocity):
		self.loc = self.man_loc.value[self.id]
		return integrate((self.loc[0], self.loc[1]), velocity, time_step)

	def is_at_goal(self, goal=None):
		self.loc = self.man_loc.value[self.id]
		if len(self.loc) == 0:
			return False
		if goal is None:
			goal = self.goal
		x, y = self.loc[0], self.loc[1]
		gx, gy = goal
		if (abs(x - gx) < goal_radius) and (abs(y - gy) < goal_radius):
			return True


	async def move_to_goal(self,vel, vel_no_rect):
		i = 0
		
		self.loc = self.man_loc.value[self.id]
		print("location is this",self.loc, "man_loc", self.man_loc)
		

		# print("obstacles_set")
		# while( (not self.is_at_goal()) and self.loc[3] == 1 ):
		# 	print()
		# 	print()
		# 	print("goal is ", self.goal, "self id is", self.id)
		# 	if self.loc[3] == 0:
		# 		await asyncio.sleep(time_step)
		# 		continue

		# 	i = i+1
		# 	self.loc = self.man_loc.value[self.id]

		# 	if len(self.loc) == 0:
		# 		print("no loc yet")
		# 		await asyncio.sleep(time_step)
		# 		continue

			# print("location is",self.loc)
			# print("hola2")
			# axle_len = 0.07
			# self.update_neighbours()
			# self.update_rov_obstacles()
			# vel = self.get_ideal_velocity()
			
			# mag, theta = vel
			# theta = degrees(theta)

			# curr_theta = degrees(self.loc[2])

			# first_turn_angle = int(curr_theta)

			# print("first_turn_angle", first_turn_angle)
			# print("theta", theta)

			# rect_angle = theta + first_turn_angle

			# if rect_angle < 0 and (-1*rect_angle) > 180:
			# 	rect_angle = 360 + rect_angle
			# elif rect_angle >= 0 and (rect_angle) > 180:
			# 	rect_angle = rect_angle - 360

		mag, rect_angle = vel

		print("*******************************************")
		print("Velocity of cozmo {}, magnitude {}, rect_angle {}, ".format(self.id, mag, degrees(rect_angle) ) )
		print("*******************************************")
		self.set_vel(vel_no_rect)

		# robot turns first
		# print("turn")
		await self.robot.turn_in_place(cozmo.util.degrees(degrees(rect_angle))).wait_for_completed()
		# print("turn2")
		# await self.robot.turn_in_place(cozmo.util.degrees(theta)).wait_for_completed()
		# print("comp2")
		if mag < 10:
			mag = 10
		await self.robot.drive_straight(distance_mm(mag*time_step), speed_mmps(mag)).wait_for_completed()
		# print("hola2")
		# await robot.turn_in_place(degrees(-theta)).wait_for_completed()
		# await asyncio.sleep(time_step)


		# self.vel = (0, 0)

		# stop events on cozmo
		# self.thread.join()
		print("location is",self.loc)
		# await asyncio.sleep(1)
		self.robot.stop_all_motors()

			# TODO: Update location
			# TODO: check if at goal an dif so set self. at goal to True

	def __del__(self):
		# self.thread.join()
		pass