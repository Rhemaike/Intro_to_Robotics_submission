#!/usr/bin/env python

from apriltag_vid import tracker
from cozmo_robot import Cozmo
import cozmo
from cozmo.util import degrees
import asyncio
from multiprocessing import Process, Queue, Manager
import sys
import time

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle, Wedge, Polygon
from funcs import plotRect

time_step = 1.2
radius = 70

# Initialize environment

fig = plt.figure()
ax = plt.axes(xlim=(-900, 900), ylim=(-700, 700))
ax.set_aspect('equal')
rectList = [[200,-210,200,150],[-320,100,200,150]]

patches = {}
ob_patches = {}
lines = {}



async def run_cozmo(ids, goals, location_q, loop, conns, loc, obs, ob_ids):

	# plotRect(rectList, ax)
	cozmos = {}
	robots = {}
	taskList = []
	rconn = {}
	velocity = {}
	velocity_no_rect = {}

	at_goal = False

	num_circles = 0
	for i, cid in enumerate(ids):
		c = await conns[cid].wait_for_robot()
		rconn[cid] = c
		num_circles += 1

	r_list = []
	for i in ob_ids:
		w = 200
		h = 150

		cx, cy = obs.value[i]
		hw = int(w/2)
		hh = int(h/2)

		bx,by = cx - hw, cy - hh
		rect_sig = [bx,by,w,h]
		r_list.append(rect_sig)

	col = 0
	incr = 1 / num_circles
	for i, cid in enumerate(ids):
		coz = Cozmo(rconn[cid], cid, goals[cid], location_q[cid], loc, ax, r_list)
		cozmos[cid] = coz

		#get current cozmo location
		start_loc = loc.value.get(cid)
		if start_loc is not None:
			patches[cid] = plt.Circle(cozmos[cid].get_loc(), radius, color=(col,0,col))
			goal = plt.Circle(goals[cid], 50,  alpha = 0.4, color=(col,0,col))
			col += incr
			ax.add_patch(patches[cid])
			ax.add_patch(goal)
			line, = ax.plot([], [], lw=2)
			line.set_data([], [])
			lines[cid] = line

	

	for i, cid in enumerate(ids):
		cozmos[cid].robots = cozmos

	while not at_goal:
		# calculate velocities
		r_list = []
		for i in ob_ids:
			w = 200
			h = 150

			cx, cy = obs.value[i]
			hw = int(w/2)
			hh = int(h/2)

			bx,by = cx - hw, cy - hh
			ob_patches[i] = plt.Rectangle((bx,by),w,h,alpha=.7, color = (0.5,0.5,0.5))
			ax.add_patch(ob_patches[i])
			rr = [bx,by,w,h]
			r_list.append(rr)

		for cid in ids:
			patches[cid].center = (cozmos[cid].get_loc())
			cozmos[cid].rects = r_list

		for i, cid in enumerate(ids):
			velocity[cid], velocity_no_rect[cid] = cozmos[cid].get_ideal_velocity()
			print("Cozmo {} ideal velocity id {}".format(cid, velocity[cid]))
			in_loc = cozmos[cid].get_loc()
			loc_x, loc_y = in_loc
			new_x, new_y = cozmos[cid].new_loc(velocity_no_rect[cid])
			x = [loc_x,new_x]
			y = [loc_y, new_y]
			print("###################################")
			print("current location {}, new location {}".format((loc_x, loc_y),(new_x, new_y)))
			print("###################################")
			lines[cid].set_data(x, y)

		plt.pause(0.002)


		# move cozmos with velocity
		taskList = []
		for i, cid in enumerate(ids):
			for p in cozmos[cid].RVO_patches:
				p.remove()

			print("move cozmo", cozmos[cid].id)
			print("Chosen_velocities:", velocity[cid][0], velocity[cid][1])
			cozmos[cid].update_neighbours()
			cozmos[cid].update_rov_obstacles()

			if not cozmos[cid].is_at_goal():

				for RVO in cozmos[cid].RVOs:
					p = plt.Polygon(RVO, alpha = 0.09, color=(0.5,0.5,0))
					cozmos[cid].RVO_patches.append(p)
					print("**** ROVS are {}**************".format(RVO))
					ax.add_patch(p)

				taskList.append(loop.create_task(cozmos[cid].move_to_goal(velocity[cid], velocity_no_rect[cid])))
		
		for i in ob_ids:
			ob_patches[i].remove()

		await asyncio.wait(taskList)
		print("task list done")

		# Check if correct cozmo is at goal
		at_goal = True
		for i, cid in enumerate(ids):
			if not cozmos[cid].is_at_goal():
				at_goal = False

		await asyncio.sleep(0.1)

	print("Goal completed")

	plt.show()

		
def run(ids, goals, location_q, loc, conn_names,obs, ob_ids):
	loop = asyncio.get_event_loop()
	conns = {}
	try:
		for i in ids:
			conn = (cozmo.connect_on_loop(loop))
			name = conn.device_info["serial"]
			conns[conn_names[name]] = conn
	except cozmo.ConnectionError as e:
		sys.exit('A connection error occurred: {}'.format(e))
	loop.run_until_complete(run_cozmo(ids, goals, location_q, loop, conns, loc, obs, ob_ids))
	print("Complete")
	loop.close()


if __name__ == "__main__":
	ids = [6, 7, 1, 0]
	ob_ids = [12,19]
	# ids = [6,7]
	con_names = {"2618a00125057ece":7,"LGM150378704f0":0,"LGM150fe79d4f0":6,"LGM150378718de":1}
	# ids = [6]
	# ids = [7]


	goals = {7:(-500,0), 6:(500,0), 1:(40,-400), 0:(30,378)} #  goal


	# for key,items in goals.items():
	# 	goal = plt.Circle(items, 50,  alpha = 0.4, color=(0,1,0))
		
	# 	ax.add_patch(goal)


	# obscales = {1:(153,-189), 2:(116,95)}
	# goals = {7:(-500,0), 6:(500,0), 0:(30,378)} #  goal

	# goals = {6:(-500,0), 7:(500,0)}
	# goals = {6:(500,0)}
	# goals = {7:(-500,0)}
	location_q = {}
	m = Manager()
	o = Manager()
	loc = m.Value('loc',{7:(-500,0,0,0), 6:(500,0,0,0), 0:(30,378,0,0), 1:(40,-400,0,0)})
	obs = o.Value('obs',{12:(200,-210), 19:(-320,10)})

	# loc = m.Value('loc',{6:(-500,0,0,0), 7:(500,0,0,0)})
	# loc = m.Value('loc',{6:(500,0,0,0)})
	# loc = m.Value('loc',{7:(-500,0,0,0)})


	for i in ids:
		q = Queue()
		location_q[i] = q

	cozmo_proc = Process(target=run, args=[ids, goals, location_q, loc, con_names, obs, ob_ids])
	tracker_proc = Process(target=tracker, args=[location_q, ids, goals, loc,obs,ob_ids])

	tracker_proc.start()
	# time.sleep(5)
	cozmo_proc.start()

	tracker_proc.join()
	cozmo_proc.join()
	


