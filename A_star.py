import cv2
import numpy as np
import time
from utils import *

class Robot:
	orn = 0
	step = 1
	cl = 5
	rd = 10
	parent = 0
	cost = 0
	pos = [rd, rd, orn]

	def __init__(self, position, radius, step, clearance):
		self.rd = radius
		self.step = step
		self.cl = clearance
		self.orn = position[2]
		self.pos = position


	def move_possible(self, move, graph):
		pass

	def action(self, storage, opn, visited, graph):
		action_set = self.orn+[-np.pi/3, -np.pi/6, 0, np.pi/6, np.pi/3]
		for i in action_set:
			new_x = round(self.step*np.cos(i))+self.pos[1]
			new_y = round(self.step*np.sin(i))+self.pos[0]
			new_orn = i
			move = [new_y, new_x, new_orn]

			if self.move_possible(move, graph):
				if not exist(move, visited):
					cost = self.cost + step
					storage[len(storage)].append([len(storage), move, cost, self.parent])
				else:
					location = find(move, visited)
					cost = self.cost + step
					if storage[location][2] > cost:
						storage[location][2] = cost
						storage[location][3] = self.parent



	def perform_action(self, move):
		pass

def find():
	pass

def exist():
	pass

def check(coords1, coords2):
	if coords1 == coords2:
		return True
	else:
		return False

def get_newnode(opn, goal):
	ar_opn = np.array(opn)[:,1]
	ar_opn = np.array([np.array(k) for k in ar_opn])
	dist = np.sum((ar_opn[:2]-goal[:2])**2, axis=1)
	idx = np.argmin(dist)
	curr_node = opn.pop(idx)
	return curr_node, opn

def a_star(goal, robot):
	storage = {}
	graph = create_graph()
	import pdb; pdb.set_trace()
	closed = np.zeros((500,800,5))
	storage[0].append([0, robot.pos, 0, 0]) # [num, coords, cost, parent]
	closed[]
	opn = []

	while not check(robot.pos, goal):
		storage, opn, closed = robot.action(storage, opn, closed, graph)
		node, opn = get_newnode(opn, goal)
		robot.pos = node[1]
		robot.parent = node[0]
		robot.cost = node[2]
		robot.orn = node[1][2]


def origin_shift(initial, goal):
	graph = create_graph()
	origin_shift = graph.shape[0]-1
	initial[0] = origin_shift - initial[0]
	goal[0] = origin_shift - goal[0]
	return initial, goal


def main():
	ip_x = int(input("Enter initial x coordinate:\n"))
	ip_y = int(input("Enter initial y coordinate:\n"))
	ip_orn = int(input("Enter initial orientation:\n"))
	g_x = int(input("Enter goal x coordinate:\n"))
	g_y = int(input("Enter goal y coordinate:\n"))
	g_orn = int(input("Enter goal orientation:\n"))
	step = int(input("Enter step size between 1 and 10 inclusive:\n"))
	clearance = int(input("Enter object clearance:\n"))
	radius = int(input("Enter robot's radius:\n"))
	start_time = time.time()

	ip_orn = ip_orn*(np.pi/180)
	g_orn = g_orn*(np.pi/180)

	initial = [ip_y, ip_x, ip_orn]
	goal = [g_y, g_x, g_orn]
	initial, goal = origin_shift(initial, goal)
	
	robot = Robot(initial, radius, step, clearance)

	a_star(goal, robot)
	end_time = time.time()

if __name__ == '__main__':
	main()