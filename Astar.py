import cv2
import numpy as np
import heapq
import time
from math import cos, sin, pi

class Node():
	def __init__(self, coord) :
		self.coord = coord
		self.weight = np.inf
		self.parent = None

	def print_me(self):
		print("Coord :", self.coord)
		print("Weight :", self.weight)
	
	def __lt__(self, nxt):
		return self.weight < nxt.weight

def backtrack(node, amg, pts) :
	amg = np.uint8(amg)
	height, width, layers = amg.shape  
	video= cv2.VideoWriter('Astar.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 1200, (width,height))

	pts = []
	cost = node.weight
	while node.parent != None :
		print(f'Y coord : {node.coord[0]} and X coord : {node.coord[1]}')
		pts.append([node.coord[1], node.coord[0]])
		node = node.parent

	npts = pts[::-1]
	for pt in npts :
		for _ in range(5): 
			amg[249-pt[0]][pt[1]] = [0,255,0]
			cv2.imshow('Path', amg)
			cv2.waitKey(1)
			video.write(amg) 

	# cv2.imshow('Path', amg)
	# cv2.waitKey(0)
	cv2.destroyAllWindows() 
	video.release()  # releasing the video generated

	print(f'Cost {cost}')
	return pts, amg

def eq(x1,y1,x2,y2,x,y,f):
	m = (y2-y1)/(x2-x1)
	if (f == 1):
		c = (m*x) - y <= (m*x1) - y1
	else:
		c = (m*x) - y >= (m*x1) - y1
	return c

def create_map():
	m = np.zeros((250,400))
	am = np.zeros((250,400,3))
	hl = 40.4145
	for y in range(m.shape[0]):
		for x in range(m.shape[1]):
			if (((y - 65) ** 2) + ((x - 300) ** 2) <= ((40) ** 2) ):
				m[y,x]=1
				am[y,x]=[0,0,255]
			if ((x > 200-35) and (x < 200 + 35) and (y <= 150) and eq(200,150-hl,165,150-(hl/2),x,y,1) and eq(200,150-hl,235,150-(hl/2),x,y,1) ):
				m[y,x]=1
				am[y,x]=[0,0,255]
			if ((x > 200-35) and (x < 200 + 35) and (y >= 150) and eq(200,150+hl,165,150+(hl/2),x,y,2) and eq(200,150+hl,235,150+(hl/2),x,y,2) ):
				m[y,x]=1
				am[y,x]=[0,0,255]
			if (eq(36,65,115,40,x,y,1) and eq(36,65,105,150,x,y,2) and (eq(80,70,105,150,x,y,1) and (y >= 250-180))):
				m[y,x]=1
				am[y,x]=[0,0,255]
			if (eq(36,65,115,40,x,y,1) and eq(36,65,105,150,x,y,2) and (eq(115,40,80,70,x,y,2) and (y <= 250-180))):
				m[y,x]=1
				am[y,x]=[0,0,255]

	return m,am

def inObs(x,y,img) :
	
	for i in range(5) :
		cl = i
		if 250-(y+cl) > 0 :
			if img[249-y-cl][x] == 1 : return True
			
		if 250-(y-cl) < 250 :
			if img[249-(y-cl)][x] == 1 : return True
			
		if 250-(y+cl) > 0 and  x+cl < 399 :
			if img[249-(y+cl)][x+cl] == 1 : return True
		
		if 250-(y+cl) > 0 and x-cl > 0 :
			if img[249-(y+5)][x-cl] == 1 : return True
		
		if 250-(y-cl) < 250 and x+cl < 399:
			if img[249-(y-cl)][x+cl] == 1 : return True

		if 250-(y-cl) < 250 and x-cl > 0: 
			if img[249-(y-cl)][x-cl] == 1 : return True
		
		if x-cl > 0 :
			if img[249-y][x-cl] == 1 : return True
		
		if x+cl < 399:
			if img[249-y][x+cl] == 1 : return True
		
	return False

def euclid(x,y,fpoint) :
	return ((x-fpoint[0])**2 + (y-fpoint[1])**2)**(1/2)

def checkMove(node, img, step, angle) :
	x, y, theta = node.coord[0], node.coord[1], node.coord[2]
	theta = theta % 360
	xc, yc, thetac = int(x+step*cos((theta+angle)*pi/180)), int(y+step*sin((theta+angle)*pi/180)), int(theta+angle)
	thetac = thetac % 360
	if xc >= 0 and xc < 400 and yc >= 0 and yc < 250 and (not inObs(xc,yc,img)) :
		return True, xc, yc, thetac
	else : 
		return False, xc, yc, thetac

def findNeigh(node, visited, nodelist, obs, step, fpoint, ipoint):
	x, y = node.coord[0], node.coord[1]
	ne = []
	fn, x1, y1, theta1 = checkMove(node, img, step, 60)
	sn, x2, y2, theta2 = checkMove(node, img, step, 30)
	tn, x3, y3, theta3 = checkMove(node, img, step, 0)
	fon, x4, y4, theta4 = checkMove(node, img, step, -30)
	fin, x5, y5, theta5 = checkMove(node, img, step, -60)

	if fn : 
		fnode = Node([x1, y1, theta1])
		xc, yc, thetac = x1, 249-(y1), theta1
		if not visited[yc][xc][thetac] and [x1, y1, theta1] not in nodelist :
			ne.append([fnode,euclid(xc,yc,fpoint)+euclid(xc,yc,[x,y])])

	if sn : 
		snode = Node([x2, y2, theta2])
		xc, yc, thetac = x2, 249-(y2), theta2
		if not visited[yc][xc][thetac] and [x2, y2, theta2] not in nodelist :
			ne.append([snode,euclid(xc,yc,fpoint)+euclid(xc,yc,[x,y])])

	if tn : 
		tnode = Node([x3, y3, theta3])
		xc, yc, thetac = x3, 249-(y3), theta3
		if not visited[yc][xc][thetac] and [x3, y3, theta3] not in nodelist :
			ne.append([tnode,euclid(xc,yc,fpoint)+euclid(xc,yc,[x,y])])

	if fon : 
		fonode = Node([x4, y4, theta4])
		xc, yc, thetac = x4, 249-(y4), theta4
		if not visited[yc][xc][thetac] and [x4, y4, theta4] not in nodelist :
			ne.append([fonode,euclid(xc,yc,fpoint)+euclid(xc,yc,[x,y])])

	if fin : 
		finode = Node([x5, y5, theta5])
		xc, yc, thetac = x5, 249-(y5), theta5
		if not visited[yc][xc][thetac] and [x5, y5, theta5] not in nodelist :
			ne.append([finode,euclid(xc,yc,fpoint)+euclid(xc,yc,[x,y])])

	return ne


def astar(ipoint, fpoint, im, am, step) :
	img = im.copy()
	amg = am.copy()
	thresh = 0.5
	visited = np.zeros((500, 800, 360), dtype=bool)
	inode = Node(ipoint)
	# inode.weight = 0
	inode.weight = euclid(inode.coord[0],inode.coord[1], fpoint)
	nlist = [inode]
	heapq.heapify(nlist)
	pts = []
	allpts = []
	nodelist = [ipoint]

	while len(nlist) != 0 :
		pnode = nlist[0]
		pcoord = pnode.coord
		pweight = pnode.weight

		visited[249-pcoord[1]][pcoord[0]][pcoord[2]] = True
		heapq.heappop(nlist)

		if pcoord[0] == fpoint[1] and pcoord[1] == fpoint[0] and pcoord[2] == fpoint[2] :
			print('Goal Reached')
			pts, amg = backtrack(pnode, amg, allpts)
			# cv2.imwrite('Path.jpg', amg)
			break 
		else :
			img1 = img.copy()
			children = findNeigh(pnode, visited, nodelist, img1, step, fpoint, ipoint)
			for i,cbox in enumerate(children) : 
				cnode = cbox[0]
				wt = cbox[1]
				cweight = cnode.weight
				ccoord = cnode.coord
				nodelist.append(ccoord)
				print(ccoord)

				if cweight > pweight+wt :
					cweight = pweight+wt
					cnode.weight = pweight+wt
					cnode.parent = pnode
					allpts.append([249-ccoord[1], ccoord[0], ccoord[2]])
					heapq.heappush(nlist, cnode)

	return pts

if __name__ == "__main__" :
	stime = time.time()
	img, amg= create_map()
	cv2.imwrite('img.jpg', amg)

	ipointx = input('Enter start x point : ')
	ipointy = input('Enter start y point : ')
	ipointt = input('Enter start theta point : ')

	fpointx = input('Enter final x point : ')
	fpointy = input('Enter final y point : ')
	fpointt = input('Enter final theta point : ')

	step = input('Enter step size : ')

	if ipointx.strip() == '' or ipointy.strip() == '' or fpointx.strip() == '' or fpointy.strip() == '' or fpointt.strip() == '' or ipointt.strip() == '':
		print('Enter the points to begin the code')
	else :
		ipoint = [int(ipointx),int(ipointy), int(ipointt)%360]
		fpoint = [int(fpointy),int(fpointx), int(fpointt)%360]
	
		if (ipoint[0] < 0 or ipoint[0] > 399 or ipoint[1] < 0 or ipoint[1] > 249) or (fpoint[0] < 0 or fpoint[0] > 249 or fpoint[1] < 0 or fpoint[1] > 399):
			print('Out of bounds')
		else :
			if inObs(ipoint[0],ipoint[1],img) or inObs(fpoint[1],fpoint[0],img) :
				print('Given points in obstacle space')
			else :
				pts = astar(ipoint, fpoint, img, amg, int(step))
	etime = time.time()
	print(f'Total Time {etime-stime}')
