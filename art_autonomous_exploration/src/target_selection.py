#!/usr/bin/env python

import rospy
import random
import math
import time
import numpy as np
from timeit import default_timer as timer
from utilities import RvizHandler
from utilities import OgmOperations
from utilities import Print
from brushfires import Brushfires
from topology import Topology
import scipy
from path_planning import PathPlanning


# find angle between 2 vectors
def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def anglebetween(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))
##

# Class for selecting the next best target
class TargetSelection:
    
    # Constructor
    def __init__(self, selection_method):
        self.goals_position = []
        self.goals_value = []
        self.omega = 0.0
        self.radius = 0
        self.method = selection_method

        self.brush = Brushfires()
        self.topo = Topology()
        self.path_planning = PathPlanning()


    def selectTarget(self, init_ogm, coverage, robot_pose, origin, \
        resolution, force_random = False):
        
        target = [-1, -1]

        ######################### NOTE: QUESTION  ##############################
        # Implement a smart way to select the next target. You have the 
        # following tools: ogm_limits, Brushfire field, OGM skeleton,
        # topological nodes.

        # Find only the useful boundaries of OGM. Only there calculations
        # have meaning
        ogm_limits = OgmOperations.findUsefulBoundaries(init_ogm, origin, resolution)

        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(init_ogm, ogm_limits)

        # Calculate Brushfire field
        tinit = time.time()
        brush = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)
        Print.art_print("Brush time: " + str(time.time() - tinit), Print.ORANGE)

        # Calculate skeletonization
        tinit = time.time()
        skeleton = self.topo.skeletonizationCffi(ogm, \
                   origin, resolution, ogm_limits)
        Print.art_print("Skeletonization time: " + str(time.time() - tinit), Print.ORANGE)

        # Find topological graph
        tinit = time.time()
        nodes = self.topo.topologicalNodes(ogm, skeleton, coverage, origin, \
                resolution, brush, ogm_limits)
        Print.art_print("Topo nodes time: " + str(time.time() - tinit), Print.ORANGE)
	#for n in nodes:
	  #print n[0],n[1],("space"),
        
        # Visualization of topological nodes
        vis_nodes = []
        for n in nodes:
            vis_nodes.append([
                n[0] * resolution + origin['x'],
                n[1] * resolution + origin['y']
            ])
        RvizHandler.printMarker(\
            vis_nodes,\
            1, # Type: Arrow
            0, # Action: Add
            "map", # Frame
            "art_topological_nodes", # Namespace
            [0.3, 0.4, 0.7, 0.5], # Color RGBA
            0.1 # Scale
        )

    	#Target Selection with cose based method
	if self.method == 'costbased' and force_random == False:
	  target = self.selectCostBasedTarget(coverage, brush, nodes, robot_pose, resolution, origin)



        # Random point
        if self.method == 'random' or force_random == True:
          target = self.selectRandomTarget(ogm, coverage, brush, ogm_limits)
        ########################################################################


        return target

    def selectCostBasedTarget(self, coverage, brush, nodes, robot_pose, resolution, origin):

	tinit = time.time()

	[rx, ry] = [\
            int(round(robot_pose['x_px'] - origin['x'] / resolution)),\
            int(round(robot_pose['y_px'] - origin['y'] / resolution))]
	wtopo = []
	wdist = []
	wcove = []
	wturn = []
	sum = 0
	
	# for all possible targets
        for n in nodes:
	  # we want to make a path for each to calculate the wanted costs
	  path = self.path_planning.createPath([rx, ry], n, resolution)
          if len(path) == 0:
	    
            continue
	  
          # Reverse the path to start from the robot
          path = path[::-1]
	  # Caclulate wDist
	  for i in range(len(path)):
	    if i == len(path) - 1:
	      break
	    sum = sum + math.sqrt( (path[i][0] - path[i+1][0]) ** 2 + (path[i][1] - path[i+1][1]) ** 2)
	  
	  wDist = sum
	  sum = 0
	  print ("wDist"),wDist
	  # Calculate wTurn
	  
	  for i in range(len(path)):
	    if i == len(path) - 1:
	      break
	    if i == 0:
	      if path[2][0] >= path[1][0]:
	        v1 = [1,0]
	      else:
	        v1 = [-1,0]
            else:
	      v1 = v2
	    v2 = [ path[i+1][0] - path[i][0] , path[i+1][1] - path[i][1] ]
	    
	    sum = sum + anglebetween(v1, v2)
	  
	  wTurn = sum
	  sum = 0
		
	  # Calculate wCove
	
	  for i in range(len(path)):
	    xp = int(round(path[i][0]))
	    yp = int(round(path[i][1]))
	    sum = sum + coverage[xp][yp]
	  print ("path length:"),len(path)
	  sum = sum / (255 * len(path))
	  
	  wCove = sum
	  sum = 0

          # Calculate wTopo
	  wTopo = brush[n[0]][n[1]]

	  wtopo.append(wTopo)
	  wcove.append(wCove)
	  wturn.append(wTurn)
	  wdist.append(wDist)
	  
	# Normalization and calculate wcoeff (prioritization) and pPre
	wcoeff = []
	pPre = []
        for i in range(len(wtopo)):
	  wtopo[i] = 1 - (wtopo[i] - min(wtopo))/(max(wtopo) - min(wtopo))
          wcove[i] = 1 - (wcove[i] - min(wcove))/(max(wcove) - min(wcove))
	  wturn[i] = 1 - (wturn[i] - min(wturn))/(max(wturn) - min(wturn))
	  wdist[i] = 1 - (wdist[i] - min(wdist))/(max(wdist) - min(wdist))
	  wcoeff.append( ( 8 * wtopo[i] + 4 * wdist[i] + 2 * wcove[i]  + wturn[i]) / 15)
	  wtopo[i] = round(wtopo[i])
	  wcove[i] = round(wcove[i])
	  wturn[i] = round(wturn[i])
	  wdist[i] = round(wdist[i])
	  pPre.append(8 * wtopo[i] + 4 * wdist[i] + 2 * wcove[i]  + wturn[i])
	
	wfinal = [wcoeff[i] * pPre[i] for i in range(len(wcoeff))]

	findmax = 0
	for i in range(len(wfinal)):
	  if wfinal[i] != max(wfinal):
	    findmax = findmax + 1
	

        Print.art_print("Select target with costed based method time: " + str(time.time() - tinit), \
            Print.ORANGE)
	return nodes[findmax]
	
	
    def selectRandomTarget(self, ogm, coverage, brushogm, ogm_limits):
      # The next target in pixels
        tinit = time.time()
        next_target = [0, 0] 
        found = False
        while not found:
          x_rand = random.randint(0, ogm.shape[0] - 1)
          y_rand = random.randint(0, ogm.shape[1] - 1)
          if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] < 50 and \
              brushogm[x_rand][y_rand] > 5:
            next_target = [x_rand, y_rand]
            found = True
        Print.art_print("Select random target time: " + str(time.time() - tinit), \
            Print.ORANGE)
        return next_target

