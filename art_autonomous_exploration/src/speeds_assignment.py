#!/usr/bin/env python

import rospy
import math
import time



from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation


# Class for assigning the robot speeds 
class RobotController:
    # Constructor
    def __init__(self):
        
      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
      # Produce speeds
      self.produceSpeeds()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0 
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
        print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
      scan = self.laser_aggregation.laser_scan
      linear  = 0
      angular = 0
      ############################### NOTE QUESTION ############################
      # Check what laser_scan contains and create linear and angular speeds
      # for obstacle avoidance

      length = len(scan) # lenscan = 667
      angles = [math.radians(-135 + i * 270/(length - 1)) for i in range(length)]
      for i in range(length):
        linear = linear - math.cos(angles[i]) / scan[i] ** 2
	angular = angular - math.sin(angles[i]) / scan[i] ** 2


# tried a different solution but it didn't work that well in the end


#      # 270 deg total. We want to go to the direction of which the lidar distance is max.
#      # if we find the max(scan) its in a 270/667 = 0.4 deg range.
#      # So we take samples of k = 10 to have a 4 deg range
#      k = 10 
#      index = 0 # here we count the sum of k samples of scan
#      count = 0 # here we count which element of our newscan to put the sum of above
#      j = 0 #j counts to k
#      i = 0 #i will scan the range or scan
#      newscan = [ 0 ]
#      while i < lenscan - 1 :
#	index = index + scan[i]
#        i = i + 1
#	j = j + 1
#        if j == k:
#	  newscan.append(index)
#	  j = 0
#	  i = i - 5
#	  index = 0
#	  count = count + 1
#          
#	
#      newlenscan = len(newscan)
#      newangles = [math.radians(-135 + i * 270/(newlenscan - 1)) for i in range(newlenscan)]


    #  print (lenscan, "start")
     # for i in range(lenscan):
	#print scan[i],
    #  print ("end")

#      indexformax = 0
#      for i in range(newlenscan):
#        if newscan[i] != max(newscan):
#	  indexformax = indexformax + 1
#        else:
#          break
#
#      indexforzeroangle = 0
#      for i in range(lenscan):
#        if angles[i] != 0:
#	  indexforzeroangle += 1
#	else:
#	  break
#      #we choose a linear model for angular speed
#      angular = 0.3 * newangles[indexformax] / math.radians(135)
#      # we choose an exponential model for linear speed
#      # if we are really close to an wall we want a very slow speed
#      if scan[indexforzeroangle] <= 1:
#        linear = 0.3 * (math.exp(scan[indexforzeroangle]) - 1) / (math.exp(1) - 1)
#      else:
#        linear = 0.3
#      for i in range(20):
#        if scan[i + indexforzeroangle - 10] <= 0.7:
#          linear = -0.3
#	  break
        
      


	

      ##########################################################################
      return [linear, angular]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):
 
      # Produce target if not existent
      if self.move_with_target == True and \
              self.navigation.target_exists == False:

        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        # Send the command
        self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_laser, a_laser] = self.produceSpeedsLaser()
      
      # You must fill these
      self.linear_velocity  = 0
      self.angular_velocity = 0
      
      if self.move_with_target == True:
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
        ############################### NOTE QUESTION ############################
        # You must combine the two sets of speeds. You can use motor schema,
        # subsumption of whatever suits your better.
	if (1 + l_laser / 200) > 0:
	  self.linear_velocity =  l_goal 
	  self.angular_velocity =  a_goal 
	else:
#	  l_laser = 1 + l_laser / 200

#          if l_laser > 0.3:
#	    l_laser = 0.3
#          elif l_laser < -0.3:
#	    l_laser = -0.3

#          if a_laser > 0.3:
#	    a_laser = 0.3
#          elif a_laser < -0.3:
#	    a_laser = -0.3

#	  self.linear_velocity = l_laser
#	  self.angular_velocity =  (2/3) * a_goal +  (1/3) * a_laser
	  k = 10000
	  l = 10000
	  self.linear_velocity  = l_goal + l_laser / k
	  self.angular_velocity = a_goal + a_laser / l
       
          if self.linear_velocity > 0.3:
            self.linear_velocity = 0.3
	  elif self.linear_velocity < -0.3:
	    self.linear_velocity = -0.3

          if self.angular_velocity > 0.3:
	    self.angular_velocity = 0.3
	  elif self.angular_velocity < -0.3:
	    self.angular_velocity = -0.3
	  #asdf = self.linear_velocity
	  #print asdf ##########################################################################
      else:
        ############################### NOTE QUESTION ############################
        # Implement obstacle avoidance here using the laser speeds.
        # Hint: Subtract them from something constant

      # we want a max of 0.3 m/s and 0.3 rad/s

     
        l_laser = 1 + l_laser / 200

        if l_laser > 0.3:
	  l_laser = 0.3
        elif l_laser < -0.3:
	  l_laser = -0.3

        if a_laser > 0.3:
	  a_laser = 0.3
        elif a_laser < -0.3:
	  a_laser = -0.3



	self.linear_velocity = l_laser
	self.angular_velocity = a_laser
        pass
        ##########################################################################

    # Assistive functions
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
