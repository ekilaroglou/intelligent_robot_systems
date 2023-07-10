import rospy
import math
import time
import sys

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation


selflazer_aggregator = LaserDataAggregator()
scan = selflazer_aggregator.laser_scan
for i in range(len(scan)):
    print scan[i],
