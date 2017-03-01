#!/usr/bin/env python

import csv
import roslib
import rospy
import actionlib
import time
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry 
from random import sample
from math import pow, sqrt
from LatLongUTMconversion import LLtoUTM, UTMtoLL



global nav #global variable to store our navigation class

def update_current_pose(current_pose): #updates the current pose in our global variable
    try:
        nav.current_pose = current_pose
    except NameError:
        print "Still booting..."

def update_utm(current_latlong): #updates lat, long, and UTM in our global variable
    try:
        nav.current_utm = LLtoUTM(23, current_latlong.latitude,current_latlong.longitude)
        nav.curLat=current_latlong.latitude        
        nav.curLong=current_latlong.longitude
    except NameError:
        print "Still booting..."

class Navigation (): #navigation class that stores iggy's position upon initialization and has functions to take a list of waypoints and turn them into goals for Iggy
    def __init__(self):     
        rospy.init_node('nav_test', anonymous=True)        
        rospy.on_shutdown(self.shutdown)    

        # Subscribe to the move_base action server
    	self.goal = MoveBaseGoal() 
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)        
        rospy.loginfo("Waiting for move_base action server...")        

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))        
        rospy.loginfo("Connected to move base server")        
           
        # Publisher to manually control the robots movement (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        self.curLat=0.0
        self.curLong=0.0

        # A variable to hold the initial and current pose of the robot
        self.current_pose = Odometry()
        self.initial_pose = Odometry()
        self.current_utm = [] # return (UTMZone, UTMEasting, UTMNorthing)
        self.initial_utm = [] # return (UTMZone, UTMEasting, UTMNorthing)

        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']       

        #Subscribes to the gps and odometry
        rospy.Subscriber('/navsat/fix', NavSatFix, update_utm)
        rospy.Subscriber('/odometry/gps', Odometry, update_current_pose)
    
    #calculates distance from goal based off of odometry
    def calculateOdomDist(self, currX, currY, lastX, lastY, isFirst): #gets distance from the current position to another position
        if isFirst!= True:
            #if this is the first time it's simply the distance from the last position to the current position
            distance = sqrt(pow(currX - lastX, 2) + pow(currY - lastY, 2))             
        else:        
            #otherwise we use the pose
            distance =  sqrt(pow(currX - self.initial_pose.pose.pose.position.x, 2) + pow(currY - self.initial_pose.pose.pose.position.y, 2))
        return distance

    #calculates dist from goal based off of GPS coordinates
    def calculateGPSDistFromGoal(self, curGoalLat, curGoalLong, resFile):
        distance= sqrt(pow(self.curLat -curGoalLat,2) + pow(self.curLong - curGoalLong, 2)) *100000
        rospy.loginfo("Distance from goal according to the GPS: " + str(distance) + " meters \n" )
        return distance 
    
    #takes in a file and writes information to it as IGGY navigates to goals, makes data analysis easier
    def infoReport(self, resultsFile):
            rospy.loginfo("Going to: (%.4f,%.4f)" %(self.goal.target_pose.pose.position.x,self.goal.target_pose.pose.position.y))
            resultsFile.write("Going to: (%.4f,%.4f)" %(self.goal.target_pose.pose.position.x,self.goal.target_pose.pose.position.y) + "\n")
            rospy.loginfo("Current lat is: " + str(self.curLat))
            resultsFile.write("Current lat is: " + str(self.curLat) + "\n")
            rospy.loginfo("Goal lat is: " + str(self.goalLat))            
            resultsFile.write("Goal lat is: " + str(self.goalLat) + "\n")            
            rospy.loginfo("Current long is: " + str(self.curLong))
            a = self.curLong            
            resultsFile.write("Current long is: " + str(a) + "\n")
            rospy.loginfo("Goal long is: " + str(self.goalLong))
            resultsFile.write("Goal long is: " + str(self.goalLong) + "\n")

    def successReport(self,n_goals,n_successes,running_time, distance_traveled, results):
        rospy.loginfo("Success so far: " + str(n_successes) + "/" + str(n_goals) + " = " + str(100 * n_successes/n_goals) + "%")
        results.write("Success so far: " + str(n_successes) + "/" + str(n_goals) + " = " + str(100 * n_successes/n_goals) + "% \n")
        rospy.loginfo("Running time: %.2f min Distance: %.3f"  % (running_time,distance_traveled))
        results.write("Running time: %.2f min Distance: %.3f"  % (running_time,distance_traveled) +"\n")

    #meat of the class; allows Iggy to navigate to a series of waypoints based off of what we put in a .csv file
    def navigate(self):
        #Get our .csv file with our waypoints to navigate to. Change the name based off what waypoints you want to go to
        filename = "waypoints.csv"
        wps = self.loadWaypoints(filename)

        # Creates a file to write our results to
        results= open('testResults3.csv', 'w')   

        #variables for the number of goals, successes and distance traveled for stats tracking
        n_goals = 0
        n_successes = 0
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0

        # How long in seconds should the robot pause at each location?
        rest_time = rospy.get_param("~rest_time", 3)
        self.setInitialPose() 
        goals=self.makeWaypointsIntoGoals(wps)
        latLongs=self.makeLatLongList(wps)

        # Begin the main loop and run through a sequence of locations
        i=0        
        firstRun=True
        while not rospy.is_shutdown():                        
            # Keep track of the distance traveled.
            cur_coord = goals[i][0]
            last_coord = goals[i-1][0]
            distance = self.calculateOdomDist(cur_coord.position.x, cur_coord.position.y, last_coord.position.x, last_coord.position.x, firstRun)
                     
            # Set up the next goal location
            self.goal.target_pose.pose = cur_coord
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.orientation = self.initial_pose.pose.pose.orientation
            self.goalLat= latLongs[i][0]
            self.goalLong= latLongs[i][1]

            # Let the user know where the robot is going next
            self.infoReport(results)
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
            
            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            
            # Check for success or failure
            if not finished_within_time: 
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
                results.write("Timed out achieving goal \n")
            else:
                #Success state, get ready for next goal
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    results.write("Goal succeeded! \n")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                    results.write("State: " + str(state) + "\n")
                    newD= self.calculateGPSDistFromGoal(self.goalLat, self.goalLong, results)
                    results.write("Distance from goal according to the GPS: " + str(distance) + " meters" )
                #Failure state
                else:
                  rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))
                  results.write("Goal failed with error code: " + str(self.goal_states[state]) + "\n")
            
            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0
            
            # Print a summary success/failure, distance traveled and time elapsed
            n_goals += 1
            self.successReport(n_goals,n_successes,running_time, distance_traveled, results)

            # Increment the counters
            i += 1

            # end if we've visited all goals
            if (i >= len(goals)):
                rospy.signal_shutdown("NO MORE GOALS TO ACHIEVE")
                results.write("NO MORE GOALS TO ACHIEVE \n")
                results.close()
            rospy.sleep(rest_time) 

    def setInitialPose(self):            
        # Make sure we have the initial pose
        rospy.loginfo("Waiting for initial pose")

        # Get the initial pose from the robot
        rospy.wait_for_message('/odometry/gps', Odometry)
        rospy.loginfo("Initial Pose from /odometry/gps recieved")

    #will get stuck if the current_utm isn't recieving any variables
	while len(self.current_utm) != 3:
            time.sleep(1) 
            rospy.loginfo("Stuck?") #was initially getting stuck in this loop, leaving this for testing purposes
        rospy.loginfo("Establishing initial position wait 10 seconds.")	
	easts=[]
	nrths=[]

    #getting the initial northings, eastings, and pose and adding them to our current_utm variable to keep track of them
	for i in range(1):
       	    utm_c=self.current_utm
	    easts.append(utm_c[1])
	    nrths.append(utm_c[2])
	    rospy.sleep(1.5)
	easting = sum(easts) / float(len(easts))
	northing = sum(nrths) / float(len(nrths))
        self.initial_pose = self.current_pose 
        self.initial_utm = [utm_c[0],easting,northing]
        rospy.loginfo("Initial pose found at (%.4f,%.4f)" %(self.initial_pose.pose.pose.position.x,self.initial_pose.pose.pose.position.y))               
	rospy.loginfo("Initial UTM at (%.4f, %.4f)" % (easting, northing) )

    # Turn list of waypoints into Goals. Goals are coordinates in the robot odom frame.


    def makeLatLongList(self,wps):
        latLongs= []
        for waypointLat,waypointLong,search_duration,rest_duration in wps:
            latLongs.append((waypointLat, waypointLong))
        return latLongs
    #function that takes a series of waypoints from a .csv file and makes them into goal coordinates in the robot odom frame

    def makeWaypointsIntoGoals(self, wps):
        log_directory = rospy.get_param("~log_directory", "~/")+"waypoints2Goals_log.csv"
        # Load and parse waypoints from file.
        goals=[] #list to keep track of goal waypoints
        for waypointLat,waypointLong,search_duration,rest_duration in wps:
            goal_pose=Pose()
            (wpZone,wpEasting,wpNorthing)=LLtoUTM(23, waypointLat,waypointLong)
            goal_pose.position.x=(wpEasting - self.initial_utm[1]) # REP103 says x is east and y is north
            goal_pose.position.y=(wpNorthing - self.initial_utm[2])
            goals.append((goal_pose, search_duration, rest_duration))
        return goals
        

    # Read from file the list of Lat,Long,Duration,Tolerance and put into a list of waypoints
    def loadWaypoints(self, filename):
        waypoints=[]
        with open(filename, mode='r') as infile:
            reader = csv.reader(infile)
            for row in reader:
                if row[0][0] != '#': # ignore commented out waypoints
                    waypoints.append((float(row[0]),float(row[1]),float(row[2]),float(row[3])))
        return waypoints
    
    #updates the global variable for pose
    def update_current_pose(self, current_pose):
        rospy.loginfo("Current Pose is: (%.4f,%.4f)" %((current_pose.pose.pose.position.x),(current_pose.pose.pose.position.y)))        
        self.current_pose = current_pose

    #updates the global variable for utm
    def update_utm(self, current_latlong):
        rospy.loginfo("Current Lat, Long is: (%f,%f)" %((current_latlong.latitude),(current_latlong.longitude)))        
        self.current_utm = LLtoUTM(23, current_latlong.latitude,current_latlong.longitude)

    #stops the ropot and shuts the program down
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    global nav
    try:
        nav = Navigation ()
        nav.navigate()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

