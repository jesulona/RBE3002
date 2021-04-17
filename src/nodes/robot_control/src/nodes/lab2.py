#!/usr/bin/env python2

import rospy, math, time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion

class Lab2:

    #initialize Pose properties
    px = 0
    py = 0
    pth = 0

    ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
    global pubTwist
    pubTwist = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2',anonymous=True)

        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        subOdom = rospy.Subscriber('/odom',Odometry,self.update_odometry)

        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        subMove = rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.go_to)
    
        rospy.sleep(1.0)
      



    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """

        ### Make and populate a new Twist messagee
        TwistMsg = Twist()
        lV = TwistMsg.linear
        aV = TwistMsg.angular
        lV.x = linear_speed
        lV.y = 0
        lV.z = 0
        aV.x = 0
        aV.y = 0
        aV.z = angular_speed

        ### Publish the message
        #while not rospy.is_shutdown():
        rospy.loginfo(TwistMsg)
        pubTwist.publish(TwistMsg)
        


    
        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """

        #Store initial pose 
        xStart = self.px
        yStart = self.py

        reachedDest = False 

        while(not reachedDest and not rospy.is_shutdown()):
            #Track distrance travelled
            xCur = self.px
            yCur = self.py
            dX = xStart - xCur
            dY = yStart - yCur
            dU = math.sqrt(pow(dX,2) + pow(dY,2))
            self.send_speed(linear_speed,0)

            if (dU >= distance):
                reachedDest = True
                self.send_speed(0,0)

 


    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        
        #Store initial pose 
        startAng = self.pth
        AtT = angle - startAng
        reachedDest = False

        while(not reachedDest and not rospy.is_shutdown()):
            angCur = self.pth
            angLeft = angle - angCur
            print(angLeft)
            if(angLeft > 0):
                self.send_speed(0,aspeed)
            else:
                self.send_speed(0,-aspeed)
            
            if (abs(angLeft) < 0.008):
                reachedDest = True
                self.send_speed(0,0)





    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        #Extract final pose information
        xDest = msg.pose.position.x
        yDest = msg.pose.position.y
        quat_orig = msg.pose.orientation 
        quat_list = [quat_orig.x,quat_orig.y,quat_orig.z,quat_orig.w] 
        (roll,pitch,yaw) = euler_from_quaternion(quat_list)
        destPth = yaw

        #Record current pose
        xCur = self.px
        yCur = self.py
        curPth = self.pth
        
        #Calculate distance to final pose
        dX = xDest - xCur
        dY = yDest - yCur

        #Calculate initial turn angle and distance
        angToDest = math.atan2(dY,dX)
        DtD = math.sqrt(pow(dX,2) + pow(dY,2))

        #Execute plan
        self.rotate(angToDest,0.75)
        self.drive(DtD,0.20)
        self.rotate(destPth,0.75)




    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        #Assign values to Pose attributes
        self.px = msg.pose.pose.position.x 
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation 
        quat_list = [quat_orig.x,quat_orig.y,quat_orig.z,quat_orig.w] 
        (roll,pitch,yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw
 

    def diff_drive(self, vL, vR, spinTime):
        #Helper function for Extra Credit
        wB = 0.16
        v = 0.5*(vL + vR)
        w = (vR-vL)/wB
        start = time.time()
        while(time.time() - start < spinTime):
            self.send_speed(v,w)


    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """

        #Extract final pose information
        xDest = position.pose.position.x
        yDest = position.pose.position.y
        quat_orig = position.pose.orientation 
        quat_list = [quat_orig.x,quat_orig.y,quat_orig.z,quat_orig.w] 
        (roll,pitch,yaw) = euler_from_quaternion(quat_list)
        destPth = yaw

        #Record current pose
        xStart = self.px
        yStart = self.py
        curPth = self.pth
        
        #Calculate distance to final pose
        dX = xDest - xStart
        dY = yDest - yStart
        dUt = math.sqrt(pow(dX,2) + pow(dY,2))

        #intersecting chord theorem 
        h = 0.75
        l = dUt
        r = (4*pow(h,2) + pow(l,2))/8*h
        #r = (pow((l/2),2) + pow(h,2))/2*h
        #r = h/2 + (pow(l,2)/8*h)
        v = 0.1
        w = v/r

        #Calculate initial turn angle and distance
        angToDest = math.atan2(dY,dX)
        if(angToDest > 90*math.pi/180):
            w = -w


        reachedDest = False

        while(not reachedDest and not rospy.is_shutdown()): 
            self.send_speed(v,w)   
            print(r)        
            curPth = self.pth
            angVel = (destPth - curPth)


            # if(abs(angVel) < 0.03):
            #     self.send_speed(v,0)
            # else:
            #     self.send_speed(v,w)
        
            angLeft = destPth - curPth

           #Track distrance travelled
            xCur = self.px
            yCur = self.py
            dX = xStart - xCur
            dY = yStart - yCur
            dU = math.sqrt(pow(dX,2) + pow(dY,2))

            if (abs(angLeft) < 0.008 and  (dU >= dUt) ):
                reachedDest = True
                self.send_speed(0,0)



    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    Lab2().run()
