#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS                SUBSCRIPTIONS
/edrone/drone_command           /edrone/gps
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu,NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf
import math


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        #self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        #self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        #self.setpoint_cmd = [1500.0, 1500.0, 1500.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        # self.setpoint_euler = [0.0, 0.0, 0.0]
        # self.setpoint_throttle = 0
        # self.pwm_cmd = prop_speed()
        # self.pwm_cmd.prop1 = 512.0
        # self.pwm_cmd.prop2 = 512.0
        # self.pwm_cmd.prop3 = 512.0
        # self.pwm_cmd.prop4 = 512.0

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corressponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [2860*0.06,1467*0.06, 1.12 ]
        self.Ki = [0.0 , 0.0, 0.000075]
        self.Kd = [5000*100 , 5000*100,1351.5]
        self.error = [0.0,0.0,0.0]  #errors in each axis
        self.prev_error = [0.0, 0.0, 0.0] #previous errors in each axis
      #  self.max_values = [256, 256, 256, 256]  #max values
      #  self.min_values = [0, 0, 0, 0]              #min values
        self.fix_lat = 19.0
        self.fix_lon = 72.0
        self.fix_alt=  3.0
        self.out_roll = 0.0
        self.out_pitch = 0.0
        self.out_throttle = 0.0
        self.errSum = [0.0, 0.0, 0.0]
        self.dErr = [0.0, 0.0, 0.0]
        self.prev_lat=0.0
        self.prev_lon=0.0
        self.prev_alt=0.0
        self.drone_cmd=edrone_cmd()
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.loop_for_top=1
        self.loop_for_right=1
        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 30  # in mseconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.drone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
      #   rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
      #   rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
      # #  rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        #rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        # rospy.Subscriber('/pid_tuning_altitude', PidTune, self.throttle_set_pid)

        rospy.Subscriber('/edrone/gps', NavSatFix , self.gps_set_pid)
        
    def gps_set_pid(self,msg):
        self.alt=msg.altitude
        self.lat=msg.latitude
        self.lon=msg.longitude
    # Imu callback function

    # def imu_callback(self, msg):

    #     self.drone_orientation_quaternion[0] = msg.orientation.x
    #     self.drone_orientation_quaternion[1] = msg.orientation.y
    #     self.drone_orientation_quaternion[2] = msg.orientation.z
    #     self.drone_orientation_quaternion[3] = msg.orientation.w

    # def drone_command_callback(self, msg):
    #     self.setpoint_cmd[0] = msg.rcRoll
    #     self.setpoint_cmd[1] = msg.rcYaw
    #     self.setpoint_cmd[2] = msg.rcPitch
    #     #self.setpoint_cmd[3] = msg.rcThrottle

    # # Callback function for /pid_tuning_roll
    # # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def roll_set_pid(self, roll):
        self.Kp[1] = roll.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = roll.Ki * 0.0008
        self.Kd[1] = roll.Kd * 100
    
    def pitch_set_pid(self, pitch):
        self.Kp[0] = pitch.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = pitch.Ki * 0.0008
        self.Kd[0] = pitch.Kd * 100
    
    # def throttle_set_pid(self, throttle):
    #     self.Kp[2] = throttle.Kp * 0.1  # This is just for an example. You can change the ratio/fraction value accordingly
    #     self.Ki[2] = throttle.Ki * 0.008
    #     self.Kd[2] = throttle.Kd * 20
    #     print(Kp[2],"booo")

    def pid(self):
      #   # Converting quaternion to euler angles
      #   (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
      #  # print(self.drone_orientation_euler[0])
      #   self.drone_orientation_euler[0]=math.degrees(self.drone_orientation_euler[0])
      #   self.drone_orientation_euler[1]=math.degrees(self.drone_orientation_euler[1])
      #   self.drone_orientation_euler[2]=math.degrees(self.drone_orientation_euler[2])
      # #  self.error[0] = 0 - self.drone_orientation_euler[0]
      # #  print(self.error[0])
      # #  self.pwm_pub.publish(self.pwm_cmd)
        
      # #  Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
      #   self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
      #   self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
      #   self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30

        #self.setpoint_throttle = self.setpoint_cmd[3] * 1.024 - 1024

        self.error[0] = (self.fix_lat-self.lat)*100000
        self.error[1] = (self.fix_lon-self.lon)*100000
        self.error[2] = self.fix_alt - self.alt
        print("Alt error is ", self.error[2])
        print(self.Kp[1],self.error[0],self.error[1])

        
        if self.loop_for_top==0:
            if self.loop_for_right:
                print("came inside")
                if abs(self.error[0]/100000)<0.000004517:
                    self.fix_alt=0.3
                    print("Reached here in right")
                    self.loop_for_right=0
        
        if self.loop_for_top:
            print("In top loop")
            if abs(self.error[2]<0.2):
                self.fix_lat=19.0000451704
                print("Reached here in top")
                self.loop_for_top=0

      #  self.error[3] = self.setpoint_throttle - self.drone_orientation_euler[2]
        #Compute all the working error variables
        self.errSum[0] = self.errSum[0] + (self.error[0] * self.sample_time)
        self.dErr[0] = (self.error[0] - self.prev_error[0]) / self.sample_time

        self.errSum[1] = self.errSum[1] + (self.error[1] * self.sample_time)
        self.dErr[1] = (self.error[1] - self.prev_error[1]) / self.sample_time

        self.errSum[2] = self.errSum[2] + (self.error[2] * self.sample_time)
        self.dErr[2] = (self.error[2] - self.prev_error[2]) / self.sample_time

        self.out_pitch = 1500 + (self.Kp[0] * self.error[1] + self.Ki[0] * self.errSum[1] + self.Kd[0] * self.dErr[1])
        
        self.out_roll =  1500 +  (self.Kp[1] * self.error[0] + self.Ki[1] * self.errSum[0] + self.Kd[1] * self.dErr[0])

        self.out_throttle = self.Kp[2] * self.error[2] + self.Ki[2] * self.errSum[2] + self.Kd[2] * self.dErr[2]

        self.prev_error[0]= self.error[0]
        self.prev_error[1]= self.error[1]
        self.prev_error[2]= self.error[2]

        print(self.lat,self.lon,self.out_roll)                                                                                                                                                                                                              
        
        if self.out_roll > 2000:
            self.out_roll= 2000
        
        if self.out_pitch > 2000:
            self.out_pitch = 2000

        if self.out_throttle > 2000:
            self.out_throttle = 2000
        
        if self.out_roll < 1000:
            self.out_roll= 1000
        
        if self.out_pitch < 1000:
            self.out_pitch = 1000

        # if self.out_throttle < 1000:
        #     self.out_throttle = 1000

        self.drone_cmd.rcYaw = 1500.0
        self.drone_cmd.rcRoll=self.out_roll
        self.drone_cmd.rcPitch=1500.0
        self.drone_cmd.rcThrottle=self.out_throttle
        print("Latest throttle is ",self.out_throttle)
        
        self.drone_pub.publish(self.drone_cmd)
        self.roll_pub.publish(self.error[1])
        self.pitch_pub.publish(self.error[0])
        
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()