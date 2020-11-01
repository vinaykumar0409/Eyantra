#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude--for throttle values of gui
        /pitch_error            /pid_tuning_pitch--
        /yaw_error              /pid_tuning_roll--roll wala
        /edrone/pwm             /edrone/imu/data---this is basically on which attitude controller will accept inputs -- basically like a feedback
                                /edrone/drone_command --this is where we give the inputs as in where to go
                                standard servo message format ie. 1000 - 2000
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries
#BHAVYA

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf   #basically transforms
import math

class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [0.0, 0.0, 0.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        # Hint: To see the message structure of prop_speed type the following command in the terminal
        # rosmsg show vitarana_drone/prop_speed
        # jash@jashubuntu:~/catkin_ws$ rosmsg show vitarana_drone/prop_speed 
        # float64 prop1
        # float64 prop2
        # float64 prop3
        # float64 prop4

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [0, 0, 0]
        self.Ki = [0, 0, 0]
        self.Kd = [0, 0, 0]
        # -----------------------Add other required variables for pid here ----------------------------------------------
        #

        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
        #        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
        #                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
        #
        # ----------------------------------------------------------------------------------------------------------
        #jash
        self.prev_error = [0,0,0]
        self.max_values = [1024, 1024, 1024, 1024]
        self.min_values = [0, 0, 0, 0]
        #jash
        self.error_sum=[0,0,0]
        self.out_roll=0
        self.out_pitch=0
        self.out_yaw=0
        self.out_throt=0
        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 30  # in seconds
        #self.last_time=time.time()

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
 
        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        self.roll_error = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_error = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.yaw_error = rospy.Publisher('/yaw_error', Float32, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        

        # ------------------------------------------------------------------------------------------------------------

    # Imu callback function
    # The function gets executed each time when imu publishes /edrone/imu/data

    # Note: The imu publishes various kind of data viz angular velocity, linear acceleration, magnetometer reading (if present),
    # but here we are interested in the orientation which can be calculated by a complex algorithm called filtering which is not in the scope of this task,
    # so for your ease, we have the orientation published directly BUT in quaternion format and not in euler angles.
    # We need to convert the quaternion format to euler angles format to understand the orienataion of the edrone in an easy manner.
    # Hint: To know the message structure of sensor_msgs/Imu, execute the following command in the terminal
    # rosmsg show sensor_msgs/Imu

    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w
        
        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.out_throt=1.024*msg.rcThrottle-1024
        # ----------------- ----------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 0.008*self.sample_time
        self.Kd[0] = roll.Kd * 0.3/self.sample_time
        #self.Kp[0] = roll.Kp   # This is just for an example. You can change the ratio/fraction value accordingly
        #self.Ki[0] = roll.Ki 
        #self.Kd[0] = roll.Kd         
        print("Kp is ",self.Kp[0])
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki * 0.008*self.sample_time
        self.Kd[1] = pitch.Kd * 0.3/self.sample_time

    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = yaw.Ki * 0.008*self.sample_time
        self.Kd[2] = yaw.Kd * 0.3/self.sample_time
    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        #   1. Convert the quaternion format of orientation to euler angles
        #   2. Convert the setpoin that is in the range of 1000 to 2000 into angles with the limit from -10 degree to 10 degree in euler angles
        #   3. Compute error in each axis. eg: error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0], where error[0] corresponds to error in roll...
        #   4. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        #   5. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        #   6. Use this computed output value in the equations to compute the pwm for each propeller. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        #   7. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        #   8. Limit the output value and the final command value between the maximum(0) and minimum(1024)range before publishing. For eg : if self.pwm_cmd.prop1 > self.max_values[1]:
        #                                                                                                                                      self.pwm_cmd.prop1 = self.max_values[1]
        #   8. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        #   9. Add error_sum to use for integral component
        #now=time.time()
        #timeChange = (now - self.last_time)
        #if(timeChange>=self.sample_time):
        # Converting quaternion to euler angles
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
        self.drone_orientation_euler[0]=math.degrees(self.drone_orientation_euler[0])
        self.drone_orientation_euler[1]=math.degrees(self.drone_orientation_euler[1])
        self.drone_orientation_euler[2]=math.degrees(self.drone_orientation_euler[2])
        print("Degree as follows") 
        print(self.drone_orientation_euler[0])
        print(self.drone_orientation_euler[1])
        print(self.drone_orientation_euler[2])
		#print(" Degree done ")        
		# Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis

        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30
        # Complete the equations for pitch and yaw axis
        error=[0,0,0]
        error_prop=[0,0,0]
        error_der=[0,0,0]
        error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0]
        error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1]
        error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]
        
        error_prop[0]=error[0]
        error_der[0]=error[0]-self.prev_error[0]
        self.error_sum[0]+=error[0]
        
        error_prop[1]=error[1]
        error_der[1]=error[1]-self.prev_error[1]
        self.error_sum[1]+=error[1]

        error_prop[2]=error[2]
        error_der[2]=error[2]-self.prev_error[2]
        self.error_sum[2]+=error[2]

        #############################################
        self.out_roll=self.Kp[0]*error_prop[0]+self.Ki[0]*self.error_sum[0]+self.Kd[0]*error_der[0]
        self.out_pitch=self.Kp[1]*error_prop[1]+self.Ki[1]*self.error_sum[1]+self.Kd[1]*error_der[1]
        self.out_yaw=self.Kp[2]*error_prop[2]+self.Ki[2]*self.error_sum[2]+self.Kd[2]*error_der[2]
        print("Out_Roll is " , self.out_roll)
        
        # Also convert the range of 1000 to 2000 to 0 to 1024 for throttle here itslef
       

        #
        #print(self.out_throt)
        self.pwm_cmd.prop1 = self.out_throt+self.out_roll+self.out_pitch+self.out_yaw
        self.pwm_cmd.prop2 = self.out_throt+self.out_roll-self.out_pitch-self.out_yaw
        self.pwm_cmd.prop3 = self.out_throt-self.out_roll-self.out_pitch+self.out_yaw
        self.pwm_cmd.prop4 = self.out_throt-self.out_roll+self.out_pitch-self.out_yaw
  
        self.prev_error[0] = error[0]
        self.prev_error[1] = error[1]
        self.prev_error[2] = error[2]
        #self.last_time=now
        if self.pwm_cmd.prop1 > self.max_values[0]:                                                                                                                               
            self.pwm_cmd.prop1 = self.max_values[0]

        if self.pwm_cmd.prop2 > self.max_values[1]:                                                                                                                               
            self.pwm_cmd.prop2 = self.max_values[1]

        if self.pwm_cmd.prop3 > self.max_values[2]:                                                                                                                               
            self.pwm_cmd.prop3 = self.max_values[2]

        if self.pwm_cmd.prop4 > self.max_values[3]:                                                                                                                               
            self.pwm_cmd.prop4 = self.max_values[3]
        #for minumum vLUES
        if self.pwm_cmd.prop1 < self.min_values[0]:                                                                                                                               
            self.pwm_cmd.prop1 = self.min_values[0]

        if self.pwm_cmd.prop2 < self.min_values[1]:                                                                                                                               
            self.pwm_cmd.prop2 = self.min_values[1]

        if self.pwm_cmd.prop3 < self.min_values[2]:                                                                                                                               
            self.pwm_cmd.prop3 = self.min_values[2]

        if self.pwm_cmd.prop4 < self.min_values[3]:                                                                                                                               
            self.pwm_cmd.prop4 = self.min_values[3]

        print("!st is ")
        print(self.pwm_cmd.prop1)
        print(self.pwm_cmd.prop2)
        print(self.pwm_cmd.prop3)
        print(self.pwm_cmd.prop4)
        print("Done")
        
        self.pwm_pub.publish(self.pwm_cmd)
        self.roll_error.publish(error[0])
        self.yaw_error.publish(error[2])
        self.pitch_error.publish(error[1])
        #self.out_throt=0


if __name__ == '__main__':
    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
