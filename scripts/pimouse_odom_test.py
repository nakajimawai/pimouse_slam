#!/usr/bin/env python
#motors.py
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php

import sys, rospy, math, tf, signal
import pandas as pd
from pimouse_ros.msg import MotorFreqs
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point, Pose
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion
from nav_msgs.msg import Odometry

class Motor():
    def __init__(self):
        if not self.set_power(False): sys.exit(1)

        rospy.on_shutdown(self.set_power)
        self.sub_raw = rospy.Subscriber('motor_raw', MotorFreqs, self.callback_raw_freq)
        #self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)
        self.srv_on = rospy.Service('motor_on', Trigger, self.callback_on)
        self.srv_off = rospy.Service('motor_off', Trigger, self.callback_off)
        self.srv_tm = rospy.Service('timed_motion', TimedMotion, self.callback_tm)

        #Save path_plan flag
        self.save_path_as_csv = True

        #Cmd_vel receive flag (auto control)
        self.subscribe_cmd_vel = True

        #Initialize odometry header
        self.odom_header = Header()
        self.odom_header.seq = 0
        self.odom_header.stamp = rospy.Time.now()
        self.odom_header.frame_id = "map"

        # Initialize pose info
        self.sim_pose = Pose()
        self.sim_pose.position.x = 0.0
        self.sim_pose.position.y = 0.0
        self.sim_pose.position.z = 0.0
        self.sim_pose.orientation.x = 0.0
        self.sim_pose.orientation.y = 0.0
        self.sim_pose.orientation.z = 0.0

        # initialize twist info
        self.sim_twist = Twist()

        # Initialize odometry info
        self.sim_odom = Odometry()
        self.sim_odom.header = self.odom_header
        self.sim_odom.child_frame_id = "base_link"
        self.sim_odom.pose.pose = self.sim_pose
        self.sim_odom.twist.twist = self.sim_twist


        #initialize publisher
        self.emu_odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)

        #initialize TF
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.map_broadcaster = tf.TransformBroadcaster()

        #set callback for ctrl and c
        signal.signal(signal.SIGINT, self.ctr_c_interruption)

        if self.save_path_as_csv == True:
            self.path_dict = {}

        if self.subscribe_cmd_vel == True:
            self.cmdvel_sub = rospy.Subscriber("/cmd_vel", Twist, self.callback_cmd_vel)
            self.cmdvel_linear_x = 0.0
            self.cmdvel_linear_y = 0.0
            self.cmdvel_angular_z = 0.0



    def send_odom(self):
        '''
        self.cur_time = rospy.Time.now()

        dt = self.cur_time.to_sec() - self.last_time.to_sec()
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt 

        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((self.x,self.y,0.0), q, self.cur_time,"base_link","odom")

        odom = Odometry()
        odom.header.stamp = self.cur_time
        odom.header.frame_id = "odom"

        odom.pose.pose.position = Point(self.x,self.y,0)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom)

        self.last_time = self.cur_time
        '''

        #Update Vehicle Pose

        sampletime = 0.1    #calculate by 100msec
        e = tf.transformations.euler_from_quaternion((self.sim_pose.orientation.x, self.sim_pose.orientation.y, self.sim_pose.orientation.z, self.sim_pose.orientation.w))
        yaw_euler = e[2]


        #update pose from user request
        if self.subscribe_cmd_vel == False:
            self.sim_pose.position.x = self.sim_pose.position.x + self.sim_twist.linear.x*sampletime*math.cos(yaw_euler)
            self.sim_pose.position.y = self.sim_pose.position.y + self.sim_twist.linear.x*sampletime*math.sin(yaw_euler)
            updated_yaw = e[2] +self.sim_twist.angular.z*sampletime
        else:
            self.sim_pose.position.x = self.sim_pose.position.x + self.cmdvel_linear_x*sampletime*math.cos(yaw_euler)
            self.sim_pose.position.y = self.sim_pose.position.y + self.cmdvel_linear_x*sampletime*math.sin(yaw_euler)
            updated_yaw = e[2] + self.cmdvel_angular_z*sampletime

        updated_quaternion =tf.transformations.quaternion_from_euler(0, 0, updated_yaw)
        self.sim_pose.orientation.x = updated_quaternion[0]
        self.sim_pose.orientation.y = updated_quaternion[1]
        self.sim_pose.orientation.z = updated_quaternion[2]
        self.sim_pose.orientation.w = updated_quaternion[3]

        #update timestamp
        self.odom_header.seq =self.odom_header.seq + 1
        self.odom_header.stamp = rospy.Time.now()
        self.sim_odom.header = self.odom_header
        self.sim_odom.pose.pose = self.sim_pose
        self.sim_odom.twist.twist = self.sim_twist
        self.emu_odom_pub.publish(self.sim_odom)

        #update TF

        self.map_broadcaster.sendTransform(
            (self.sim_odom.pose.pose.position.x, self.sim_odom.pose.pose.position.y, self.sim_odom.pose.pose.position.z),
            updated_quaternion,
            rospy.Time.now(),
            "odom",
            "map"
        )

        base_link_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        self.odom_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            base_link_quat,
            rospy.Time.now(),
            # "base_link",
            "base_link",
            "odom"
        )

        #Save path_plan
        if self.save_path_as_csv == True:
            addRow = [0,self.sim_pose.position.x,self.sim_pose.position.y,0,updated_quaternion[0],updated_quaternion[1],updated_quaternion[2],updated_quaternion[3],
                      self.sim_twist.linear.x,self.sim_twist.linear.y,self.sim_twist.linear.z,0,0,self.sim_twist.angular.z]
            self.path_dict[len(self.path_dict)] = addRow

    ############
    # save csv #
    ############
    def save_csv(self):
        # Save CSV path file
        cols = ["time", "x", "y", "z", "w0", "w1", "w2", "w3", "vx", "vy", "vz", "roll", "pitch", "yaw"]
        df = pd.DataFrame.from_dict(self.path_dict, orient='index',columns=cols)
        df.to_csv("pimouse_path_data_0130_2.csv", index=False)


    def set_power(self,onoff=False):
        en = "/dev/rtmotoren0"
        try:
            with open(en,'w') as f:
                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
        except:
            rospy.logerr("cannot write to " + en)

        return False

    def set_raw_freq(self,left_hz,right_hz):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return

        try:
            with open("/dev/rtmotor_raw_l0",'w') as lf,\
                 open("/dev/rtmotor_raw_r0",'w') as rf:
                lf.write(str(int(round(left_hz))) + "\n")
                rf.write(str(int(round(right_hz))) + "\n")
        except:
            rospy.logerr("cannot write to rtmotor_raw_*")

    def callback_raw_freq(self,message):
        self.set_raw_freq(message.left_hz,message.right_hz)

    def callback_cmd_vel(self,message):
        if not self.is_on:
            return
        #self.cmdvel = message.linear.x
        #self.vth = message.angular.z
        self.cmdvel_linear_x =-message.linear.x*2
        self.cmdvel_linear_y =-message.linear.x*2
        self.cmdvel_angular_z =message.angular.z


        forward_hz = 80000.0*message.linear.x/(9*math.pi)
        rot_hz = 400.0*message.angular.z/math.pi
        self.set_raw_freq(forward_hz-rot_hz, forward_hz+rot_hz)

    def onoff_response(self,onoff):
        d = TriggerResponse()
        d.success = self.set_power(onoff)
        d.message = "ON" if self.is_on else "OFF"
        return d

    def callback_on(self,message): return self.onoff_response(True)
    def callback_off(self,message): return self.onoff_response(False)

    def callback_tm(self,message):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return False

        dev = "/dev/rtmotor0"
        try:
            with open(dev,'w') as f:
                f.write("%d %d %d\n" %
                    (message.left_hz,message.right_hz,message.duration_ms))
        except:
            rospy.logerr("cannot write to " + dev)
            return False

        return True


    #######################
    # ctrl and c callabck #
    #######################
    def ctr_c_interruption(self, signum, frame):
        self.save_csv()
        print("finish")
	sys.exit()

if __name__ == '__main__':
    rospy.init_node('motors')
    m = Motor()
    m.save_csv()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.send_odom()
        rate.sleep()
