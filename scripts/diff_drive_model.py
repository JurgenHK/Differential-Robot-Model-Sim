#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

command = np.array([0,0,0] , dtype=np.float)
Jacobian = np.zeros(3)

#Topic names definition
nameLeftWheelTopic = "/left_wheel_speed/command"
nameRightWheelTopic = "/right_wheel_speed/command"

#Publisher creation
left_wheel_pub = rospy.Publisher(nameLeftWheelTopic, Float64, queue_size=10)
right_wheel_pub = rospy.Publisher(nameRightWheelTopic, Float64, queue_size=10)

#Modelo Robot

#Parametros generales
wheel_radius = 0.1
wheel_center_dist = 0.36/2

#Parametros rueda izq
alpha_1 = np.pi/2
beta_1 = 0
l_1 = wheel_center_dist 

#Parametros rueda der
alpha_2 = -1*np.pi/2
beta_2 = np.pi
l_2 = wheel_center_dist 

#Parametros rueda esferica
alpha_3 = 0
beta_3 = np.pi/2
l_3 = 0.3

#Matriz de restricciones
J1 = np.array( [( np.sin(alpha_1+beta_1), -np.cos(alpha_1+beta_1), -l_1*np.cos(beta_1)),
                ( np.sin(alpha_2+beta_2), -np.cos(alpha_2+beta_2), -l_2*np.cos(beta_2)),
                ( np.sin(alpha_3+beta_3), -np.cos(alpha_3+beta_3), -l_3*np.cos(beta_3))])
    
#Radios
J2=wheel_radius*np.identity(3)

#Jacobiano / Modelo cinematico
Jacobian = np.matmul(np.linalg.inv(J2),J1)

msgFloat_left = Float64()
msgFloat_right = Float64()

#Callback for message reception
def callback(data):
    global command
    global left_wheel_pub
    global right_wheel_pub
    global Jacobian
    global msgFloat_left 
    global msgFloat_right 
    #X,Y,Theta speeds
    command[0] = data.linear.x 
    command[1] = data.linear.y 
    command[2] = data.angular.z 
    
    result = np.matmul(Jacobian, command)

    # Left Wheel
    msgFloat_left = Float64()
    msgFloat_left.data = result[0]
    left_wheel_pub.publish(msgFloat_left)
        
    # Right Wheel
    msgFloat_right = Float64()
    msgFloat_right.data = result[1] 
    right_wheel_pub.publish(msgFloat_right)

    rospy.loginfo("Left speed: "+str(msgFloat_left)+" Right speed: "+str(msgFloat_right))

def drive():
    global msgFloat_left 
    global msgFloat_right 
    
    #ROS Initialization
    rospy.init_node('my_diff_drive', anonymous=True)
    rospy.loginfo("Node starting...")

    #Subscriber creation
    rospy.Subscriber("diff_speed", Twist, callback)

    rospy.loginfo("Differential drive init")
    rate = rospy.Rate(20) # 20 Hz publish rate

    while(not rospy.is_shutdown()):
        rospy.loginfo("Left speed: "+str(msgFloat_left)+" Right speed: "+str(msgFloat_right))
        rate.sleep()


if __name__ == '__main__':
    try:
        drive()
    except rospy.ROSInterruptException:
        pass


