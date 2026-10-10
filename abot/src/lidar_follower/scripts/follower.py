#!/usr/bin/env python

import rospy
import time 
import numpy as np
from geometry_msgs.msg import Twist, Vector3
#from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

class Follower:
    def __init__(self):
        self.max_speed = rospy.get_param('~maxSpeed') 
        self.active=False
        self.target_dist = 1.0
        self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size =3)
        #self.joySubscriber = rospy.Subscriber('/joy', Joy, self.buttonCallback)
        self.positionSubscriber = rospy.Subscriber('/object_tracker/target_position', Point, self.positionCallback)
        PID_param = rospy.get_param('~PID_controller')
        self.PID_controller = simplePID([0, self.target_dist], PID_param['P'], PID_param['I'], PID_param['D'])
        rospy.on_shutdown(self.shutdown)

    def positionCallback(self, point):
	if(not(self.active)):
	    return #if we are not active we will return imediatly without doing anything

        angleX = point.x
        angleY = point.y
        distance = point.z
        [uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])

        angularSpeed = np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed)
        linearSpeed = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed)

        velocity = Twist()
        velocity.linear = Vector3(linearSpeed, 0, 0)
        velocity.angular = Vector3(0, 0, angularSpeed)
        rospy.loginfo('linearSpeed:{}   angularSpeed: {}'.format(linearSpeed, angularSpeed))
        self.cmdVelPublisher.publish(velocity)
    
    def stopMoving(self):
        velocity = Twist()
        velocity.linear = Vector3(0., 0., 0.)
        velocity.angular = Vector3(0., 0., 0.)
        self.cmdVelPublisher.publish(velocity)
    
    def shutdown(self):
        self.stopMoving()
        self.active = False
        rospy.loginfo('stop moving')
    
class simplePID:
    def __init__(self, target, P, I, D):
        if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
	        raise TypeError('input parameters shape is not compatable')   
        rospy.loginfo('PID initialised with P:{}, I:{}, D:{}'.format(P,I,D))    
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.setPoint = np.array(target)
        self.last_error = 0
        self.integrator = 0
        self.integrator_max = float('inf')
        self.timeOfLastCall = None
    
    def update(self, current_value):
        current_value = np.array(current_value)
        if(np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if(self.timeOfLastCall is None):
			# the PID was called for the first time. we don't know the deltaT yet
			# no controll signal is applied
			self.timeOfLastCall = time.clock()
			return np.zeros(np.size(current_value))	 
        error = self.setPoint - current_value
        P = error
        current_time = time.clock()
        deltaT = (current_time - self.timeOfLastCall)
        self.integrator = self.integrator + (error*deltaT)
        I = self.integrator
        D = (error - self.last_error)/deltaT
        self.last_error = error
        self.timeOfLastCall = current_time
        return self.Kp*P + self.Ki*I + self.Kd*D  

if __name__ == '__main__':
    print('starting')
    rospy.init_node('follower')
    follower = Follower()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print('exception')     


