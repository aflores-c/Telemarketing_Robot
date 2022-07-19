#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseActionGoal
import time
import numpy as np

LongitudTotal=0
position_x_anterior=0
position_y_anterior=0
tiempo_inicio_navegacion=0
LongitudTotalGlobalPlanner=0
cont=0

def callback(odometry_msg):
    global LongitudTotal
    global position_x_anterior
    global position_y_anterior
    
    position_x = odometry_msg.pose.pose.position.x
    position_y = odometry_msg.pose.pose.position.y

    if(position_x_anterior==0 and position_y_anterior==0):
        position_x_anterior=position_x
        position_y_anterior=position_y
    LongitudTotal += np.sqrt(np.power((position_x - position_x_anterior), 2) + np.power((position_y- position_y_anterior), 2))

    position_x_anterior = position_x
    position_y_anterior = position_y

def callback1(mensaje_goal):
    global tiempo_inicio_navegacion
    global LongitudTotal,cont
    tiempo_inicio_navegacion=time.time()
    LongitudTotal=0
    cont=0

def callback2(mensaje_llegada):
    tiempo_navegacion=time.time()-tiempo_inicio_navegacion
    print("Tiempo total de navegacion: "+str(tiempo_navegacion)+" segundos")
    print("Recorrido total de navegacion: "+str(LongitudTotal)+" metros")
    
    
def callback3(path_msg):
    global cont
    global LongitudTotalGlobalPlanner
    path_length=0
    for i in range(len(path_msg.poses) - 1):
        position_a_x = path_msg.poses[i].pose.position.x
        position_b_x = path_msg.poses[i+1].pose.position.x
        position_a_y = path_msg.poses[i].pose.position.y
        position_b_y = path_msg.poses[i+1].pose.position.y

        path_length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2))
    if cont==0:
        LongitudTotalGlobalPlanner=path_length
    cont+=1
rospy.init_node('suscriptor')
rospy.Subscriber("/odom", Odometry, callback)
rospy.Subscriber("move_base/goal", MoveBaseActionGoal, callback1)
rospy.Subscriber("move_base/result", MoveBaseActionResult, callback2)
rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, callback3)

rospy.spin()