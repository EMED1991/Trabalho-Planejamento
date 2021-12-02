#!/usr/bin/env python

#importar os pacotes para desenvolvimento do projeto.
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import  *
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
import sys
import random

laser = LaserScan()


global freq # Frequencia de simulacao no stage
freq = 20.0  # Hz

global Usat # Velocidade de saturacao
Usat = 5

global x_n, y_n, theta_n # Estados do robo
x_n = 0.1  # posicao x atual do robo
y_n = 0.2  # posicao y atual do robo
theta_n = 0.001  # orientacao atual do robo

global x_goal, y_goal # Estados do robo
x_goal = 0
y_goal = 0

global d # Relativo ao feedback linearization
d = 0.80

global Kp # Relativo ao controlador (feedforward + ganho proporcional)
Kp = 1

global center # Selecao area de atuacao do laser.
center = 0
global righ
righ = 0

def laser_callback(msg): # Declara o sensor laser.
	global center
	global righ
	global laser
	laser = msg
	center = min(msg.ranges[75:195])
	righ = min(msg.ranges[0:75])
	
def callback_pose(data):# Rotina callback para a obtencao da pose do robo
    global x_n, y_n, theta_n
    x_n = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y_n = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta_n = euler[2]  # orientaco 'theta' do robo no mundo
    return

def refference_trajectory(time):# Rotina para a geracao da trajetoria de referencia
    global x_goal, y_goal #MUDAR PARA CIRCULO
    x_ref = x_goal
    y_ref = y_goal
    Vx_ref = 0
    Vy_ref = 0
    return (x_ref, y_ref, Vx_ref, Vy_ref)

def trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref):# Rotina para a geracao da entrada de controle
    global x_n, y_n, theta_n
    global Kp
    global Usat
    Ux = Vx_ref + Kp * (x_ref - x_n)
    Uy = Vy_ref + Kp * (y_ref - y_n)
    absU = sqrt(Ux ** 2 + Uy ** 2)
    if (absU > Usat):
        Ux = Usat * Ux / absU
        Uy = Usat * Uy / absU
    return (Ux, Uy)

def feedback_linearization(Ux, Uy):# Rotina feedback linearization
    global x_n, y_n, theta_n
    global d
    VX = cos(theta_n) * Ux + sin(theta_n) * Uy
    WZ = (-sin(theta_n) / d) * Ux + (cos(theta_n) / d) * Uy
    return (VX, WZ)

def example(): # Rotina primaria
    	global freq
    	global x_n, y_n, theta_n
   	global pub_rviz_ref, pub_rviz_pose
	global laser
	global center
	
	rospy.init_node("stage_controller_node",anonymous = False)
    	rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose) #declaracao do topico onde sera lido o estado do robo
	rospy.Subscriber("/base_scan",LaserScan, laser_callback)
	pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1) #declaracao do topico para comando de velocidade
   	vel = Twist()
    	i = 0
    
    #Inicializa os nos para enviar os marcadores para o rviz
    	pub_rviz_ref = rospy.Publisher("/visualization_marker_ref", Marker, queue_size=1) #rviz marcador de velocidade de referencia
    	pub_rviz_pose = rospy.Publisher("/visualization_marker_pose", Marker, queue_size=1) #rviz marcador de velocidade do robo
    #Define uma variavel que controlar[a a frequencia de execucao deste no
    	rate = rospy.Rate(freq)
    	sleep(0.2)
    # O programa do no consiste no codigo dentro deste while
   
 	while not rospy.is_shutdown(): #"Enquanto o programa nao ser assassinado"
			
        	i = i + 1 # Incrementa o tempo
        	time = i / float(freq)
     
		if (len(laser.ranges) > 0):
			if(center > 1 and righ > 1):
				[x_ref, y_ref, Vx_ref, Vy_ref] = refference_trajectory(time)
				[Ux, Uy] = trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref)
				[V_forward, w_z] = feedback_linearization(Ux, Uy)
				vel.linear.x = V_forward
				vel.angular.z = w_z
				rospy.loginfo("Navegando...")
			
			else:
				if(0.5 < center < 1):
					vel.linear.x = 0.0				
					vel.angular.z = 0.25
					rospy.loginfo("Girando...")
					
				if(center > 1 and righ < 1):
					vel.linear.x = 0.5
					vel.angular.z = 0.0
					rospy.loginfo("Proximo a parede...")

        	pub_stage.publish(vel)

        rate.sleep() #Espera por um tempo de forma a manter a frequencia desejada

# Funcao inicial
if __name__ == '__main__':
    x_goal = int(sys.argv[1])# Obtem os argumentos, no caso a posicao do alvo
    y_goal = int(sys.argv[2])
    try:
        	example()

    except rospy.ROSInterruptException:
        pass
