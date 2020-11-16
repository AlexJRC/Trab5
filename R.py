import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math

V_Mat = [2018000309, 2016006869, 2017009838, 34219,  2017003253]


kp_ang = 0.01
ki_ang = 0.0001
kd_ang = 0.01

kp = 1
ki = 0.01
kd = 1

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------

#FUNCAO PARA ACHAR A FREQUENCIA DO TIMER

def loop_timer_CallBack (V_Mat): 
    global mat
    n = len(V_Mat)
    res_each = 0
    m = 0
    f = 0
    t = 0
	
    for mat in V_Mat:
        res_each = 0
        for x in str (mat):
            res_each = res_each + int(x)
            m = m+res_each
    m = m/n
    f = m
    t = 1/f
    return t
    
tempo_loop = loop_timer_CallBack(V_Mat)


# OBTER ANGULO EM GRAU
def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw

# CALLBACKS --------------------------------------------------------
def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg
#--------------------------------------------------------------------

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):

#DEFINICAO DE VARIAVEIS    
    I_ang = 0
    I = 0
    msg = Twist()
    yaw = getAngle(odom)
    scan_len = len(scan.ranges)
# ERRO ANTERIOR INICIALIZADOS COMO NULO    
    p_erro_ang =0
    p_erro =0
    control_ang = 0

#ESTADO INICIAL BUSCA ANGULO QUE O ROBO DEVE IR    
    estado = 'Angulo'
     
    if not(scan_len > 0):
        control_ang = 0
        msg.linear.x = 0
			
    elif estado == 'Angulo':
       
        if min(scan.ranges[scan_len-10 : scan_len+10]) < 100: # encontrou objeto
           
            msg.angular.z = 0
            
            dir_obj = min (scan.ranges[scan_len-10 : scan_len+10])
            
            setpoint_ang = (dir_obj - scan.ranges[0])/(scan.ranges[scan_len-1] - scan.ranges[0]) #interpolacao
            setpoint_ang *= 200 #interpolacao
            setpoint_ang -= 100 #interpolacao
            
            error_ang = (setpoint_ang - yaw) #ref - y
            
            if abs(error_ang) > 180:
                if setpoint_ang < 0:
                    error_ang += 360 
                else:
                    error_ang -= 360
                    
            P_ang = kp_ang*error_ang #parte proporcional
            I_ang = I_ang + ki_ang*error_ang  #parte integrativa
            D_ang = (error_ang - p_erro_ang)*kd_ang #parte derivativa
            
            control_ang = P_ang + I_ang + D_ang
            p_erro_ang = error_ang #erro atual passa ser antigo na proxima vez
            
            msg.angular.z = control_ang
            estado = 'Distancia' 
            
             
				
        else:	
            if min(scan.ranges[scan_len-15 : scan_len+15]) < 100: #se nao enncontrou o objeto roda ate achar
                
                msg.angular.z = 0.15
            else:
                msg.angular.z = 0.3
           
       
                    
    if estado == 'Distancia':
        setpoint = 0.5 # 50 cm do objeto
        msg.angular.z =0
        scan_len = len(scan.ranges)
        if scan_len > 0:
            real_dist = min(scan.ranges[scan_len-10 : scan_len+10])

            error = -(setpoint - real_dist)
        
            P = kp*error #parte proporcional
            I = I + ki*error  #parte integrativa
            D = (error - p_erro)*kd # parte derivativa
            
            control = P+I+D
            erro = p_erro #erro atual sera proximo antigo
                
            if control > 1:
                control = 1
            elif control < -1:
                control = -1
        else:
            control = 0        
        
       
        msg.linear.x = control
        
  
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(0.05), timerCallBack)

rospy.spin()