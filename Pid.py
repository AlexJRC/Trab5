import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math

kp2 = 0.01
kp3 = 1

ki2 = 0.0001
ki3 = 0.01

kd2 = 0.01
kd3 = 1

mat = [2018000309, 2016006869, 2017009838, 34219,  2017003253]
odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------
def timer (mat): #Recebe a matricula e calcula a soma
    global matricula
    n = len(mat)
    resultado = 0
    media = 0
    freq = 0
    time = 0
	
    for matricula in mat:
        resultado = 0
        for x in str (matricula):
            resultado += int(x)
            media = media+resultado
    media = media/n
    freq = media
    time = 1/freq
    return time
    
time = timer(mat)
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
    control2 = 0
    erro1=0
    I1=0
    I2=0
    I3=0
    erro2=0
    erro3=0
    state = 'state1'
    msg = Twist()
    yaw = getAngle(odom)
    scan_len = len(scan.ranges)
	

	
    if (scan_len < 0):
        control2 = 0
        msg.linear.x = 0
			
    elif state == 'state1':
	      
            if min(scan.ranges[scan_len-10 : scan_len+10]) < 100:
           
                msg.angular.z = 0
                point = min (scan.ranges[scan_len-10 : scan_len+10])
                setpoint2 = (200*((point - scan.ranges[0])/(scan.ranges[scan_len-1] - scan.ranges[0]))) - 100
                error2 = (setpoint2 - yaw)
                if abs(error2) > 180:
                    if setpoint2 < 0:
                        error2 += 360 
                    else:
                        error2 -= 360
                P2 = kp2*error2
                I2 = ki2*error2 + I2 
                D2 = kd2*(error2 - erro2)
                control2 = P2+I2+D2
                erro2 = error2
                msg.angular.z = control2
                state = 'state2'   
       
                
            else:	
                if min(scan.ranges[scan_len-15 : scan_len+15]) < 100:
                    msg.angular.z = 0.3*0.5
                else:
                    msg.angular.z = 0.3
        

                    
    if state == 'state2':
            setpoint3 = 0.5
    
            scan_lenp = len(scan.ranges)
            if scan_lenp > 0:
                read = min(scan.ranges[scan_lenp-10 : scan_lenp+10])

                error3 = -(setpoint3 - read)
        
                P3 = kp3*error3
                I3 = ki3*error3 + I3 
                D3 = kd3*(error3 - erro3)
                control3 = P3+I3+D3
                erro3 = error3
                
                if control3 > 1:
                    control3 = 1
                elif control3 < -1:
                    control3 = -1
            else:
                control3 = 0        
        
            msg.linear.x = control3
        
  
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(time), timerCallBack)

rospy.spin()