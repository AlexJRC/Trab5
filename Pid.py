import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math


#kp1 = 1
kp2 = 0.01
kp3 = 1

#ki1 = 1
ki2 = 0.001
ki3 = 0.01

#kd1 = 1
kd2 = 0.01
kd3 = 0.1


odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------



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
	
    print ("Tamanho: ", scan_len)
    print ("Angulo lido: ", yaw)
	
    if not(scan_len > 0):
        control2 = 0
        msg.linear.x = 0
			
    elif state == 'state1':
	
    # POSICIONA DIRECAO ---------------------------------   
		
        print('Buscando...')
       
        if min(scan.ranges[scan_len-10 : scan_len+10]) < 100:
            print ("AAA")
            msg.angular.z = 0
            point = min (scan.ranges[scan_len-10 : scan_len+10])
            setpoint2 = (200*((point - scan.ranges[0])/(scan.ranges[scan_len-1] - scan.ranges[0]))) - 100
            print (setpoint2)
            error2 = (setpoint2 - yaw)
            if abs(error2) > 180:
                if setpoint2 < 0:
                    error2 += 360 
                else:
                    error2 -= 360
            P2 = kp2*error2
            I2 = ki2*error2 + I2 #ki1*error1
            D2 = kd2*(error2 - erro2)
            control2 = P2+I2+D2
            erro2 = error2
            msg.angular.z = control2
            state = 'state2'
            
             
				
        else:	
            if min(scan.ranges[scan_len-15 : scan_len+15]) < 100:
                print ("gira inferno")
                msg.angular.z = 0.3*0.5
            else:
                msg.angular.z = 0.3
           
       
                    
    if state == 'state2':
        setpoint3 = 0.5
    
        scan_len = len(scan.ranges)
        if scan_len > 0:
            read = min(scan.ranges[scan_len-10 : scan_len+10])

            error3 = -(setpoint3 - read)
        
            P3 = kp3*error3
            I3 = ki3*error3 + I3 #ki1*error1
            D3 = kd3*(error3 - erro3)
            control3 = P3+I3+D3
            erro3 = error3
                
            if control3 > 1:
                control3 = 1
            elif control3 < -1:
                control3 = -1
        else:
            control3 = 0        
        
        print (state)
        msg.linear.x = control3
        
  
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(0.05), timerCallBack)

rospy.spin()