import rospy
# import copter_utils
from clever import srv 
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
from geometry_msgs.msg import Point
import time
import threading
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Int8
from simple_pid import PID
# m_srv.arm
rospy.init_node('first')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
release = rospy.ServiceProxy('release', Trigger)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
id_pub = rospy.Publisher('/id_i', Int8, queue_size=10)

def take_off_us(set_velocity_serv, navigate_serv, z = 1, speed = 0.3, tolerance = 0.08):
    z_now = 0
    x_now = 0
    y_now = 0

    def callback(data):
        global z_now
        z_now = data.range
        # print(data.range)
        
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.Subscriber("/ultrasound", Range, callback)
    def callback_2(data):
        global x_now, y_now
        if data.x == -1:
            x_now = 160
            y_now = 120
        else:
            x_now = data.x
            y_now = data.y
        # print(data)
        
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.Subscriber("/point_vision", Point, callback_2)
    pid_x = PID(0.6, 0, 0.0, setpoint=0)
    pid_y = PID(0.6, 0, 0.0, setpoint=0)
    pid_z = PID(0.6, 0, 0.0, setpoint=0)
    pid_x.output_limits = (-0.2, 0.2)
    pid_y.output_limits = (-0.2, 0.2)
    pid_z.output_limits = (-0.2, 0.2)

    print('arming')
    # arming_serv(True)
    
    navigate_serv(z=0, yaw=0, speed=0.2, frame_id='fcu_horiz', auto_arm=True)
    rospy.sleep(1)
    while True:
        global z_now,x_now, y_now
        con_x = pid_x((160-x_now) / 160)
        con_y = pid_y((y_now-120)/120)
        # con_z = pid_z(z_now-z)
        print(con_x, con_y)
        set_velocity_serv(vx=((x_now-160)/160)*0.2, vy=((120-y_now)/120)*0.2, vz=speed, frame_id='fcu_horiz')
        # print(z_now)
        if abs((z_now - z)) < tolerance:
            break
        rospy.sleep(0.3)
    set_velocity_serv(vx=0.0, vy=0.0, vz=0.0, frame_id='fcu_horiz')
    set_velocity_serv(vx=0.0, vy=0.0, vz=0.0, frame_id='fcu_horiz')
def stay(set_velocity_serv, navigate_serv, z = 1, speed = 0.3, tolerance = 0.08):
    z_now = 0
    x_now = 0
    y_now = 0

    def callback(data):
        global z_now
        z_now = data.range
        # print(data.range)
        
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.Subscriber("/ultrasound", Range, callback)
    def callback_2(data):
        global x_now, y_now
        if data.x == -1:
            x_now = 160
            y_now = 120
        else:
            x_now = data.x
            y_now = data.y
        # print(data)
        
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.Subscriber("/point_vision", Point, callback_2)
    pid_x = PID(0.4, 0, 0.0, setpoint=0)
    pid_y = PID(0.4, 0, 0.0, setpoint=0)
    pid_z = PID(0.4, 0, 0.0, setpoint=0)
    pid_x.output_limits = (-0.08, 0.08)
    pid_y.output_limits = (-0.08, 0.08)
    pid_z.output_limits = (-0.08, 0.08)

    # print('arming')
    # arming_serv(True)
    
    # navigate_serv(z=0, yaw=0, speed=0.2, frame_id='fcu_horiz', auto_arm=True)
    rospy.sleep(0.3)
    for i in range(int(5/0.2)):
        global z_now,x_now, y_now
        con_x = pid_x((160-x_now) / 160)
        con_y = pid_y((y_now-120)/120)
        # con_z = pid_z(z_now-z)
        print(con_x, con_y)
        set_velocity_serv(vx=((x_now-160)/160)*0.2, vy=((120-y_now)/120)*0.2, vz=(z-z_now)*0.2, frame_id='fcu_horiz')
        # print(z_now)
        # if abs((z_now - z)) < tolerance:
            # break
        rospy.sleep(0.3)
    set_velocity_serv(vx=0.0, vy=0.0, vz=0.0, frame_id='fcu_horiz')
    set_velocity_serv(vx=0.0, vy=0.0, vz=0.0, frame_id='fcu_horiz')
def land_us(set_velocity_serv, arming_serv, speed):
    z_now = 2

    def callback(data):
        global z_now
        z_now = data.range
        print(data.range)
        
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.Subscriber("/ultrasound", Range, callback)
    while True:
        global z_now
        set_velocity_serv(vx=0.0, vy=0.0, vz=-speed, frame_id='fcu_horiz')
        print(z_now)
        if z_now < 0.06 and z_now > 0.03:
            break
        rospy.sleep(0.2)
    set_velocity_serv(vx=0.0, vy=0.0, vz=0.0, frame_id='fcu_horiz')
    set_velocity_serv(vx=0.0, vy=0.0, vz=0.0, frame_id='fcu_horiz')
    print('disarming')
    arming_serv(False)
#id_pub.publish(2)
#take_off_us(set_velocity, navigate, 1, 0.6,  0.08)
navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='fcu_horiz', auto_arm=True)
stay(set_velocity, navigate, 1.5, 0.2,  0.08)
# id_pub.publish(3)
# stay(set_velocity, navigate, 2, 1, 0.3)
land_us(set_velocity, arming, 0.7)

# while True:
