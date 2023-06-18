# moton/に入ったjsonに記載された位置に移動する
# 使用するjsonファイルは引数で受け取る

# 操作対象のロボット名は quadrotor1 と quadrotor2, 目標位置(速度)を伝える先は /<ロボット名>/uav/nav
# メッセージ形式はaerial_robot_msgs/FlightNav
import rospy
import sys
import json
import math
from numpy import deg2rad, arccos
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped, Point

config_sim={"name1":"quadrotor1","name2":"quadrotor2"}
config_real={"name1":"assemble_quadrotors1","name2":"assemble_quadrotors2"}

def task_command(isreal, task_name):

    file_name=task_name+".json"
    if isreal==True:
        name1=config_real["name1"]
        name2=config_real["name2"]
    else:
        name1=config_sim["name1"]
        name2=config_sim["name2"]
    try:
        with open(f"motion/{file_name}",'r') as file:
            motion_data=json.loads(file.read())["movement"]
    except FileNotFoundError:
        print(f"movement {task_name} not found")
        return

    # prepare publisher
    rospy.init_node("task_command")
    pub1 = rospy.Publisher(f'/{name1}/uav/nav',FlightNav, queue_size=10)
    pub2 = rospy.Publisher(f'/{name2}/uav/nav',FlightNav, queue_size=10)
    rate=rospy.Rate(100)

    # get starting position/yaw
    pos1=[]
    pos2=[]

    yaw1=[]
    yaw2=[]
    def cb1(data):
        pos1.append(data.pose.position)
        yaw1.append(2*arccos(data.pose.orientation.w))

    def cb2(data):
        pos2.append(data.pose.position)
        yaw2.append(2*arccos(data.pose.orientation.w))

    def retrieve_pos():
        sub1=rospy.Subscriber(f'/{name1}/mocap/pose',PoseStamped,cb1)
        sub2=rospy.Subscriber(f'/{name2}/mocap/pose',PoseStamped,cb2)
        while not rospy.is_shutdown():
            if len(pos1)>=10 and len(pos2)>=10:
                sub1.unregister()
                sub2.unregister()
                break
        rospy.sleep(0.1)

    def mean_position(point_array):
        mean_point = Point()
        num_points = len(point_array)

        if num_points == 0:
            return mean_point

        sum_x = 0.0
        sum_y = 0.0
        sum_z = 0.0

        for point in point_array:
            sum_x += point.x
            sum_y += point.y
            sum_z += point.z

        mean_point.x = sum_x / num_points
        mean_point.y = sum_y / num_points
        mean_point.z = sum_z / num_points

        return mean_point

    retrieve_pos()

    pos1_orig=mean_position(pos1)
    pos2_orig=mean_position(pos2)

    yaw1_orig=sum(yaw1)/len(yaw1)
    yaw2_orig=sum(yaw2)/len(yaw2)
    print(yaw1_orig,yaw2_orig)

    # execute described motions

    for command in motion_data:
        print(command)
        qr1=command["qr1"]
        qr2=command["qr2"]
        duration=command["duration"]

        # generate message
        
        qr1_msg=FlightNav()
        qr2_msg=FlightNav()

        qr1_tgt=[pos1_orig.x+qr1["x"], pos1_orig.y+qr1["y"], pos1_orig.z+qr1["z"], yaw1_orig+deg2rad(qr1["yaw"]) ]
        qr2_tgt=[pos2_orig.x+qr2["x"], pos2_orig.y+qr2["y"], pos2_orig.z+qr2["z"], yaw2_orig+deg2rad(qr2["yaw"]) ]

        qr1_msg.pos_xy_nav_mode=2
        qr1_msg.yaw_nav_mode=2
        qr1_msg.pos_z_nav_mode=0
        [qr1_msg.target_pos_x, qr1_msg.target_pos_y, qr1_msg.target_pos_z, qr1_msg.target_yaw]=qr1_tgt

        qr2_msg.pos_xy_nav_mode=2
        qr2_msg.yaw_nav_mode=2
        qr2_msg.pos_z_nav_mode=0
        [qr2_msg.target_pos_x, qr2_msg.target_pos_y, qr2_msg.target_pos_z, qr2_msg.target_yaw]=qr2_tgt

        #publish message for duration

        time_delta=rospy.Duration(duration)
        end_time=rospy.get_rostime()+time_delta

        while not rospy.is_shutdown() and rospy.get_rostime()<end_time:
            pub1.publish(qr1_msg)
            pub2.publish(qr2_msg)
            rate.sleep()

    rospy.signal_shutdown("Movement finished")

if __name__=="__main__":
    
    try: 
        task_name=sys.argv[1]
    except IndexError:
        print("Not enough arguments")
        exit()
    
    try:
        real=(sys.argv[2]=="real")
    except IndexError:
        print("Not enough arguments")
        exit()
        
    task_command(real,task_name)