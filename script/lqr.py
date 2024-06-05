import sys
import rospy
import time
import math
from kortex_driver.srv import *
from kortex_driver.msg import *
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Arm import Gen3LiteArm
from scipy.spatial.transform import Rotation
import signal
from collections import deque

class ArmControl(object):
    def __init__(self):
        try:
            rospy.init_node('arm_control')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = 'my_gen3_lite'

            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init class members
            self.my_path_ = None
            self.curr_state_ = None
            self.state_history = deque(maxlen=8)  # 窗口大小为5的队列
            self.plot = False
            self.real_path_ = Path()
            self.real_path_.header.frame_id = 'base_link'
            self.arm_model = Gen3LiteArm()
            self.target_pose_list = []

            # Init LQR Controller
            self.dt = 0.001
            self.lqr = LQRControl(self.dt, True)
            self.lqr.solve()
            self.v_history = deque(maxlen=7)

            # Init the subscribers and publishers
            self.path_subscriber = rospy.Subscriber("/myPath", Path, self.get_path, queue_size=1)
            self.curr_state_sub = rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.get_curr_state, queue_size=1)
            self.real_path_publisher = rospy.Publisher("/real_path", Path, queue_size=1)
            
            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)
            
            rospy.wait_for_service('/my_gen3_lite/base/send_joint_speeds_command')
            self.send_speeds = rospy.ServiceProxy('/my_gen3_lite/base/send_joint_speeds_command', SendJointSpeedsCommand)

            
            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def signal_handler(self, sig, frame):
        self.send_joint_speeds_command([0, 0, 0, 0, 0, 0])
        print("Ctrl+C pressed")
        sys.exit(0)
    
    def cb_action_topic(self, notif):
            self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True
    
    def go_home(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()
    
    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True
        
    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def get_path(self, msg):
        self.my_path_ = msg
    
    def get_curr_state(self, msg):
        current_state = np.array(msg.position[0:6])
        self.state_history.append(current_state)
        if len(self.state_history) == self.state_history.maxlen:
            self.curr_state_ = np.mean(self.state_history, axis=0)
        else:
            self.curr_state_ = current_state
        # print(np.degrees(self.curr_state_))
        # self.curr_state_ = msg
        self.plot_real_path()

    def get_joint_angels(self):
        return np.degrees(self.curr_state_)

    def plot_real_path(self):
        # if self.plot == False:
        #     return
        # 创建一个 PoseStamped 消息
        real_poses = PoseStamped()
        real_poses.header.frame_id = "base_link"
        real_poses.header.stamp = rospy.Time.now()
        self.real_path_.header.stamp = rospy.Time.now()

        # 获取机械臂末端的实际位置
        tip_pose = self.forward_kinematics(self.curr_state_)

        # 填充 PoseStamped 消息
        real_poses.pose.position.x = tip_pose[0]
        real_poses.pose.position.y = tip_pose[1]
        real_poses.pose.position.z = tip_pose[2]

        # 将 PoseStamped 消息添加到 Path 中
        self.real_path_.poses.append(real_poses)

        # 发布实际路径
        self.real_path_publisher.publish(self.real_path_)

    def gen_T(self,theta,a,alpha,d):
        T=np.array([[np.cos(theta),-np.cos(alpha)*np.sin(theta),np.sin(theta)*np.sin(alpha),a*np.cos(theta)],
                    [np.sin(theta),np.cos(alpha)*np.cos(theta),np.cos(theta)*-np.sin(alpha),a*np.sin(theta)],
                    [0,np.sin(alpha),np.cos(alpha),d],
                    [0,0,0,1]])
        return T

    def get_transform_matrix(self,theta):
        theta_copy = theta.copy()
        theta_copy[1]+=np.pi/2
        theta_copy[2]+=np.pi/2
        theta_copy[3]+=np.pi/2
        theta_copy[4]+=np.pi
        theta_copy[5]+=np.pi/2
        alpha=np.array([np.pi/2,np.pi,np.pi/2,np.pi/2,np.pi/2,0])
        a=np.array([0,280,0,0,0,0])
        d=np.array([128.3+115.0,30,20,140+105,28.5+28.5,105+130])
        T0_1=self.gen_T(theta_copy[0],a[0],alpha[0],d[0])
        T1_2=self.gen_T(theta_copy[1],a[1],alpha[1],d[1])
        T2_3=self.gen_T(theta_copy[2],a[2],alpha[2],d[2])
        T3_4=self.gen_T(theta_copy[3],a[3],alpha[3],d[3])
        T4_5=self.gen_T(theta_copy[4],a[4],alpha[4],d[4])
        T5_6=self.gen_T(theta_copy[5],a[5],alpha[5],d[5])
        T_rote=np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
        T0_6=T0_1@T1_2@T2_3@T3_4@T4_5@T5_6@T_rote
        return T0_6

    def forward_kinematics(self,state):
        # thetas=state.position[0:6]
        # thetas=np.array(thetas)
        T = self.get_transform_matrix(state)
        rotation_matrix = T[:3, :3]
        rotation = Rotation.from_matrix(rotation_matrix)
        euler_angles = rotation.as_euler('xyz', degrees=True)
        z=euler_angles[2]
        y=euler_angles[1]
        x=euler_angles[0]
        x_l=T[0,3]/1000
        y_l=T[1,3]/1000
        z_l=T[2,3]/1000
        return [x_l,y_l,z_l,x,y,z]
    
    def send_joint_speeds_command(self,speeds):
        try:
            # 创建请求
            joint_speeds = Base_JointSpeeds()
            for i, speed in enumerate(speeds):
                joint_speed = JointSpeed()
                joint_speed.joint_identifier = i
                joint_speed.value = speed
                joint_speed.duration = 0  # 0 for continuous command
                joint_speeds.joint_speeds.append(joint_speed)

            # 发送请求
            response = self.send_speeds(joint_speeds)
            print(f"Sent joint speeds: {speeds}")

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def go_to_pose(self, xd):
        self.v_history.clear()
        # Convert target to degrees
        xd = np.degrees(xd)
        x0 = self.get_joint_angels()
        # print("Current State: ", x0)
        while np.linalg.norm(x0 - xd) > 1:
            x0 = self.get_joint_angels()
            print("Current State: ", np.degrees(self.curr_state_))
            print("Target State: ", xd)
            u = -self.lqr.K.dot(x0 - xd)
            u = np.radians(u)
            print("Control Signal: ", u)
            # self.v_history.append(u)
            # if len(self.v_history) == self.v_history.maxlen:
            #     u = np.mean(self.v_history, axis=0)
            # print("Control Signal: ", u)
            self.send_joint_speeds_command(u)
            time.sleep(self.dt)
        return True

    def follow_path(self):
        self.plot=True
        if self.my_path_ == None:
            rospy.logerr("No path received yet")
            return False
        
        # First calculate the target poses
        for i in range(len(self.my_path_.poses)):
            ref_ee_pose=[self.my_path_.poses[i].pose.position.x,self.my_path_.poses[i].pose.position.y,self.my_path_.poses[i].pose.position.z]
            ev_angles=self.arm_model.arm.inverse_kinematics(ref_ee_pose,np.zeros(6))
            # ev_poses=self.forward_kinematics(ev_angles)
            self.target_pose_list.append(ev_angles)
        
        print("Begin Path!")
        # Then go to each target pose
        for i in range(len(self.target_pose_list)):
            if not self.go_to_pose(self.target_pose_list[i]):
                return False
        return True
    
    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        rospy.wait_for_service('/my_gen3_lite/base/send_gripper_command')
        self.send_gripper_command = rospy.ServiceProxy('/my_gen3_lite/base/send_gripper_command', SendGripperCommand)
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(3)
            return True

    def main(self):
        signal.signal(signal.SIGINT, self.signal_handler)

        if not self.is_init_success:
            rospy.logerr("Init failed!")
            return

        # Clear faults
        if not self.example_clear_faults():
            return

        # Set the cartesian reference frame
        if not self.example_set_cartesian_reference_frame():
            return

        # Subscribe to the robot notification
        if not self.example_subscribe_to_a_robot_notification():
            return
        
        if not self.example_send_gripper_command(1.0):
            return

        # Home the robot
        if not self.go_home():
            return
        
        # print(self.forward_kinematics(self.curr_state_))
        # while True:
        #     time.sleep(0.5)
        # Define a target pose
        if not self.follow_path():
            return

        # Home the robot
        if not self.go_home():
            return

        # Unsubscribe from the robot notification
        self.action_topic_sub.unregister()

        rospy.loginfo("All done!")

class LQRControl:
    def __init__(self, dt, SIMULATION=False):
        self.n = 6
        self.A = np.eye(self.n)
        self.B = dt*np.eye(self.n)
        self.dt = dt
        self.Q = np.diag([30, 30, 30, 30, 30, 30])
        self.R = np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        if SIMULATION:
            self.R = np.diag([20, 20, 20, 20, 20, 20])
        self.K = None
    
    def solve(self):
        P = self.Q
        max_iter = 100000
        tolerance = 1e-5
        for i in range(max_iter):
            Pnext = self.Q + self.A.T.dot(P).dot(self.A) - self.A.T.dot(P).dot(self.B).dot(np.linalg.inv(self.R + self.B.T.dot(P).dot(self.B))).dot(self.B.T).dot(P).dot(self.A)
            # If Converged:
            if np.linalg.norm(P - Pnext) < tolerance:
                break
            P = Pnext
        else:
            print('Max Iterations Reached')
            exit(0)
        self.K = np.linalg.inv(self.R + self.B.T.dot(P).dot(self.B)).dot(self.B.T).dot(P).dot(self.A)

if __name__ == '__main__':
    arm_control = ArmControl()
    arm_control.main()