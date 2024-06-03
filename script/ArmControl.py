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
            self.plot = False
            self.real_path_ = Path()
            self.real_path_.header.frame_id = 'base_link'
            self.arm_model = Gen3LiteArm()
            self.target_pose_list = []

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

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

        
    
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
    
    def go_to_pose(self, pose, id):
        my_cartesian_speed = CartesianSpeed()
        my_cartesian_speed.translation = 0.2 # m/s
        my_cartesian_speed.orientation = 20  # deg/s

        my_constrained_pose = ConstrainedPose()
        my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

        # my_constrained_pose.target_pose.x = pose.position.x
        # my_constrained_pose.target_pose.y = pose.position.y
        # my_constrained_pose.target_pose.z = pose.position.z
        # my_constrained_pose.target_pose.theta_x = 90.0
        # my_constrained_pose.target_pose.theta_y = 0.0
        # my_constrained_pose.target_pose.theta_z = 150.0
        my_constrained_pose.target_pose.x = pose[0]
        my_constrained_pose.target_pose.y = pose[1]
        my_constrained_pose.target_pose.z = pose[2]
        my_constrained_pose.target_pose.theta_x = pose[3]
        my_constrained_pose.target_pose.theta_y = pose[4]
        my_constrained_pose.target_pose.theta_z = pose[5]

        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
        req.input.name = "pose"+str(id)
        req.input.handle.action_type = ActionType.REACH_POSE
        req.input.handle.identifier = 1001

        rospy.loginfo("Sending pose 1...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send" + req.input.name)
            return False
        else:
            rospy.loginfo("Waiting for"+req.input.name+"to finish...")

        return self.wait_for_action_end_or_abort()
    
    def get_path(self, msg):
        self.my_path_ = msg
    
    def follow_path(self):
        self.plot=True
        if self.my_path_ == None:
            rospy.logerr("No path received yet")
            return False
        
        # First calculate the target poses
        for i in range(len(self.my_path_.poses)):
            ref_ee_pose=[self.my_path_.poses[i].pose.position.x,self.my_path_.poses[i].pose.position.y,self.my_path_.poses[i].pose.position.z]
            ev_angles=self.arm_model.arm.inverse_kinematics(ref_ee_pose,np.zeros(6))
            ev_poses=self.forward_kinematics(ev_angles)
            self.target_pose_list.append(ev_poses)
        
        # Then go to each target pose
        for i in range(len(self.target_pose_list)):
            if not self.go_to_pose(self.target_pose_list[i],i):
                return False
        return True
    
    def get_curr_state(self, msg):
        self.curr_state_ = np.array(msg.position[0:6])
        # self.curr_state_ = msg
        self.plot_real_path()

    def plot_real_path(self):
        if self.plot == False:
            return
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
        theta[1]+=np.pi/2
        theta[2]+=np.pi/2
        theta[3]+=np.pi/2
        theta[4]+=np.pi
        theta[5]+=np.pi/2
        alpha=np.array([np.pi/2,np.pi,np.pi/2,np.pi/2,np.pi/2,0])
        a=np.array([0,280,0,0,0,0])
        d=np.array([128.3+115.0,30,20,140+105,28.5+28.5,105+130])
        T0_1=self.gen_T(theta[0],a[0],alpha[0],d[0])
        T1_2=self.gen_T(theta[1],a[1],alpha[1],d[1])
        T2_3=self.gen_T(theta[2],a[2],alpha[2],d[2])
        T3_4=self.gen_T(theta[3],a[3],alpha[3],d[3])
        T4_5=self.gen_T(theta[4],a[4],alpha[4],d[4])
        T5_6=self.gen_T(theta[5],a[5],alpha[5],d[5])
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
    
    def main(self):
        if not self.is_init_success:
            rospy.logerr("Initialization failed")
            return False
        
        if not self.example_clear_faults():
            return False
        
        if not self.go_home():
            return False
        
        if not self.example_set_cartesian_reference_frame():
            return False
        
        if not self.example_subscribe_to_a_robot_notification():
            return False

        while not rospy.is_shutdown():
            if self.my_path_ != None:
                break
        if not self.follow_path():
            return False
        rospy.loginfo("Complete painting!")
        return True

if __name__ == '__main__':
    arm_control = ArmControl()
    arm_control.main()