import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MoveItCartesianCircleDemo:
    def __init__(self):
        # Initialize move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('wuyong', anonymous=True)
        
        # Initialize the move group for the robotic arm
        self.arm = MoveGroupCommander('arm')
        
        # Allow replanning
        self.arm.allow_replanning(True)
        
        # Set reference frame
        self.arm.set_pose_reference_frame('base_link')
        
        # Set goal position and orientation tolerance
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        
        # Get the end effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # self.arm.set_max_velocity_scaling_factor(0.3) 
        # self.arm.set_max_acceleration_scaling_factor(0.01)  # 将加速度缩放因子设置为最大值1.0


        # 初始化Marker
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        
        # Move the arm to the home position
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
        
        # Get the starting pose
        self.start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # Define circle parameters
        self.radius = 0.1  # radius of the circle
        self.center_x = self.start_pose.position.x
        self.center_y = self.start_pose.position.y
        
        # Generate waypoints for the circle
        self.generate_circle_waypoints(self.center_x, self.center_y, self.radius)
        

        # Return to home position
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
        
        # Shutdown MoveIt
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    
    def generate_circle_waypoints(self, center_x, center_y, radius):
        waypoints = []
        num_points = 36  # number of waypoints around the circle
        angle_step = 2 * np.pi / num_points

        for i in range(num_points):
            angle = i * angle_step
            x = center_x + radius * np.cos(angle)
            y = center_y + radius * np.sin(angle)
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = self.start_pose.position.z
            pose.orientation = self.start_pose.orientation
            waypoints.append(pose)

        self.publish_marker(waypoints)
     
        fraction = 0.0
        maxtries = 100
        attempts = 0
        
        self.arm.set_start_state_to_current_state()
        
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints,   # waypoint poses
                0.01,            # eef_step
                0.0)             # jump_threshold
                
            attempts += 1
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
        
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.arm.execute(plan, wait=True)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")
     


    def publish_marker(self, waypoints):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for pose in waypoints:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z
            marker.points.append(point)

        self.marker_pub.publish(marker)  
if __name__ == "__main__":
    try:
        MoveItCartesianCircleDemo()
    except rospy.ROSInterruptException:
        pass
