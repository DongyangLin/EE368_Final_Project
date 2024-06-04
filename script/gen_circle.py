import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class Quaternion:
    def __init__(self, x=0, y=0, z=0, w=0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

def deg_to_rad(deg):
    return deg * math.pi / 180.0

def euler_to_quaternion(alpha, beta, theta):
    alpha = deg_to_rad(alpha)
    beta = deg_to_rad(beta)
    theta = deg_to_rad(theta)

    cos_alpha = math.cos(alpha / 2)
    sin_alpha = math.sin(alpha / 2)
    cos_beta = math.cos(beta / 2)
    sin_beta = math.sin(beta / 2)
    cos_theta = math.cos(theta / 2)
    sin_theta = math.sin(theta / 2)

    q = Quaternion()
    q.w = cos_alpha * cos_beta * cos_theta + sin_alpha * sin_beta * sin_theta
    q.x = sin_alpha * cos_beta * cos_theta - cos_alpha * sin_beta * sin_theta
    q.y = cos_alpha * sin_beta * cos_theta + sin_alpha * cos_beta * sin_theta
    q.z = cos_alpha * cos_beta * sin_theta - sin_alpha * sin_beta * cos_theta

    return q

def generate_circle_path():
    path = Path()
    path.header.frame_id = "base_link"
    path.header.stamp = rospy.Time.now()

    center_x = 0.435
    center_y = 0.194
    center_z = 0.457
    radius = 0.1
    sample_angel = 8
    num_points = int(360/sample_angel)

    for i in range(num_points):
        angle = i * sample_angel    
        angle_rad = deg_to_rad(angle)
        x = center_x + radius * math.cos(angle_rad)
        z = center_z + radius * math.sin(angle_rad)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = center_y  # 固定y轴坐标
        pose_stamped.pose.position.z = z
        q = euler_to_quaternion(90.0, 0.0, 150.0)  # 保持角度不变
        pose_stamped.pose.orientation.x = q.x
        pose_stamped.pose.orientation.y = q.y
        pose_stamped.pose.orientation.z = q.z
        pose_stamped.pose.orientation.w = q.w
        path.poses.append(pose_stamped)
    
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "base_link"
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.pose.position.x = center_x + radius
    pose_stamped.pose.position.y = center_y  # 固定y轴坐标
    pose_stamped.pose.position.z = center_z
    q = euler_to_quaternion(90.0, 0.0, 150.0)  # 保持角度不变
    pose_stamped.pose.orientation.x = q.x
    pose_stamped.pose.orientation.y = q.y
    pose_stamped.pose.orientation.z = q.z
    pose_stamped.pose.orientation.w = q.w
    path.poses.append(pose_stamped)

    return path

if __name__ == '__main__':
    rospy.init_node('gen_path_node')
    path_pub = rospy.Publisher('/myPath', Path, queue_size=1)
    rate = rospy.Rate(10)  # 发布频率为10Hz

    while not rospy.is_shutdown():
        path = generate_circle_path()
        path_pub.publish(path)
        rate.sleep()
