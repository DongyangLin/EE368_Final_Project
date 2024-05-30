import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
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

def generate_path():
    path = Path()
    path.header.frame_id = "base_link"
    path.header.stamp = rospy.Time.now()

    vertices = [
        [0.435, 0.194, 0.457],           # 顶点 1 (home)
        [0.435 + 0.1, 0.194, 0.457],     # 顶点 2 (沿X轴正方向)
        [0.435, 0.194 + 0.1, 0.457],     # 顶点 3 (沿Y轴正方向)
        [0.435, 0.194, 0.457 + 0.1]      # 顶点 4 (沿Z轴正方向)
    ]

    edges = [
        [0, 1], [1, 2], [2, 0],    # 底面三条边
        [0, 3], [1, 3], [2, 3]     # 三条从底面到顶点的边
    ]

    num_points_per_edge = 2
    interpolated_poses = []

    for edge in edges:
        start = vertices[edge[0]]
        end = vertices[edge[1]]
        for j in range(num_points_per_edge):
            t = float(j) / (num_points_per_edge - 1)
            interp_pose = [(1 - t) * start[i] + t * end[i] for i in range(3)]
            interp_pose.extend([90.0, 0.0, 150.0])  # 保持角度不变
            interpolated_poses.append(interp_pose)

    for pose in interpolated_poses[0:len(interpolated_poses) - 1]:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = pose[2]
        q = euler_to_quaternion(pose[3], pose[4], pose[5])
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
        path = generate_path()
        path_pub.publish(path)
        rate.sleep()
