import rospy
from kortex_driver.srv import SendJointSpeedsCommand
from kortex_driver.msg import JointSpeed, Base_JointSpeeds

def send_joint_speeds_command(speeds):
    rospy.wait_for_service('/my_gen3_lite/base/send_joint_speeds_command')
    try:
        send_speeds = rospy.ServiceProxy('/my_gen3_lite/base/send_joint_speeds_command', SendJointSpeedsCommand)

        # 创建请求
        joint_speeds = Base_JointSpeeds()
        for i, speed in enumerate(speeds):
            joint_speed = JointSpeed()
            joint_speed.joint_identifier = i + 1
            joint_speed.value = speed
            joint_speed.duration = 0  # 0 for continuous command
            joint_speeds.joint_speeds.append(joint_speed)

        # 发送请求
        response = send_speeds(joint_speeds)
        print(f"Sent joint speeds: {speeds}, Response: {response}")

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('send_joint_speeds_command_example')

    # 设置关节速度，例如6个关节的速度
    joint_speeds = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    send_joint_speeds_command(joint_speeds)
