import rospy
from sensor_msgs.msg import JointState

def talker():
    state_pub = rospy.Publisher('/vec6/thruster_command', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        effort_ = JointState()
        effort_.name = ["f_port","f_star","m_port","m_star","b_port","b_star"] # port = left, star = right
        effort_.effort = [10,0,0,0,0,0]
        
        state_pub.publish(effort_)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
