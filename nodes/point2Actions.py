#!/usr/bin/python
import rospy
from geometry_msgs.msg import Point
import roslib
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
roslib.load_manifest('nomad')

ac = None

def callback(msg):
        global ac
        goal = MoveBaseGoal()
        
        rospy.loginfo("Recieved %f, %f", msg.x, msg.y)

        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.position.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, 0.0)
        goal.target_pose.pose.orientation = Quaternion(*quat.tolist())

        rospy.loginfo("Sending goal")
        ac.send_goal(goal)
        ac.wait_for_result()
        print (ac.get_result())

def listener():
        global ac
        rospy.init_node('point2Actions')
        pub = rospy.Subscriber("nomad/points", Point, callback)
        ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        print ("Waiting for server...")
        ac.wait_for_server()
        print ("Done!")

        rospy.spin()

if __name__ == '__main__':
        try:
                listener()
        except rospy.ROSInterruptException:
                pass
