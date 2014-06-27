#!/usr/bin/env python

#import roslib; roslib.load_manifest('turtlebot_patrol')
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

class GoalPublisher():
    """ Publishes goals. """
    def __init__(self, goal, goal_num):
        rospy.loginfo('Got goal #{0}.'.format(goal_num))
        self.goal = goal

        self.waiting_for_nav = True

        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
        while self.pub.get_num_connections() == 0:  # wait for publisher to connect!
            rospy.sleep(0.1)

        self.sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.navigation_callback)

        self.publish_once()
        rospy.loginfo('Sent goal #{0}.'.format(goal_num))

        while self.waiting_for_nav:
            rospy.sleep(0.01)
        rospy.loginfo('Done with goal #{0}.\n\n'.format(goal_num))


    def publish_once(self):
        """ Publishes the current goal. """
        self.pub.publish(self.goal)


    def navigation_callback(self, result):
        """ Sets flag to show that navigation is done -- ready for new goal! """
        self.waiting_for_nav = False
        

if __name__ == '__main__':

    rospy.init_node('goal_publisher')

    #### DEFINE GOALS ####
    goals = []

    # Goal #1: 
    goals.append(PoseStamped())
    goals[-1].header.frame_id = 'map'
    goals[-1].header.stamp = rospy.Time.now()
    goals[-1].pose.position.x =  3.0
    goals[-1].pose.position.y = -2.8
    goals[-1].pose.orientation.z = 0.18
    goals[-1].pose.orientation.w = 0.98

    # Goal #2: 
    goals.append(PoseStamped())
    goals[-1].header.frame_id = 'map'
    goals[-1].header.stamp = rospy.Time.now()
    goals[-1].pose.position.x =  4.8
    goals[-1].pose.position.y = -6.5
    goals[-1].pose.orientation.z = 0.46
    goals[-1].pose.orientation.w = 0.89

    # Goal #3: 
    goals.append(PoseStamped())
    goals[-1].header.frame_id = 'map'
    goals[-1].header.stamp = rospy.Time.now()
    goals[-1].pose.position.x =  3.2
    goals[-1].pose.position.y = -3.9
    goals[-1].pose.orientation.z = 1.0
    goals[-1].pose.orientation.w = 0.0

    
    #### GO TO GOALS ####
    num = 1
    for goal in goals:
        GoalPublisher(goal, num)
        num += 1
