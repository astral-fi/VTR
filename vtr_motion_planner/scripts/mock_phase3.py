#!/usr/bin/env python
# mock_phase3.py
# ---------------------------------------------------------------------------
# Standalone test helper – publishes a synthetic goal list on /vtr/goal_list
# so Phase 4 can be exercised without a running Phase 3.
#
# Usage (after sourcing your workspace):
#   rosrun vtr_motion_planner mock_phase3.py
#   rosrun vtr_motion_planner mock_phase3.py _rate:=2.0 _pattern:=curved
# ---------------------------------------------------------------------------

import rospy
import math
from geometry_msgs.msg import Point
from vtr_motion_planner.msg import GoalList

# ---------------------------------------------------------------------------
# Goal patterns
# ---------------------------------------------------------------------------

def straight_goals(n=5, spacing=1.0):
    """Goals in a straight line ahead of the robot."""
    goals = []
    for i in range(1, n + 1):
        p = Point()
        p.x = i * spacing
        p.y = 0.0
        p.z = 0.0
        goals.append(p)
    return goals


def curved_goals(n=5, spacing=1.0, curvature=0.3):
    """Goals along a gentle left-curving arc."""
    goals = []
    theta = 0.0
    x, y  = 0.0, 0.0
    for i in range(1, n + 1):
        theta += curvature * spacing
        x += spacing * math.cos(theta)
        y += spacing * math.sin(theta)
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        goals.append(p)
    return goals


def zigzag_goals(n=5, spacing=1.0, lateral=0.4):
    """Goals alternating left/right – tests scoring breadth."""
    goals = []
    for i in range(1, n + 1):
        p = Point()
        p.x = i * spacing
        p.y = lateral * (1 if i % 2 == 0 else -1)
        p.z = 0.0
        goals.append(p)
    return goals


PATTERNS = {
    'straight': straight_goals,
    'curved':   curved_goals,
    'zigzag':   zigzag_goals,
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rospy.init_node('mock_phase3', anonymous=True)

    rate_hz = rospy.get_param('~rate', 2.0)
    pattern = rospy.get_param('~pattern', 'straight')
    topic   = rospy.get_param('~goal_list_topic', '/vtr/goal_list')

    if pattern not in PATTERNS:
        rospy.logerr("Unknown pattern '%s'. Choose from: %s",
                     pattern, list(PATTERNS.keys()))
        return

    pub  = rospy.Publisher(topic, GoalList, queue_size=1)
    rate = rospy.Rate(rate_hz)

    rospy.loginfo("[mock_phase3] Publishing '%s' goals at %.1f Hz on %s",
                  pattern, rate_hz, topic)

    while not rospy.is_shutdown():
        msg              = GoalList()
        msg.header.stamp = rospy.Time.now()
        msg.frame_id     = 'base_link'
        msg.goals        = PATTERNS[pattern]()
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
