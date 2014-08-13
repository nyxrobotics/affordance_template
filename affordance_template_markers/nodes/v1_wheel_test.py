#!/usr/bin/env python
import argparse

import rospy
import roslib; roslib.load_manifest("affordance_template_markers")
import rospkg

from interactive_markers.interactive_marker_server import *


from affordance_template_markers.robot_config import *
from affordance_template_markers.affordance_template import *
from affordance_template_markers.template_utilities import *
from affordance_template_markers.recognition_object import *
import affordance_template_markers.atdf_parser


if __name__ == '__main__':

    rospy.init_node('v1_wheel_test')

    rospack = rospkg.RosPack()
    atl_path = rospack.get_path('affordance_template_library')

    server = InteractiveMarkerServer("affordance_template_server")

    template_filename = atl_path + "/templates/wheel.atdf"
    robot_config_filename = atl_path + "/robots/v1.yaml"

    try:

        # load RobotConfig from yaml and moveit package
        rc = RobotConfig()
        rc.load_from_file(robot_config_filename)
        rc.configure()

        # load in Affordance Template from file
        at = AffordanceTemplate(server, 0, robot_config=rc)
        at.load_from_file(template_filename)
        # at.print_structure()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass




