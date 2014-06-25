
import yaml

import rospy
import tf
import PyKDL as kdl

import geometry_msgs.msg
import sensor_msgs.msg

from nasa_robot_teleop.moveit_interface import *
from nasa_robot_teleop.kdl_posemath import *
from nasa_robot_teleop.pose_update_thread import *
from nasa_robot_teleop.end_effector_helper import *

# class MoveItInterfaceThread(threading.Thread) :
#     def __init__(self, robot_name, config_pacakge, manipulator_group) :
#         super(MoveItInterfaceThread,self).__init__()
#         self.mutex = threading.Lock()

#         self.robot_name = robot_name
#         self.config_package = config_package
#         self.manipulator_group = manipulator_group

#         self.moveit_interface = MoveItInterface(self.robot_name,self.config_package)
#         self.root_frame = self.moveit_interface.get_planning_frame()
#         self.moveit_ee_groups = self.moveit_interface.srdf_model.get_end_effector_groups()
#         selfparent_group = self.moveit_interface.srdf_model.get_end_effector_parent_group(g)

#     def run(self) :
#         while True :
#             self.mutex.acquire()
#             try :
#                 try :
#                     rospy.sleep(0.1)
#                 except :
#                     rospy.logdebug("MoveItInterfaceThread::run() -- could not update thread")
#             finally :
#                 self.mutex.release()

#             rospy.sleep(0.1)

#     def get_end_effector_groups(self) :
#         return self.moveit_interface.srdf_model.get_end_effector_groups()

#     def get_groups(self) :
#         return self.moveit_interface.groups

#     def get_urdf_model(self) :
#         return self.moveit_interface.get_urdf_model()

#     def get_parent_link(self, g) :
#         if g in self.moveit_interface.srdf_model.group_end_effectors:
#             return self.moveit_interface.srdf_model.group_end_effectors[g].parent_link
#         else :
#             return ""

#     def get_group_links(self, g) :
#         return self.moveit_interface.get_group_links(g)

#     def get_end_effector_parent_group(self, g) :
#         return self.moveit_interface.srdf_model.get_end_effector_parent_group(g)

#     def add_manipulator_group(self, g) :
#         self.moveit_interface.add_group(g, group_type="manipulator")
#         self.moveit_interface.set_display_mode(g, "all_points")

#     def print_basic_info(self) :
#         self.moveit_interface.print_basic_info()


class RobotConfig(object) :
    def __init__(self) :

        self.robot_name = ""
        self.config_package =  ""

        self.moveit_ee_groups = []
        self.end_effector_names = []
        self.end_effector_id_map = {}
        self.end_effector_name_map = {}
        self.end_effector_pose_map = {}
        self.end_effector_link_data = {}
        self.end_effector_markers = {}
        self.frame_id = "world"
        self.root_offset = geometry_msgs.msg.Pose()
        self.tf_listener = tf.TransformListener()
        self.joint_data = sensor_msgs.msg.JointState()
        self.stored_poses = {}

        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.joint_state_callback)

    def load_from_file(self, filename) :
        try:
            f = open(filename)
            self.yaml_config = yaml.load(f.read())
            f.close()
            self.print_yaml()

            self.robot_name = self.yaml_config['robot_name']
            self.config_package = str(self.yaml_config['moveit_config_package'])
            self.frame_id = self.yaml_config['frame_id']

            q = (kdl.Rotation.RPY(self.yaml_config['root_offset'][3],self.yaml_config['root_offset'][4],self.yaml_config['root_offset'][5])).GetQuaternion()
            self.root_offset.position.x = self.yaml_config['root_offset'][0]
            self.root_offset.position.y = self.yaml_config['root_offset'][1]
            self.root_offset.position.z = self.yaml_config['root_offset'][2]
            self.root_offset.orientation.x = q[0]
            self.root_offset.orientation.y = q[1]
            self.root_offset.orientation.z = q[2]
            self.root_offset.orientation.w = q[3]

            for ee in self.yaml_config['end_effector_map']:
                self.end_effector_names.append(ee['name'])
                self.end_effector_name_map[ee['id']] = ee['name']
                self.end_effector_id_map[ee['name']] = ee['id']
                p = geometry_msgs.msg.Pose()
                q = (kdl.Rotation.RPY(ee['pose_offset'][3],ee['pose_offset'][4],ee['pose_offset'][5])).GetQuaternion()
                p.position.x = float(ee['pose_offset'][0])
                p.position.y = float(ee['pose_offset'][1])
                p.position.z = float(ee['pose_offset'][2])
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]
                self.end_effector_pose_map[ee['name']] = p
            return self.configure()

        except :
            rospy.logerr("RobotConfig::load_from_file() -- error opening config file")
            return False

    # def configure(self) :
    #     self.moveit_interface = MoveItInterface(self.robot_name,self.config_package)
    #     self.root_frame = self.moveit_interface.get_planning_frame()
    #     self.moveit_ee_groups = self.moveit_interface.srdf_model.get_end_effector_groups()
    #     print "RobotConfig::configure -- moveit groups: ", self.moveit_interface.groups
    #     for g in self.end_effector_names :
    #         if not g in self.moveit_ee_groups:
    #             rospy.logerr("RobotConfig::configure() -- group ", g, " not in moveit end effector groups!")
    #             return False
    #         else :

    #             # self.end_effector_link_data[g] = EndEffectorHelper(self.robot_name, g, self.moveit_interface.get_control_frame(g), self.tf_listener)
    #             print "control frame: ", self.moveit_interface.srdf_model.group_end_effectors[g].parent_link
    #             self.end_effector_link_data[g] = EndEffectorHelper(self.robot_name, g, self.moveit_interface.srdf_model.group_end_effectors[g].parent_link, self.tf_listener)
    #             self.end_effector_link_data[g].populate_data(self.moveit_interface.get_group_links(g), self.moveit_interface.get_urdf_model())
    #             rospy.sleep(2)
    #             self.end_effector_markers[g] = self.end_effector_link_data[g].get_current_position_marker_array(scale=1.0,color=(1,1,1,0.5))
    #             pg = self.moveit_interface.srdf_model.get_end_effector_parent_group(g)
    #             if not pg == None :
    #                 print "trying to add MoveIt! group: ", pg
    #                 self.moveit_interface.add_group(pg, group_type="manipulator")
    #                 self.moveit_interface.set_display_mode(pg, "all_points")
    #             else :
    #                 print "no manipulator group found for end-effector: ", g

    #     # what do we have?
    #     self.moveit_interface.print_basic_info()
    #     return True

    def configure(self) :
        # self.moveit_interface_threads = {}
        self.moveit_interface = MoveItInterface(self.robot_name,self.config_package)
        self.root_frame = self.moveit_interface.get_planning_frame()
        self.moveit_ee_groups = self.moveit_interface.srdf_model.get_end_effector_groups()
        print "RobotConfig::configure -- moveit groups: ", self.moveit_interface.groups
        print "RobotConfig::configure -- end_effector_names: ",self.end_effector_names
        for g in self.end_effector_names :
            if not g in self.moveit_ee_groups:
                rospy.logerr("RobotConfig::configure() -- group ", g, " not in moveit end effector groups!")
                return False
            else :

                # self.end_effector_link_data[g] = EndEffectorHelper(self.robot_name, g, self.moveit_interface.get_control_frame(g), self.tf_listener)
                print "control frame: ", self.moveit_interface.srdf_model.group_end_effectors[g].parent_link
                self.end_effector_link_data[g] = EndEffectorHelper(self.robot_name, g, self.moveit_interface.srdf_model.group_end_effectors[g].parent_link, self.tf_listener)
                self.end_effector_link_data[g].populate_data(self.moveit_interface.get_group_links(g), self.moveit_interface.get_urdf_model())
                rospy.sleep(2)
                self.end_effector_markers[g] = self.end_effector_link_data[g].get_current_position_marker_array(scale=1.0,color=(1,1,1,0.5))
                pg = self.moveit_interface.srdf_model.get_end_effector_parent_group(g)
                if not pg == None :
                    print "trying to add MoveIt! group: ", pg
                    self.moveit_interface.add_group(pg, group_type="manipulator")
                    self.moveit_interface.set_display_mode(pg, "all_points")
                    # self.moveit_interface_threads[pg] = MoveItInterfaceThread(self.robot_name,self.config_package, g)
                else :
                    print "no manipulator group found for end-effector: ", g

            self.stored_poses[g] = {}
            for state_name in self.moveit_interface.get_stored_state_list(g) :
                rospy.loginfo(str("RobotConfig::configure() adding stored pose \'" + state_name + "\' to group \'" + g + "\'"))
                self.stored_poses[g][state_name] = self.moveit_interface.get_stored_group_state(g, state_name)

        # what do we have?
        self.moveit_interface.print_basic_info()
        return True

    def print_yaml(self) :
        if not self.yaml_config == None:
            print "============================="
            print "Robot Config Info: "
            print "============================="
            print " robot name: ", self.yaml_config['robot_name']
            print " root offset: ", self.yaml_config['root_offset']
            print " moveit_config: ", self.yaml_config['moveit_config_package']
            for ee in self.yaml_config['end_effector_map']:
                print "\t", "map: ", ee['name'], " --> ", ee['id']
                print "\t", "pose: ", ee['pose_offset']
            print "============================="

    def joint_state_callback(self, data) :
        self.joint_data = data

    def get_end_effector_name(self, id) :
        return self.end_effector_name_map[id]

    def get_manipulator(self, ee) :
        return self.moveit_interface.srdf_model.get_end_effector_parent_group(ee)



