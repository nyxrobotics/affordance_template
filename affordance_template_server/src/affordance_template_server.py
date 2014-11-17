import os
import subprocess
import imp
import sys
import signal
import zmq
import yaml
import json
        
import rospy
import affordance_template_markers
import affordance_template_server_protobuf

from json_interface import JSONInterface
from protobuf_interface import ProtobufInterface
from visualization_msgs.msg import Marker, MarkerArray

from threading import Thread

import roslib; roslib.load_manifest("affordance_template_markers")
import rospkg

from interactive_markers.interactive_marker_server import *

from affordance_template_markers.robot_config import *
from affordance_template_markers.affordance_template import *
from affordance_template_markers.template_utilities import *
from affordance_template_markers.recognition_object import *
from affordance_template_markers.affordance_template_data import *
# import affordance_template_markers.atdf_parser


class AffordanceTemplateServer(Thread):
    """Affordance Template server."""
    def __init__(self, topic_arg=None):
        Thread.__init__(self)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.topic_arg = topic_arg
        self.robot_config = None
        self.structure = {}

        self.interfaces = {}
        self.interfaces['json'] = JSONInterface(self)
        self.interfaces['protobuf'] = ProtobufInterface(self)

        self.server = InteractiveMarkerServer("affordance_template_server")

        # get path to template marker package
        self._package_path = self.getPackagePath("affordance_template_library")
        # get path to actual template source files
        self._template_path    = os.path.join(self._package_path, 'templates')
        self._robot_path    = os.path.join(self._package_path, 'robots')
        self._recognition_object_path  = os.path.join(self._package_path, 'recognition_objects')
        self.at_data = self.getAvailableTemplates(self._template_path)
        # self.class_map, self.image_map, self.file_map, self.waypoint_map = self.getAvailableTemplates(self._template_path)
        self.robot_map = self.getRobots(self._robot_path)
        self.recognition_object_map, self.recognition_object_info, self.recognition_object_subscribers = self.getRecognitionObjects(self._recognition_object_path)

        self.running_templates = {}
        self.running_recog_objects = {}

        # put something here to laod robot config to debug faster
        # self.robot_config = RobotConfig()
        # self.robot_config.load_from_file('/home/swhart/ros_workspace/catkin_workspace/src/affordance_templates/affordance_template_library/robots/r2.yaml')
        # # self.robot_config.configure()
        # self.addTemplate('Wheel', 0)


    @property
    def resource_path(self):
        """Return the path to the RViz images."""
        return str(self._resource_path)

    @property
    def plugin_description(self):   
        """Return the path to the plugin_description.xml associated with the server."""
        return str(self._plugin_description[0])

    @property
    def template_path(self):
        """Return the path to the template nodes."""
        return str(self._template_path)

    def configureServer(self):
        """Configure the server's ZMQ ports and ROS subscribers."""

        # TODO: add dynamic reconfigure to change subscriber topic
        # configure ROS subscriber for bootstrapping templates
        sub = rospy.Subscriber("/foo", Marker, self.markerSub)

        # init zmq to port 6789
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind("tcp://*:6789")
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        print "Affordance Template Server started on port 6789"

        self.recognition_object_update_flags = {}

    def run(self):

        self.configureServer()
        # process mapping
        while not rospy.is_shutdown():

            # poll every second
            try:
                socks = self.poller.poll(1000)
            except zmq.ZMQError, e:
                break

            for sock, state in socks:

                if state != zmq.POLLIN:
                    continue

                msg = sock.recv()

                try:
                    response = self.interfaces['protobuf'].parse_request(msg)
                except:
                    response = self.interfaces['json'].parse_request(msg)

                self.socket.send(response)

    def removeTemplate(self, class_type, instance_id):
        """Stop a template process and remove it from the server's map.

        @type class_type string
        @param class_type The class type e.g. "Wheel", "Car", etc.

        @type instance_id int
        @param instance_id The ID of this instance.

        @rtype bool
        @returns True if process was stopped/removed.
        """
        if class_type in self.at_data.class_map and instance_id in self.at_data.class_map[class_type]:
            self.at_data.class_map[class_type][instance_id].terminate()
            if self.running_templates[instance_id] == class_type:
                del self.running_templates[instance_id]
            del self.at_data.class_map[class_type][instance_id]

    def removeRecognitionObject(self, object_type, instance_id):
        """Stop a template process and remove it from the server's map.

        @type class_type string
        @param object_type The class type e.g. "Wheel", "Car", etc.

        @type instance_id int
        @param instance_id The ID of this instance.

        @rtype bool
        @returns True if process was stopped/removed.
        """
        if object_type in self.recognition_object_map and instance_id in self.recognition_object_map[object_type]:
            self.recognition_object_map[object_type][instance_id].terminate()
            del self.recognition_object_map[object_type][instance_id]

    def addTemplate(self, class_type, instance_id):
        """Start a template process using subprocess.Popen.

        @type class_type string
        @param class_type The class type e.g. "Wheel", "Car", etc.

        @type instance_id int
        @param instance_id The ID of this instance.

        @rtype int
        @returns The Popen object started by the server.
        """
        if class_type in self.at_data.class_map:
            print "Trying to add ", class_type
            print "sending in RC: ", self.robot_config
            at = AffordanceTemplate(self.server, instance_id, robot_config=self.robot_config)
            print "Created new AT class"
            filename = self.at_data.file_map[class_type]
            print "Trying to load: ", filename
            at.load_from_file(filename)
            self.running_templates[instance_id] = class_type
            self.at_data.class_map[class_type][instance_id] = at  # TODO this is dumb, need to just have a local list of multiple ATs
            rospy.loginfo(str("AffordanceTemplateServer::addTemplate: adding template " + str(class_type)))
            return True
        else :
            rospy.logerr(str("AffordanceTemplateServer:: ERROR -- no template type " + str(class_type)))
            return False

    def startRecognitionProcess(self, object_type, launch_file, package, topic, instance_id):
        """Start a template process using subprocess.Popen.

        @type object_type string
        @param object_type The class type e.g. "handle", "torus", etc.

        @type launch_file string
        @param launch_file The launch file to run

        @type package string
        @param package The package the launch file is in

        @rtype int
        @returns The Popen object started by the server.
        """
        if not self.getPackagePath(package) :
            rospy.loginfo("AffordanceTemplateServer::startRecognitionProcess(" + object_type + ") No package found: " + package)
            return False
        # should check if launch file exists as well here

        self.running_recog_objects[instance_id] = object_type

        # print self.recognition_object_subscribers.keys()
        # print self.recognition_object_subscribers[object_type].keys()
        self.recognition_object_subscribers[object_type][instance_id] = rospy.Subscriber(topic, MarkerArray, self.recognitionObjectCallback)

        import subprocess
        cmd = str('roslaunch ' + package + ' ' + launch_file)
        proc = subprocess.Popen([cmd], shell=True)
        pid = proc.pid # <--- access `pid` attribute to get the pid of the child process.

        self.recognition_object_map[object_type][instance_id] = proc

        return True

    def getNextTemplateID(self, class_type):
        ids = self.at_data.class_map[class_type].keys()
        i = 0
        while True:
            if i in ids:
                i += 1
            else:
                return i

    def getNextRecogObjectID(self, object_type):
        ids = self.recognition_object_map[object_type].keys()
        i = 0
        while True:
            if i in ids:
                i += 1
            else:
                return i

    def getPackagePath(self, pkg):
        """Return the path to the ROS package."""
        import rospkg
        try:
            rp = rospkg.RosPack()
            return rp.get_path(pkg)
        except:
            # print 'No package found: ', pkg
            return False

    def getTemplatePath(self):
        """Return the path to the template nodes."""
        import rospkg
        rp = rospkg.RosPack()
        return os.path.join(rp.get_path('affordance_template_markers'), 'src', 'affordance_template_markers')

    def getPluginDescription(self, pkg):
        """Return the plugin_description.xml for a ROS package."""
        import rospkg
        rp = rospkg.RosPack()
        man = rp.get_manifest(pkg)
        return man.get_export(pkg, 'plugin')

    def getAvailableTemplates(self, path):
        """Parse affordance_templates manifest for available classes."""
        # if os.path.exists(manifest):

        from xml.etree.ElementTree import ElementTree
        import glob

        at_data = AffordanceTemplateCollection()
        at_data.class_map = {}
        at_data.traj_map = {}
        at_data.image_map = {}
        at_data.file_map = {}
        at_data.waypoint_map = {}

        os.chdir(path)

        import json
        for atfn in glob.glob("*.json") :
            
            try :
                atf = open(atfn).read()
                structure = json.loads(atf)

                at_name = str(structure['name'])
                image = str(structure['image'])
                at_data.class_map[at_name] = {}
                at_data.traj_map[at_name] = []
                at_data.image_map[at_name] = image
                at_data.file_map[at_name] = os.path.join(path,atfn)

                print "Found AT File: ", at_name
                for traj in structure['end_effector_trajectory'] :
                    traj_name = str(traj['name'])
                    # print traj_name
                    at_data.traj_map[at_name].append(traj_name)
                    key = (at_name,traj_name)
                    at_data.waypoint_map[key] = {}
                    
                    for ee_group in traj['end_effector_group'] :
                        ee_id = ee_group['id']
                        wp_id = len(ee_group['end_effector_waypoint'])
                        at_data.waypoint_map[key][ee_id] = wp_id
                        print " adding ", wp_id, " waypoints for ee ", ee_id, " in ", key
            except :
                rospy.logwarn(str("AffordanceTemplateServer::getAvailableTemplates() -- error parsing " + atfn))
            # self.structure[at_name] = structure

        # return traj_map, class_map, image_map, file_map, waypoint_map
        return at_data

    def loadFromFile(self, filename) :
        print filename
        atf = open(filename).read()
        print atf
        return json.loads(atf)

    def getRobots(self, path):
        """Parse parses available robots from fs."""
        robot_map = {}
        import glob
        os.chdir(path)
        for r in glob.glob("*.yaml") :
            print "found robot yaml: ", r
            rc = RobotConfig()
            rc.load_from_file(r)

            # check to see if package actually is "installed" and only add it if it is.
            if self.getPackagePath(rc.config_package) :
                robot_map[r] = rc
            else :
                rospy.loginfo("AffordanceTemplateServer::getRobots(" + r + ") not found, not adding...")

        return robot_map

    def getRecognitionObjects(self, path):
        """Parse parses available robots from fs."""
        recognition_object_map = {}
        recognition_object_info = {}
        recognition_object_subscribers = {}
        import glob
        os.chdir(path)
        for r in glob.glob("*.yaml") :
            print "found recognition_object yaml: ", r
            ro = RecognitionObject()
            ro.load_from_file(r)
            recognition_object_map[ro.type] = {}
            recognition_object_info[ro.type] = ro
            recognition_object_subscribers[ro.type] = {}

        return recognition_object_map, recognition_object_info, recognition_object_subscribers

    def loadRobotFromMsg(self, robot) :
        r = RobotConfig()
        try:
            r.robot_name = robot.name
            r.config_package = robot.moveit_config_package
            r.frame_id = robot.frame_id
            print "loading robot: " , r.robot_name

            r.root_offset.position.x = robot.root_offset.position.x
            r.root_offset.position.y = robot.root_offset.position.y
            r.root_offset.position.z = robot.root_offset.position.z
            r.root_offset.orientation.x = robot.root_offset.orientation.x
            r.root_offset.orientation.y = robot.root_offset.orientation.y
            r.root_offset.orientation.z = robot.root_offset.orientation.z
            r.root_offset.orientation.w = robot.root_offset.orientation.w

            r.end_effector_names = []
            r.end_effector_name_map = {}
            r.manipulator_id_map = {}
            r.manipulator_pose_map = {}
            r.tool_offset_map = {}

            r.gripper_service = robot.gripper_service

            for ee in robot.end_effectors.end_effector:
                r.end_effector_names.append(ee.name)
                r.end_effector_name_map[ee.id] = ee.name
                r.manipulator_id_map[ee.name] = ee.id
                
                p = geometry_msgs.msg.Pose()
                p.position.x = ee.pose_offset.position.x
                p.position.y = ee.pose_offset.position.y
                p.position.z = ee.pose_offset.position.z
                p.orientation.x = ee.pose_offset.orientation.x
                p.orientation.y = ee.pose_offset.orientation.y
                p.orientation.z = ee.pose_offset.orientation.z
                p.orientation.w = ee.pose_offset.orientation.w
                r.manipulator_pose_map[ee.name] = p

                t = geometry_msgs.msg.Pose()
                t.position.x = ee.tool_offset.position.x
                t.position.y = ee.tool_offset.position.y
                t.position.z = ee.tool_offset.position.z
                t.orientation.x = ee.tool_offset.orientation.x
                t.orientation.y = ee.tool_offset.orientation.y
                t.orientation.z = ee.tool_offset.orientation.z
                t.orientation.w = ee.tool_offset.orientation.w
                r.tool_offset_map[ee.name] = t

            for ee_pid in robot.end_effector_pose_ids.pose_group:
                if not ee_pid.group in r.end_effector_pose_map :
                    r.end_effector_pose_map[ee_pid.group] = {}
                if not ee_pid.group in r.end_effector_id_map :
                    r.end_effector_id_map[ee_pid.group] = {}
                # print "*********************************ee[", ee_pid.group, "] adding group [", ee_pid.name, "] with id [", ee_pid.id, "]"
                r.end_effector_pose_map[ee_pid.group][ee_pid.name] = int(ee_pid.id)
                r.end_effector_id_map[ee_pid.group][int(ee_pid.id)] = ee_pid.name

            # print "done!"
            return r

        except :
            rospy.logerr("AffordanceTemplateServer::loadRobotFromMsg() -- error parsing robot protobuf file - trying JSON")
            r.robot_name = robot['name']
            r.config_package = robot['moveit_config_package']
            r.frame_id = robot['frame_id']
            print "loading robot: " , r.robot_name

            r.root_offset.position.x = robot['root_offset']['position']['x']
            r.root_offset.position.y = robot['root_offset']['position']['y']
            r.root_offset.position.z = robot['root_offset']['position']['z']
            r.root_offset.orientation.x = robot['root_offset']['orientation']['x']
            r.root_offset.orientation.y = robot['root_offset']['orientation']['y']
            r.root_offset.orientation.z = robot['root_offset']['orientation']['z']
            r.root_offset.orientation.w = robot['root_offset']['orientation']['w']

            r.end_effector_names = []
            r.end_effector_name_map = {}
            r.manipulator_id_map = {}
            r.manipulator_pose_map = {}
            r.tool_offset_map = {}

            r.gripper_service = robot['gripper_service']
            
            for ee in robot['end_effectors']:
                r.end_effector_names.append(ee['name'])
                r.end_effector_name_map[ee['id']] = ee['name']
                r.manipulator_id_map[ee['name']] = ee['id']
                
                p = geometry_msgs.msg.Pose()
                p.position.x = ee['pose_offset']['position']['x']
                p.position.y = ee['pose_offset']['position']['y']
                p.position.z = ee['pose_offset']['position']['z']
                p.orientation.x = ee['pose_offset']['orientation']['x']
                p.orientation.y = ee['pose_offset']['orientation']['y']
                p.orientation.z = ee['pose_offset']['orientation']['z']
                p.orientation.w = ee['pose_offset']['orientation']['w']
                r.manipulator_pose_map[ee['name']] = p

                t = geometry_msgs.msg.Pose()
                t.position.x = ee['tool_offset']['position']['x']
                t.position.y = ee['tool_offset']['position']['y']
                t.position.z = ee['tool_offset']['position']['z']
                t.orientation.x = ee['tool_offset']['orientation']['x']
                t.orientation.y = ee['tool_offset']['orientation']['y']
                t.orientation.z = ee['tool_offset']['orientation']['z']
                t.orientation.w = ee['tool_offset']['orientation']['w']
                r.tool_offset_map[ee['name']] = t

            for ee_pid in robot['end_effector_pose_ids']:
                if not ee_pid['group'] in r.end_effector_pose_map :
                    r.end_effector_pose_map[ee_pid['group']] = {}
                if not ee_pid['group'] in r.end_effector_id_map :
                    r.end_effector_id_map[ee_pid['group']] = {}
                # print "*********************************ee[", ee_pid.group, "] adding group [", ee_pid.name, "] with id [", ee_pid.id, "]"
                r.end_effector_pose_map[ee_pid['group']][ee_pid['name']] = int(ee_pid['id'])
                r.end_effector_id_map[ee_pid['group']][int(ee_pid['id'])] = ee_pid['name']

            # print "done!"
            return r
            # return None

    def loadRecognitionObjectFromMsg(self, recognition_object) :
        r = RecogntionObject()

        print "creating new robot from pb message"
        try:

            r.robot_name = robot.name
            r.config_package = robot.moveit_config_package
            r.frame_id = robot.frame_id
            print "loading robot: " , r.robot_name

            r.root_offset.position.x = robot.root_offset.position.x
            r.root_offset.position.y = robot.root_offset.position.y
            r.root_offset.position.z = robot.root_offset.position.z
            r.root_offset.orientation.x = robot.root_offset.orientation.x
            r.root_offset.orientation.y = robot.root_offset.orientation.y
            r.root_offset.orientation.z = robot.root_offset.orientation.z
            r.root_offset.orientation.w = robot.root_offset.orientation.w

            r.end_effector_names = []
            r.end_effector_name_map = {}
            r.manipulator_id_map = {}
            r.manipulator_pose_map = {}
            r.tool_offset_map = {}

            for ee in robot.end_effectors.end_effector:
                r.end_effector_names.append(ee.name)
                r.end_effector_name_map[ee.id] = ee.name
                r.manipulator_id_map[ee.name] = ee.id
                
                p = geometry_msgs.msg.Pose()
                p.position.x = ee.pose_offset.position.x
                p.position.y = ee.pose_offset.position.y
                p.position.z = ee.pose_offset.position.z
                p.orientation.x = ee.pose_offset.orientation.x
                p.orientation.y = ee.pose_offset.orientation.y
                p.orientation.z = ee.pose_offset.orientation.z
                p.orientation.w = ee.pose_offset.orientation.w
                r.manipulator_pose_map[ee.name] = p

                t = geometry_msgs.msg.Pose()
                t.position.x = ee.tool_offset.position.x
                t.position.y = ee.tool_offset.position.y
                t.position.z = ee.tool_offset.position.z
                t.orientation.x = ee.tool_offset.orientation.x
                t.orientation.y = ee.tool_offset.orientation.y
                t.orientation.z = ee.tool_offset.orientation.z
                t.orientation.w = ee.tool_offset.orientation.w
                r.tool_offset_map[ee.name] = t

            print "done!"
            return r

        except :
            rospy.logerr("AffordanceTemplateServer::loadRobotFromMsg() -- error parsing robot protobuf file")
            return None

    def getRawName(self, name):
        """Parse the class_name and return just the type."""
        if '/' in name:
            return name.rsplit('/',1)[1]
        if '::' in name:
            return name.rsplit('::',1)[1]

    def markerSub(self, data):
        print 'markerSub: ', data

    def recognitionObjectCallback(self, data) :
        for m in data.markers :
            id = m.ns + str(m.id)
            if not id in self.recognition_object_update_flags :
                print "creating a new AT for a recognized object"
                roat = AffordanceTemplate(self.server, m.id, m.ns, robot_config=self.robot_config)
                roat.load_from_marker(m)
                self.recognition_object_update_flags[id] = False
