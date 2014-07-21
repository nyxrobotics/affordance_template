import os
import subprocess
import imp
import sys
import signal
import pprint

import rospy
import zmq
import affordance_template_markers
import affordance_template_server_protobuf

from AffordanceTemplateServerCmd_pb2 import Template, Request, Response, Pose, Position, Orientation, EndEffector, Robot, EndEffectorMap
from visualization_msgs.msg import Marker

from threading import Thread


import roslib; roslib.load_manifest("affordance_template_markers")
import rospkg

from interactive_markers.interactive_marker_server import *

from affordance_template_markers.robot_config import *
from affordance_template_markers.affordance_template import *
from affordance_template_markers.template_utilities import *
from affordance_template_markers.robot_config import *
import affordance_template_markers.atdf_parser


class AffordanceTemplateServer(Thread):
    """Affordance Template server."""
    def __init__(self, topic_arg=None):
        Thread.__init__(self)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.topic_arg = topic_arg
        self.robot_config = None

        self.server = InteractiveMarkerServer("affordance_template_server")

        self.at = None

        # get exported templates from affordance_templates package
        # self._plugin_description = self.getPluginDescription("affordance_template_markers")
        # self._resource_path = self.getPackagePath("affordance_template_markers")
        # self._resource_path = os.path.join(self._resource_path, 'resources', 'rviz')
        # self._template_path = self.getTemplatePath()
        # self.class_map = self.getAvailableTemplates(self.plugin_description)

        # get path to template marker package
        self._package_path = self.getPackagePath("affordance_template_library")
        # get path to actual template source files
        self._template_path    = os.path.join(self._package_path, 'templates')
        self._robot_path    = os.path.join(self._package_path, 'robots')
        # # get path to resources
        # self._resource_path = os.path.join(self._package_path, 'resources', 'rviz')
        # # find plugin_description.xml file that holds all templates
        # filename = self._package_path + "/plugin_description.xml"
        # # call getAvailableTemplates to parse out templates
        self.class_map, self.image_map, self.file_map, self.waypoint_map = self.getAvailableTemplates(self._template_path)
        self.robot_map = self.getRobots(self._robot_path)

        # import rospkg
        # rp = rospkg.RosPack()
        # robot_config_file = rp.get_path('affordance_template_library') + "/robots/r2.yaml"
        # self.robot_config = None
        # try:
        #     # load RobotConfig from yaml and moveit package
        #     self.robot_config = RobotConfig()
        #     if self.robot_config.load_from_file(robot_config_file) :
        #         print "configuring R2:"
        #         self.robot_config.configure()
        # except rospy.ROSInterruptException:
        #     pass

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

                # parse string to protobuf message
                request = Request()
                request.ParseFromString(msg)

                # for template in request.affordance_template:
                #     rospy.loginfo('received: ' + str(template.type))

                response = Response()
                response.success = False
                # respond with available templates
                if request.type == request.QUERY:
                    print "new QUERY request"

                    try:
                        for class_type in self.class_map.iterkeys():
                            template = response.affordance_template.add()
                            template.type = class_type
                            template.image_path = self.image_map[class_type]
                            for p in self.waypoint_map[class_type].keys() :
                                wp = template.waypoint_info.add()
                                wp.id = int(p)
                                wp.num_waypoints = self.waypoint_map[class_type][p]

                        for name in self.robot_map.iterkeys():
                            robot = response.robot.add()
                            robot.filename = name
                            robot.name = self.robot_map[name].robot_name
                            robot.moveit_config_package = self.robot_map[name].config_package
                            robot.frame_id = self.robot_map[name].frame_id
                            robot.root_offset.position.x = self.robot_map[name].root_offset.position.x
                            robot.root_offset.position.y = self.robot_map[name].root_offset.position.y
                            robot.root_offset.position.z = self.robot_map[name].root_offset.position.z
                            robot.root_offset.orientation.x = self.robot_map[name].root_offset.orientation.x
                            robot.root_offset.orientation.y = self.robot_map[name].root_offset.orientation.y
                            robot.root_offset.orientation.z = self.robot_map[name].root_offset.orientation.z
                            robot.root_offset.orientation.w = self.robot_map[name].root_offset.orientation.w
                            for e in self.robot_map[name].end_effector_names:
                                ee = robot.end_effectors.end_effector.add()
                                ee.name = e
                                ee.id =self.robot_map[name].end_effector_id_map[e]
                                ee.pose_offset.position.x = self.robot_map[name].end_effector_pose_map[e].position.x
                                ee.pose_offset.position.y = self.robot_map[name].end_effector_pose_map[e].position.y
                                ee.pose_offset.position.z = self.robot_map[name].end_effector_pose_map[e].position.z
                                ee.pose_offset.orientation.x = self.robot_map[name].end_effector_pose_map[e].orientation.x
                                ee.pose_offset.orientation.y = self.robot_map[name].end_effector_pose_map[e].orientation.y
                                ee.pose_offset.orientation.z = self.robot_map[name].end_effector_pose_map[e].orientation.z
                                ee.pose_offset.orientation.w = self.robot_map[name].end_effector_pose_map[e].orientation.w

                        response.success = True
                    except:
                        print 'Error with query for available templates'

                # add a new template and add it to the mapping
                elif request.type == request.ADD:
                    print "new ADD request"
                    try:
                        for template in request.affordance_template:
                            class_type = str(template.type)
                            new_id = self.getNextID(class_type)
                            ret = self.addTemplate(class_type, new_id)
                            print "ret: ", ret
                            # if popen != None:
                            #     self.class_map[class_type][new_id] = popen
                        response.success = ret
                    except:
                        print 'Error adding template to server'

                # respond with a list of the running templates on the server
                elif request.type == request.RUNNING:
                    print "new RUNNING request"
                    try:
                        # push the running templates into a temporary list so we can sort them
                        running_templates = []
                        for template in self.class_map.iterkeys():
                            running_templates.append(template)
                        running_templates.sort()

                        for class_type in self.class_map.iterkeys():
                            for id_ in self.class_map[class_type].iterkeys():
                                template = response.affordance_template.add()
                                template.type = class_type
                                # template.id = id_

                        response.success = True
                    except:
                        print 'Error parsing running templates'

                elif request.type == request.KILL:
                    print "new KILL request"
                    try:
                        for template in request.affordance_template:
                            self.removeTemplate(template.type, template.id)
                    except:
                        print 'Error trying to kill template'

                elif request.type == request.LOAD_ROBOT:
                    print "new LOAD_ROBOT request: ", request.robot.name
                    try:
                        self.robot_config = self.loadRobotFromMsg(request.robot)
                        self.robot_config.configure()
                        response.success = True
                    except:
                        print 'Error trying to load robot from message'

                elif request.type == request.COMMAND:
                    print "new COMMAND request: ", request.robot.name
                    try:
                        print "command: ", request.command.type
                        print "steps: ", request.command.steps
                        print "end_effectors: ", request.command.end_effector
                        print "execute on plan: ", request.command.execute
                        for ee in request.command.end_effector :

                            if request.command.type == request.command.GO_TO_START :
                                idx = self.at.plan_path_to_waypoint(str(ee), backwards=True, steps=10000, direct=True)
                            elif request.command.type == request.command.GO_TO_END :
                                idx = self.at.plan_path_to_waypoint(str(ee), steps=10000, direct=True)
                            elif request.command.type == request.command.PLAY_BACKWARD :
                                idx = self.at.plan_path_to_waypoint(str(ee), backwards=True, steps=request.command.steps)
                            elif request.command.type == request.command.PLAY_FORWARD :
                                idx = self.at.plan_path_to_waypoint(str(ee), steps=request.command.steps)
                            elif request.command.type == request.command.STEP_BACKWARD :
                                idx = self.at.plan_path_to_waypoint(str(ee), backwards=True, steps=request.command.steps)
                            elif request.command.type == request.command.STEP_FORWARD :
                                idx = self.at.plan_path_to_waypoint(str(ee), steps=request.command.steps)
                            elif request.command.type == request.command.PAUSE :
                                pass

                            if request.command.execute :
                                print "Executing!!!"
                                self.at.move_to_waypoint(str(ee), idx)
                                wp = response.waypoint_info.add()
                                wp.id = int(self.at.robot_config.end_effector_id_map[str(ee)])
                                wp.num_waypoints = idx
                            else :
                                wp = response.waypoint_info.add()
                                wp.id = int(self.at.robot_config.end_effector_id_map[str(ee)])
                                wp.num_waypoints = self.at.waypoint_index[wp.id]


                        response.success = True
                    except:
                        print 'Error trying to load robot from message'

                self.socket.send(response.SerializeToString())

    def removeTemplate(self, class_type, instance_id):
        """Stop a template process and remove it from the server's map.

        @type class_type string
        @param class_type The class type e.g. "Wheel", "Car", etc.

        @type instance_id int
        @param instance_id The ID of this instance.

        @rtype bool
        @returns True if process was stopped/removed.
        """
        if class_type in self.class_map and instance_id in self.class_map[class_type]:
            self.class_map[class_type][instance_id].terminate()
            del self.class_map[class_type][instance_id]

    def addTemplate(self, class_type, instance_id):
        """Start a template process using subprocess.Popen.

        @type class_type string
        @param class_type The class type e.g. "Wheel", "Car", etc.

        @type instance_id int
        @param instance_id The ID of this instance.

        @rtype int
        @returns The Popen object started by the server.
        """
        if class_type in self.class_map:
            self.at = AffordanceTemplate(self.server, instance_id, robot_config=self.robot_config)
            filename = self.file_map[class_type]
            print "Trying to load: ", filename
            self.at.load_from_file(filename)
            print "Success?"
            # filename = os.path.join(self.template_path, ''.join([class_type, '.py']))
            # if self.topic_arg is None:
            #     args = [filename, str(instance_id), "True"]
            # else:
            #     args = [filename, str(instance_id)]
            # args = [filename, str(instance_id)]
            print args
            print("templateServer.addTemplate: adding template " + str(class_type))
            return True


    def getNextID(self, class_type):
        ids = self.class_map[class_type].keys()
        i = 0
        while True:
            if i in ids:
                i += 1
            else:
                return i

    def getPackagePath(self, pkg):
        """Return the path to the ROS package."""
        import rospkg
        rp = rospkg.RosPack()
        return rp.get_path(pkg)

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

        class_map = {}
        image_map = {}
        file_map = {}

        waypoint_map = {}

        os.chdir(path)
        for atf in glob.glob("*.atdf") :
            print atf
            structure = affordance_template_markers.atdf_parser.AffordanceTemplateStructure.from_file(atf)

            class_map[structure.name] = {}
            image_map[structure.name] = structure.image
            file_map[structure.name] = os.path.join(path,atf)
            waypoint_map[structure.name] = {}

            for wp in structure.end_effector_waypoints.end_effector_waypoints :
                if not wp.end_effector in waypoint_map[structure.name] :
                    waypoint_map[structure.name][wp.end_effector] = 1
                else :
                    if (int(wp.id)+1) > waypoint_map[structure.name][wp.end_effector] :
                        waypoint_map[structure.name][wp.end_effector] = int(wp.id)+1

        return class_map, image_map, file_map, waypoint_map

    def getRobots(self, path):
        """Parse parses available robots from fs."""

        robot_map = {}
        import glob, yaml
        os.chdir(path)
        for r in glob.glob("*.yaml") :
            print "found robot yaml: ", r
            rc = RobotConfig()
            rc.load_from_file(r)
            robot_map[r] = rc
        return robot_map

    def loadRobotFromMsg(self, robot) :
        r = RobotConfig()

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
            r.end_effector_id_map = {}
            r.end_effector_pose_map = {}

            for ee in robot.end_effectors.end_effector:
                r.end_effector_names.append(ee.name)
                r.end_effector_name_map[ee.id] = ee.name
                r.end_effector_id_map[ee.name] = ee.id
                p = geometry_msgs.msg.Pose()
                p.position.x = ee.pose_offset.position.x
                p.position.y = ee.pose_offset.position.y
                p.position.z = ee.pose_offset.position.z
                p.orientation.x = ee.pose_offset.orientation.x
                p.orientation.y = ee.pose_offset.orientation.y
                p.orientation.z = ee.pose_offset.orientation.z
                p.orientation.w = ee.pose_offset.orientation.w
                r.end_effector_pose_map[ee.name] = p

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