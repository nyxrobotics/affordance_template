import os
import subprocess
import imp
import sys
import signal
import pprint
import json

import rospy
import zmq
import affordance_template_markers
import affordance_template_server_protobuf

from AffordanceTemplateServerCmd_pb2 import Template, Request, Response, Pose, Position, Orientation, EndEffector, Robot, EndEffectorMap, RecogObject
from visualization_msgs.msg import Marker, MarkerArray

from threading import Thread


import roslib; roslib.load_manifest("affordance_template_markers")
import rospkg

from interactive_markers.interactive_marker_server import *

from affordance_template_markers.robot_config import *
from affordance_template_markers.affordance_template import *
from affordance_template_markers.template_utilities import *
from affordance_template_markers.robot_config import *
from affordance_template_markers.recognition_object import *
import affordance_template_markers.atdf_parser


class AffordanceTemplateServer(Thread):
    """Affordance Template server."""
    def __init__(self, topic_arg=None):
        Thread.__init__(self)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.topic_arg = topic_arg
        self.robot_config = None

        self.server = InteractiveMarkerServer("affordance_template_server")

        # get path to template marker package
        self._package_path = self.getPackagePath("affordance_template_library")
        # get path to actual template source files
        self._template_path    = os.path.join(self._package_path, 'templates')
        self._robot_path    = os.path.join(self._package_path, 'robots')
        self._recognition_object_path    = os.path.join(self._package_path, 'recognition_objects')
        self.class_map, self.image_map, self.file_map, self.waypoint_map = self.getAvailableTemplates(self._template_path)
        self.robot_map = self.getRobots(self._robot_path)
        self.recognition_object_map, self.recognition_object_info, self.recognition_object_subscribers = self.getRecognitionObjects(self._recognition_object_path)

        self.running_templates = {}
        self.running_recog_objects = {}

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

    def handle_query_json(self):
        response = {}
        response['success'] = False

        try:
            # affordance_template
            response['affordance_templates'] = []
            for class_type in self.class_map.iterkeys():
                template = {}
                template['type'] = class_type
                template['image_path'] = self.image_map[class_type]
                template['waypoint_info'] = []
                for p in self.waypoint_map[class_type].keys():
                    wp = {}
                    wp['id'] = int(p)
                    wp['num_waypoints'] = self.waypoint_map[class_type][p]
                    template['waypoint_info'].append(wp)
                response['affordance_templates'].append(template)

            # robot
            response['robot'] = []
            for name in self.robot_map.iterkeys():
                robot = {}
                robot['filename'] = name
                robot['name'] = self.robot_map[name].robot_name
                robot['moveit_config_package'] = self.robot_map[name].config_package
                robot['frame_id'] = self.robot_map[name].frame_id
                robot['root_offset'] = {}
                robot['root_offset']['position'] = {}
                robot['root_offset']['position']['x'] = self.robot_map[name].root_offset.position.x
                robot['root_offset']['position']['y'] = self.robot_map[name].root_offset.position.y
                robot['root_offset']['position']['z'] = self.robot_map[name].root_offset.position.z
                robot['root_offset']['orientation'] = {}
                robot['root_offset']['orientation']['x'] = self.robot_map[name].root_offset.orientation.x
                robot['root_offset']['orientation']['y'] = self.robot_map[name].root_offset.orientation.y
                robot['root_offset']['orientation']['z'] = self.robot_map[name].root_offset.orientation.z
                robot['root_offset']['orientation']['w'] = self.robot_map[name].root_offset.orientation.w

                # end_effectors
                robot['end_effectors'] = []
                for e in self.robot_map[name].end_effector_names:
                    ee = {}
                    ee['name'] = e
                    ee['id'] = self.robot_map[name].manipulator_id_map[e]
                    ee['pose_offset'] = {}
                    ee['pose_offset']['position'] = {}
                    ee['pose_offset']['position']['x'] = self.robot_map[name].manipulator_pose_map[e].position.x
                    ee['pose_offset']['position']['y'] = self.robot_map[name].manipulator_pose_map[e].position.y
                    ee['pose_offset']['position']['z'] = self.robot_map[name].manipulator_pose_map[e].position.z
                    ee['pose_offset']['orientation'] = {}
                    ee['pose_offset']['orientation']['x'] = self.robot_map[name].manipulator_pose_map[e].orientation.x
                    ee['pose_offset']['orientation']['y'] = self.robot_map[name].manipulator_pose_map[e].orientation.y
                    ee['pose_offset']['orientation']['z'] = self.robot_map[name].manipulator_pose_map[e].orientation.z
                    ee['pose_offset']['orientation']['w'] = self.robot_map[name].manipulator_pose_map[e].orientation.w
                    robot['end_effectors'].append(ee)

                # end_effector_pose_ids
                robot['end_effector_pose_ids'] = []
                for ee_g in self.robot_map[name].end_effector_pose_map.iterkeys():
                    for ee_n in self.robot_map[name].end_effector_pose_map[ee_g].iterkeys():
                        pid = {}
                        pid['name'] = ee_n
                        pid['group'] = ee_g
                        pid['id'] =  self.robot_map[name].end_effector_pose_map[ee_g][ee_n]
                        robot['end_effector_pose_ids'].append(pid)

                response['robot'].append(robot)

            # recognition objects
            response['recognition_object'] = []
            for object_type in self.recognition_object_map.keys():
                recognition_object = {}
                recognition_object['image_path'] = self.recognition_object_info[object_type].image_path
                recognition_object['type'] = object_type
                recognition_object['launch_file'] = self.recognition_object_info[object_type].launch_file
                recognition_object['package'] = self.recognition_object_info[object_type].package
                recognition_object['topic'] = self.recognition_object_info[object_type].topic
                response['recognition_object'].append(recognition_object)

            response['waypoint_info'] = {}
            response['success'] = True
        except:
            print 'Error with query for available templates'

        return response

    def handle_query(self):
        """Return a protobuf Response containing the available templates."""
        response = Response()
        response.success = False
        # respond with available templates
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
                    ee.id =self.robot_map[name].manipulator_id_map[e]
                    ee.pose_offset.position.x = self.robot_map[name].manipulator_pose_map[e].position.x
                    ee.pose_offset.position.y = self.robot_map[name].manipulator_pose_map[e].position.y
                    ee.pose_offset.position.z = self.robot_map[name].manipulator_pose_map[e].position.z
                    ee.pose_offset.orientation.x = self.robot_map[name].manipulator_pose_map[e].orientation.x
                    ee.pose_offset.orientation.y = self.robot_map[name].manipulator_pose_map[e].orientation.y
                    ee.pose_offset.orientation.z = self.robot_map[name].manipulator_pose_map[e].orientation.z
                    ee.pose_offset.orientation.w = self.robot_map[name].manipulator_pose_map[e].orientation.w
                for ee_g in self.robot_map[name].end_effector_pose_map.iterkeys() :
                    for ee_n in self.robot_map[name].end_effector_pose_map[ee_g].iterkeys() :
                        pid = robot.end_effector_pose_ids.pose_group.add()
                        pid.name = ee_n
                        pid.group = ee_g
                        pid.id =  self.robot_map[name].end_effector_pose_map[ee_g][ee_n]

            for object_type in self.recognition_object_map.keys():
                recognition_object = response.recognition_object.add()
                recognition_object.image_path = self.recognition_object_info[object_type].image_path
                recognition_object.type = object_type
                recognition_object.launch_file = self.recognition_object_info[object_type].launch_file
                recognition_object.package = self.recognition_object_info[object_type].package
                recognition_object.topic = self.recognition_object_info[object_type].topic

            response.success = True
        except:
            print 'Error with query for available templates'

        return response

    def handle_add_json(self, request):
        response = {}
        response['success'] = False
        print "new ADD request"
        try:
            ret = False
            for template in request['affordance_template']:
                class_type = str(template['type'])
                new_id = self.getNextTemplateID(class_type)
                ret = self.addTemplate(class_type, new_id)
            response['success'] = ret
        except:
            print 'Error adding template to server'

        return response

    def handle_add(self, request):
        response = Response()
        response.success = False
        print "new ADD request"
        print request
        try:
            ret = False
            for template in request.affordance_template:
                class_type = str(template.type)
                new_id = self.getNextTemplateID(class_type)
                ret = self.addTemplate(class_type, new_id)
            response.success = ret
        except:
            print 'Error adding template to server'

        return response

    def handle_start_recognition_json(self, request):
        response = {}
        response['success'] = False
        print "new START_RECOGNITION request"
        try:
            ret = False
            for object_type in request['recognition_object']:
                object_type = str(recognition_object['type'])
                new_id = self.getNextRecogObjectID(object_type)
                ret = self.startRecognitionProcess(object_type, self.recognition_object_info[object_type].launch_file, self.recognition_object_info[object_type].package, self.recognition_object_info[object_type].topic, new_id)
            response.success = ret
        except:
            print 'Error starting recognition object from server'

        return response

    def handle_start_recognition(self, request):
        response = Response()
        response.success = False
        print "new START_RECOGNITION request"
        print request
        try:
            ret = False
            for object_type in request.recognition_object:
                object_type = str(recognition_object.type)
                new_id = self.getNextRecogObjectID(object_type)
                ret = self.startRecognitionProcess(object_type, self.recognition_object_info[object_type].launch_file, self.recognition_object_info[object_type].package, self.recognition_object_info[object_type].topic, new_id)
            response.success = ret
        except:
            print 'Error starting recognition object from server'

        return response

    def handle_running_json(self):
        response = {}
        response['success'] = False
        print "new RUNNING request"
        try:
            response['affordance_template'] = []
            for id in self.running_templates.iterkeys():
                at = {}
                at['type'] = self.running_templates[id]
                at['id'] = id
                response['affordance_template'].append(at)

            response['recognition_object'] = []
            for id in self.running_recog_objects.iterkeys():
                ro = {}
                ro['type'] = self.running_recog_objects[id]
                ro['id'] = id
                response['recognition_object']

            response['success'] = True
        except:
            print 'Error parsing running templates'
        return response

    def handle_running(self):
        response = Response()
        response.success = False
        print "new RUNNING request"
        try:

            for id in self.running_templates.iterkeys():
                at = response.affordance_template.add()
                at.type = self.running_templates[id]
                at.id = id

            for id in self.running_recog_objects.iterkeys():
                ro = response.recognition_object.add()
                ro.type = self.running_recog_objects[id]
                ro.id = id

            response.success = True
        except:
            print 'Error parsing running templates'
        return response

    def handle_kill_json(self, request):
        response = {}
        response['success'] = False
        print "new KILL request"
        try:
            for template in request['affordance_template']:
                self.removeTemplate(template['type'], template['id'])
            for obj in request['recognition_object']:
                self.removeRecognitionObject(obj['type'], obj['id'])
        except:
            print 'Error trying to kill template'
        return response

    def handle_kill(self, request):
        response = Response()
        response.success = False
        print "new KILL request"
        print request
        try:
            for template in request.affordance_template:
                self.removeTemplate(template.type, template.id)
            for obj in request.recognition_object:
                self.removeRecognitionObject(obj.type, obj.id)
        except:
            print 'Error trying to kill template'
        return response

    def handle_load_robot_json(self, request):
        response = {}
        response['success'] = False
        print "new LOAD_ROBOT request: ", request['robot']['name']
        try:
            self.robot_config = self.loadRobotFromMsg(request['robot'])
            self.robot_config.configure()
            response['success'] = True
        except:
            print 'Error trying to load robot from message'
        return response

    def handle_load_robot(self, request):
        response = Response()
        response.success = False
        print "new LOAD_ROBOT request: ", request.robot.name
        print request
        try:
            self.robot_config = self.loadRobotFromMsg(request.robot)
            self.robot_config.configure()
            response.success = True
        except:
            print 'Error trying to load robot from message'
        return response

    def handle_command_json(self, request):
        response = {}
        response['success'] = False
        print "new COMMAND request: "
        try:
            print "command: ", request['command']['type']
            print "steps: ", request['command']['steps']
            print "end_effectors: ", request['command']['end_effector']
            print "execute on plan: ", request['command']['execute']

            for template in request['affordance_template']:
                at = self.class_map[template['type']][template['id']]

                for ee in request['command']['end_effector'] :

                    command_type = request['command']['type']

                    if command_type == 'GO_TO_START':
                        idx = at.plan_path_to_waypoint(str(ee), backwards=True, steps=-999, direct=True)
                    elif command_type == 'GO_TO_END':
                        idx = at.plan_path_to_waypoint(str(ee), steps=999, direct=True)
                    elif command_type == 'PLAY_BACKWARD':
                        idx = at.plan_path_to_waypoint(str(ee), backwards=True, steps=-999, direct=False)
                    elif command_type == 'PLAY_FORWARD':
                        idx = at.plan_path_to_waypoint(str(ee), steps=999, direct=False)
                    elif command_type == 'STEP_BACKWARD':
                        idx = at.plan_path_to_waypoint(str(ee), backwards=True, steps=request['command']['steps'])
                    elif command_type == 'STEP_FORWARD':
                        idx = at.plan_path_to_waypoint(str(ee), steps=request['command']['steps'])
                    elif command_type == 'STOP':
                        at.stop(str(ee))

                    response['waypoint_info'] = []
                    if request['command']['execute']:
                        print "Executing!!!"
                        at.move_to_waypoint(str(ee), idx)
                        wp = {}
                        wp['id'] = int(at.robot_config.manipulator_id_map[str(ee)])
                        wp['num_waypoints'] = idx
                        response['waypoint_info'].append(wp)
                    else :
                        wp = {}
                        wp['id'] = int(at.robot_config.manipulator_id_map[str(ee)])
                        wp['num_waypoints'] = at.waypoint_index[wp['id']]
                        response['waypoint_info'].append(wp)


            response['success'] = True
        except:
            print 'Error trying to load robot from message'
        return response

    def handle_command(self, request):
        response = Response()
        response.success = False
        print "new COMMAND request: ", request.robot.name
        print request
        try:
            print "command: ", request.command.type
            print "steps: ", request.command.steps
            print "end_effectors: ", request.command.end_effector
            print "execute on plan: ", request.command.execute

            for template in request.affordance_template :
                at = self.class_map[template.type][template.id]

                for ee in request.command.end_effector :

                    if request.command.type == request.command.GO_TO_START :
                        idx = at.plan_path_to_waypoint(str(ee), backwards=True, steps=-999, direct=True)
                    elif request.command.type == request.command.GO_TO_END :
                        idx = at.plan_path_to_waypoint(str(ee), steps=999, direct=True)
                    elif request.command.type == request.command.PLAY_BACKWARD :
                        idx = at.plan_path_to_waypoint(str(ee), backwards=True, steps=-999, direct=False)
                    elif request.command.type == request.command.PLAY_FORWARD :
                        idx = at.plan_path_to_waypoint(str(ee), steps=999, direct=False)
                    elif request.command.type == request.command.STEP_BACKWARD :
                        idx = at.plan_path_to_waypoint(str(ee), backwards=True, steps=request.command.steps)
                    elif request.command.type == request.command.STEP_FORWARD :
                        idx = at.plan_path_to_waypoint(str(ee), steps=request.command.steps)
                    elif request.command.type == request.command.STOP :
                        at.stop(str(ee))

                    if request.command.execute :
                        print "Executing!!!"
                        at.move_to_waypoint(str(ee), idx)
                        wp = response.waypoint_info.add()
                        wp.id = int(at.robot_config.manipulator_id_map[str(ee)])
                        wp.num_waypoints = idx
                    else :
                        wp = response.waypoint_info.add()
                        wp.id = int(at.robot_config.manipulator_id_map[str(ee)])
                        wp.num_waypoints = at.waypoint_index[wp.id]


            response.success = True
        except:
            print 'Error trying to load robot from message'
        return response

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
                    # parse string to protobuf message
                    request = Request()
                    request.ParseFromString(msg)

                    # for template in request.affordance_template:
                    #     rospy.loginfo('received: ' + str(template.type))

                    response = Response()
                    response.success = False
                    # respond with available templates
                    if request.type == request.QUERY:
                        response = self.handle_query()

                    # add a new template and add it to the mapping
                    elif request.type == request.ADD:
                        response = self.handle_add(request)

                    elif request.type == request.START_RECOGNITION:
                        response = self.handle_start_recognition(request)

                    # respond with a list of the running templates on the server
                    elif request.type == request.RUNNING:
                        response = self.handle_running()

                    elif request.type == request.KILL:
                        response = self.handle_kill(request)

                    elif request.type == request.LOAD_ROBOT:
                        response = self.handle_load_robot(request)

                    elif request.type == request.COMMAND:
                        response = self.handle_command(request)

                    self.socket.send(response.SerializeToString())

                except:
                    request = json.loads(msg)
                    response = {}
                    if request['type'] == 'add':
                        response = self.handle_add_json(request)
                    # elif request['type'] == 'delete': pass
                    # elif request['type'] == 'reset': pass
                    elif request['type'] == 'query':
                        response = self.handle_query_json()
                    # elif request['type'] == 'shutdown': pass
                    # elif request['type'] == 'ping': pass
                    elif request['type'] == 'kill':
                        response = self.handle_kill_json(request)
                    elif request['type'] == 'running':
                        response = self.handle_running_json()
                    elif request['type'] == 'load_robot':
                        response = self.handle_load_robot_json(request)
                    elif request['type'] == 'command':
                        response = self.handle_command_json(request)
                    elif request['type'] == 'start_recognition':
                        response = self.handle_start_recognition_json(request)

                    self.socket.send(json.dumps(response))

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
        if class_type in self.class_map:
            at = AffordanceTemplate(self.server, instance_id, robot_config=self.robot_config)
            filename = self.file_map[class_type]
            print "Trying to load: ", filename
            at.load_from_file(filename)
            self.running_templates[instance_id] = class_type
            self.class_map[class_type][instance_id] = at  # TODO this is dumb, need to just have a local list of multiple ATs
            print("templateServer.addTemplate: adding template " + str(class_type))
            return True

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

        print self.recognition_object_subscribers.keys()
        print self.recognition_object_subscribers[object_type].keys()
        self.recognition_object_subscribers[object_type][instance_id] = rospy.Subscriber(topic, MarkerArray, self.recognitionObjectCallback)

        import subprocess
        cmd = str('roslaunch ' + package + ' ' + launch_file)
        proc = subprocess.Popen([cmd], shell=True)
        pid = proc.pid # <--- access `pid` attribute to get the pid of the child process.

        self.recognition_object_map[object_type][instance_id] = proc

        return True

    def getNextTemplateID(self, class_type):
        ids = self.class_map[class_type].keys()
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
        import glob, yaml
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
            rospy.logerr("AffordanceTemplateServer::loadRobotFromMsg() -- error parsing robot protobuf file")
            return None

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
