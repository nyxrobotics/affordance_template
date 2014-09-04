from AffordanceTemplateServerCmd_pb2 import Template, Request, Response, Pose, Position, Orientation, EndEffector, Robot, EndEffectorMap, RecogObject

class ProtobufInterface(object):
    def __init__(self, server):
        self.server = server

    def parse_request(self, msg):
        """Parse an incoming message as Protobuf and handle it.

        @type msg Stringified Protobuf
        @param msg A Protobuf request made to the Affordance Template Server.

        @rtype response Stringified Protobuf
        @returns The response to the request.
        """
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

        return response.SerializeToString()

    def handle_query(self):
        """Return a protobuf Response containing the available templates."""
        response = Response()
        response.success = False
        # respond with available templates
        print "new QUERY request"

        try:
            for class_type in self.server.class_map.iterkeys():
                template = response.affordance_template.add()
                template.type = class_type
                template.image_path = self.server.image_map[class_type]
                for p in self.server.waypoint_map[class_type].keys() :
                    wp = template.waypoint_info.add()
                    wp.id = int(p)
                    wp.num_waypoints = self.server.waypoint_map[class_type][p]

            for name in self.server.robot_map.iterkeys():
                robot = response.robot.add()
                robot.filename = name
                robot.name = self.server.robot_map[name].robot_name
                robot.moveit_config_package = self.server.robot_map[name].config_package
                robot.frame_id = self.server.robot_map[name].frame_id
                robot.root_offset.position.x = self.server.robot_map[name].root_offset.position.x
                robot.root_offset.position.y = self.server.robot_map[name].root_offset.position.y
                robot.root_offset.position.z = self.server.robot_map[name].root_offset.position.z
                robot.root_offset.orientation.x = self.server.robot_map[name].root_offset.orientation.x
                robot.root_offset.orientation.y = self.server.robot_map[name].root_offset.orientation.y
                robot.root_offset.orientation.z = self.server.robot_map[name].root_offset.orientation.z
                robot.root_offset.orientation.w = self.server.robot_map[name].root_offset.orientation.w
                for e in self.server.robot_map[name].end_effector_names:
                    ee = robot.end_effectors.end_effector.add()
                    ee.name = e
                    ee.id =self.server.robot_map[name].manipulator_id_map[e]
                    ee.pose_offset.position.x = self.server.robot_map[name].manipulator_pose_map[e].position.x
                    ee.pose_offset.position.y = self.server.robot_map[name].manipulator_pose_map[e].position.y
                    ee.pose_offset.position.z = self.server.robot_map[name].manipulator_pose_map[e].position.z
                    ee.pose_offset.orientation.x = self.server.robot_map[name].manipulator_pose_map[e].orientation.x
                    ee.pose_offset.orientation.y = self.server.robot_map[name].manipulator_pose_map[e].orientation.y
                    ee.pose_offset.orientation.z = self.server.robot_map[name].manipulator_pose_map[e].orientation.z
                    ee.pose_offset.orientation.w = self.server.robot_map[name].manipulator_pose_map[e].orientation.w
                for ee_g in self.server.robot_map[name].end_effector_pose_map.iterkeys() :
                    for ee_n in self.server.robot_map[name].end_effector_pose_map[ee_g].iterkeys() :
                        pid = robot.end_effector_pose_ids.pose_group.add()
                        pid.name = ee_n
                        pid.group = ee_g
                        pid.id =  self.server.robot_map[name].end_effector_pose_map[ee_g][ee_n]

            for object_type in self.server.recognition_object_map.keys():
                recognition_object = response.recognition_object.add()
                recognition_object.image_path = self.server.recognition_object_info[object_type].image_path
                recognition_object.type = object_type
                recognition_object.launch_file = self.server.recognition_object_info[object_type].launch_file
                recognition_object.package = self.server.recognition_object_info[object_type].package
                recognition_object.topic = self.server.recognition_object_info[object_type].topic

            response.success = True
        except:
            print 'Error with query for available templates'

        return response

    def handle_add(self, request):
        response = Response()
        response.success = False
        print "new ADD request"
        # print request
        try:
            ret = False
            for template in request.affordance_template:
                class_type = str(template.type)
                new_id = self.server.getNextTemplateID(class_type)
                ret = self.server.addTemplate(class_type, new_id)
            response.success = ret
        except:
            print 'Error adding template to server'

        return response

    def handle_start_recognition(self, request):
        response = Response()
        response.success = False
        print "new START_RECOGNITION request"
        # print request
        try:
            ret = False
            for recognition_object in request.recognition_object:
                object_type = str(recognition_object.type)
                obj = self.server.recognition_object_info[object_type]
                new_id = self.server.getNextRecogObjectID(object_type)
                ret = self.server.startRecognitionProcess(
                        object_type,
                        obj.launch_file,
                        obj.package,
                        obj.topic,
                        new_id)
            response.success = ret
        except:
            print 'Error starting recognition object from server'

        return response

    def handle_running(self):
        response = Response()
        response.success = False
        print "new RUNNING request"
        try:

            for t in self.server.class_map.keys():
                for id in self.server.class_map[t].keys():
                    at = response.affordance_template.add()
                    at.type = t
                    at.id = id
            # for id in self.server.running_templates.iterkeys():
            #     at = response.affordance_template.add()
            #     at.type = self.server.running_templates[id]
            #     at.id = id

            for id in self.server.running_recog_objects.iterkeys():
                ro = response.recognition_object.add()
                ro.type = self.server.running_recog_objects[id]
                ro.id = id

            response.success = True
        except:
            print 'Error parsing running templates'
        return response

    def handle_kill(self, request):
        response = Response()
        response.success = False
        print "new KILL request"
        # print request
        try:
            for template in request.affordance_template:
                self.server.removeTemplate(template.type, template.id)
            for obj in request.recognition_object:
                self.server.removeRecognitionObject(obj.type, obj.id)
            response.success = True
        except:
            print 'Error trying to kill template'
        print response
        return response

    def handle_load_robot(self, request):
        response = Response()
        response.success = False
        print "new LOAD_ROBOT request: ", request.robot.name
        # print request
        try:
            self.server.robot_config = self.server.loadRobotFromMsg(request.robot)
            self.server.robot_config.configure()
            response.success = True
        except:
            print 'Error trying to load robot from message'
        return response

    def handle_command(self, request):
        response = Response()
        response.success = False
        print "new COMMAND request: ", request.robot.name
        # print request
        try:
            print "command: ", request.command.type
            print "steps: ", request.command.steps
            print "end_effectors: ", request.command.end_effector
            print "execute on plan: ", request.command.execute

            for template in request.affordance_template :
                at = self.server.class_map[template.type][template.id]

                # plan first
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

                # execute after
                for ee in request.command.end_effector :

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
