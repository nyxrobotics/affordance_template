import json

class JSONInterface(object):
    def __init__(self, server):
        self.server = server

    def parse_request(self, msg):
        """Parse an incoming message as JSON and handle it.

        @type msg Stringified JSON
        @param msg A JSON request made to the Affordance Template Server.

        @rtype response Stringified JSON
        @returns The response to the request.
        """
        request = json.loads(msg)
        response = {}
        if request['type'] == 'add':
            response = self.handle_add(request)
        # elif request['type'] == 'delete': pass
        # elif request['type'] == 'reset': pass
        elif request['type'] == 'query':
            response = self.handle_query()
        # elif request['type'] == 'shutdown': pass
        # elif request['type'] == 'ping': pass
        elif request['type'] == 'kill':
            response = self.handle_kill(request)
        elif request['type'] == 'running':
            response = self.handle_running()
        elif request['type'] == 'load_robot':
            response = self.handle_load_robot(request)
        elif request['type'] == 'command':
            response = self.handle_command(request)
        elif request['type'] == 'start_recognition':
            response = self.handle_start_recognition(request)

        return json.dumps(response)

    def handle_query(self):
        response = {}
        response['success'] = False

        try:
            # affordance_template
            response['affordance_template'] = []
            for class_type in self.server.class_map.iterkeys():
                template = {}
                template['type'] = class_type
                template['image_path'] = self.server.image_map[class_type]
                template['waypoint_info'] = []
                for p in self.server.waypoint_map[class_type].keys():
                    wp = {}
                    wp['id'] = int(p)
                    wp['num_waypoints'] = self.server.waypoint_map[class_type][p]
                    template['waypoint_info'].append(wp)
                response['affordance_template'].append(template)

            # robot
            response['robot'] = []
            for name in self.server.robot_map.iterkeys():
                robot = {}
                robot['filename'] = name
                robot['name'] = self.server.robot_map[name].robot_name
                robot['moveit_config_package'] = self.server.robot_map[name].config_package
                robot['frame_id'] = self.server.robot_map[name].frame_id
                robot['root_offset'] = {}
                robot['root_offset']['position'] = {}
                robot['root_offset']['position']['x'] = self.server.robot_map[name].root_offset.position.x
                robot['root_offset']['position']['y'] = self.server.robot_map[name].root_offset.position.y
                robot['root_offset']['position']['z'] = self.server.robot_map[name].root_offset.position.z
                robot['root_offset']['orientation'] = {}
                robot['root_offset']['orientation']['x'] = self.server.robot_map[name].root_offset.orientation.x
                robot['root_offset']['orientation']['y'] = self.server.robot_map[name].root_offset.orientation.y
                robot['root_offset']['orientation']['z'] = self.server.robot_map[name].root_offset.orientation.z
                robot['root_offset']['orientation']['w'] = self.server.robot_map[name].root_offset.orientation.w

                # end_effectors
                robot['end_effectors'] = []
                for e in self.server.robot_map[name].end_effector_names:
                    ee = {}
                    ee['name'] = e
                    ee['id'] = self.server.robot_map[name].manipulator_id_map[e]
                    ee['pose_offset'] = {}
                    ee['pose_offset']['position'] = {}
                    ee['pose_offset']['position']['x'] = self.server.robot_map[name].manipulator_pose_map[e].position.x
                    ee['pose_offset']['position']['y'] = self.server.robot_map[name].manipulator_pose_map[e].position.y
                    ee['pose_offset']['position']['z'] = self.server.robot_map[name].manipulator_pose_map[e].position.z
                    ee['pose_offset']['orientation'] = {}
                    ee['pose_offset']['orientation']['x'] = self.server.robot_map[name].manipulator_pose_map[e].orientation.x
                    ee['pose_offset']['orientation']['y'] = self.server.robot_map[name].manipulator_pose_map[e].orientation.y
                    ee['pose_offset']['orientation']['z'] = self.server.robot_map[name].manipulator_pose_map[e].orientation.z
                    ee['pose_offset']['orientation']['w'] = self.server.robot_map[name].manipulator_pose_map[e].orientation.w
                    robot['end_effectors'].append(ee)

                # end_effector_pose_ids
                robot['end_effector_pose_ids'] = []
                for ee_g in self.server.robot_map[name].end_effector_pose_map.iterkeys():
                    for ee_n in self.server.robot_map[name].end_effector_pose_map[ee_g].iterkeys():
                        pid = {}
                        pid['name'] = ee_n
                        pid['group'] = ee_g
                        pid['id'] =  self.server.robot_map[name].end_effector_pose_map[ee_g][ee_n]
                        robot['end_effector_pose_ids'].append(pid)

                response['robot'].append(robot)

            # recognition objects
            response['recognition_object'] = []
            for object_type in self.server.recognition_object_map.keys():
                recognition_object = {}
                recognition_object['image_path'] = self.server.recognition_object_info[object_type].image_path
                recognition_object['type'] = object_type
                recognition_object['launch_file'] = self.server.recognition_object_info[object_type].launch_file
                recognition_object['package'] = self.server.recognition_object_info[object_type].package
                recognition_object['topic'] = self.server.recognition_object_info[object_type].topic
                response['recognition_object'].append(recognition_object)

            response['waypoint_info'] = {}
            response['success'] = True
        except:
            print 'Error with query for available templates'

        return response

    def handle_add(self, request):
        response = {}
        response['success'] = False
        print "new ADD request"
        try:
            ret = False
            for template in request['affordance_template']:
                class_type = str(template['type'])
                new_id = self.server.getNextTemplateID(class_type)
                ret = self.server.addTemplate(class_type, new_id)
            response['success'] = ret
        except:
            print 'Error adding template to server'

        return response

    def handle_start_recognition(self, request):
        response = {}
        response['success'] = False
        print "new START_RECOGNITION request"
        try:
            ret = False
            for object_type in request['recognition_object']:
                object_type = str(recognition_object['type'])
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
        response = {}
        response['success'] = False
        print "new RUNNING request"
        try:
            response['affordance_template'] = []
            for id in self.server.running_templates.iterkeys():
                at = {}
                at['type'] = self.server.running_templates[id]
                at['id'] = id
                response['affordance_template'].append(at)

            response['recognition_object'] = []
            for id in self.server.running_recog_objects.iterkeys():
                ro = {}
                ro['type'] = self.server.running_recog_objects[id]
                ro['id'] = id
                response['recognition_object']

            response['success'] = True
        except:
            print 'Error parsing running templates'
        return response

    def handle_kill(self, request):
        response = {}
        response['success'] = False
        print "new KILL request"
        try:
            for template in request.get('affordance_template', []):
                self.server.removeTemplate(template['type'], template['id'])
            for obj in request.get('recognition_object', []):
                self.server.removeRecognitionObject(obj['type'], obj['id'])
        except:
            print 'Error trying to kill template'
        return response

    def handle_load_robot(self, request):
        response = {}
        response['success'] = False
        print "new LOAD_ROBOT request: ", request['robot']['name']
        try:
            self.server.robot_config = self.server.loadRobotFromMsg(request['robot'])
            self.server.robot_config.configure()
            response['success'] = True
        except:
            print 'Error trying to load robot from message'
        return response

    def handle_command(self, request):
        response = {}
        response['success'] = False
        print "new COMMAND request: "
        print request
        try:
            print "command: ", request['command']['type']
            print "steps: ", request['command']['steps']
            print "end_effectors: ", request['command']['end_effector']
            print "execute on plan: ", request['command']['execute']

            for template in request['affordance_template']:
                at = self.server.class_map[template['type']][template['id']]

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
