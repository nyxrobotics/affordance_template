from AffordanceTemplateServerCmd_pb2 import Template, Request, Response, Pose, Position, Orientation, EndEffector, Robot, EndEffectorMap, RecogObject, TrajectoryInfo, WaypointInfo

import rospy

from affordance_template_msgs.msg import *  
from affordance_template_msgs.srv import *  

class ServiceInterface(object):

    def __init__(self, server):
        
        rospy.loginfo("ServiceInterface() starting")
        # the affordance template server
        self.server = server

        # services
        self.robot_info_service =      rospy.Service('/affordance_template_server/get_robots', GetRobotConfigInfo, self.handle_robot_request)
        self.template_info_service =   rospy.Service('/affordance_template_server/get_templates', GetAffordanceTemplateConfigInfo, self.handle_template_request)
        self.load_robot_service =      rospy.Service('/affordance_template_server/load_robot', LoadRobotConfig, self.handle_load_robot)
        self.add_template_service =    rospy.Service('/affordance_template_server/add_template', AddAffordanceTemplate, self.handle_add_template)
        self.delete_template_service = rospy.Service('/affordance_template_server/delete_template', DeleteAffordanceTemplate, self.handle_kill)
        self.get_running_service =     rospy.Service('/affordance_template_server/get_running', GetRunningAffordanceTemplates, self.handle_running)
        self.command_service =         rospy.Service('/affordance_template_server/command', AffordanceTemplateCommand, self.handle_command)


    def handle_robot_request(self, request) :
        rospy.loginfo(str("ServiceInterface::handle_robot_request() -- requested robot info " + request.name))
        response = GetRobotConfigInfoResponse()
        if request.name and request.name in self.server.robot_map.keys() :
            response.robots.append(self.server.robot_map[request.name].robot_config)
        else :
            for r in self.server.robot_map.keys() :
                response.robots.append(self.server.robot_map[r].robot_config)
        return response

    def handle_load_robot(self, request):
        rospy.loginfo(str("ServiceInterface::handle_load_robot() -- load request for robot " + request.robot_config.name))
        response = LoadRobotConfigResponse()
        response.status = False
        try:
            if self.server.robot_interface.configured :
                self.server.robot_interface.tear_down() 

            if request.filename :
                self.server.robot_interface.load_from_msg(request.filename)
            else :
                self.server.robot_interface.load_from_msg(request.robot_config)

            response.status = self.server.robot_interface.configure()
        except:
            rospy.logerror("ServiceInterface::handle_load_robot()  -- Error trying to load robot from message")
        return response

    def handle_template_request(self, request) :
        rospy.loginfo(str("ServiceInterface::handle_template_request() -- requested template info " + request.name))
        response = GetAffordanceTemplateConfigInfoResponse()
        if request.name and request.name in self.server.at_data.class_map.keys() :
            class_type = request.name
            at_config = AffordanceTemplateConfig()
            at_config.type = class_type
            at_config.image_path = self.server.at_data.image_map[class_type]
            for t in self.server.at_data.traj_map[class_type] :
                wp_traj = WaypointTrajectory()
                wp_traj.name = t
                for p in self.server.at_data.waypoint_map[(class_type,t)].keys() :
                    wp = WaypointInfo()
                    wp.id = int(p)
                    wp.num_waypoints = self.server.at_data.waypoint_map[(class_type,t)][p]
                    wp_traj.waypoint_info.append(wp)
                at_config.trajectory_info.append(wp_traj)
            response.templates.append(at_config)
        else :
            for class_type in self.server.at_data.class_map.keys():
                at_config = AffordanceTemplateConfig()
                at_config.type = class_type
                at_config.image_path = self.server.at_data.image_map[class_type]
                for t in self.server.at_data.traj_map[class_type] :
                    wp_traj = WaypointTrajectory()
                    wp_traj.name = t
                    for p in self.server.at_data.waypoint_map[(class_type,t)].keys() :
                        wp = WaypointInfo()
                        wp.id = int(p)
                        wp.num_waypoints = self.server.at_data.waypoint_map[(class_type,t)][p]
                        wp_traj.waypoint_info.append(wp)
                    at_config.trajectory_info.append(wp_traj)
                response.templates.append(at_config)
        return response

    def handle_add_template(self, request):
        rospy.loginfo(str("ServiceInterface::handle_add_template() -- adding template " + request.class_type))
        response = AddAffordanceTemplateResponse()
        response.status = False
        try:
            pid = self.server.get_next_template_id(request.class_type)
            response.id = pid
            response.status = self.server.add_template(request.class_type, response.id)
        except:
            rospy.logerr("ServiceInterface::handle_add_template() -- error adding template to server")
        return response

    def handle_running(self, request):
        rospy.loginfo("ServiceInterface::handle_running()")
        response = GetRunningAffordanceTemplatesResponse()
        try:
            for t in self.server.at_data.class_map.keys():
                for i in self.server.at_data.class_map[t].keys():
                    response.templates.append(t + ":" + str(i))
            response.templates.sort()
        except:
            rospy.logerr("ServiceInterface::handle_running() -- error getting running templates")
        return response

    def handle_kill(self, request):
        rospy.loginfo(str("ServiceInterface::handle_kill() -- killing template " + request.class_type + "[" + str(request.id) + "]"))
        response = DeleteAffordanceTemplateResponse()
        response.status = False
        try:
            response.status = self.server.remove_template(request.class_type, request.id)
        except:
            rospy.logerr("ServiceInterface::handle_kill() -- error deleting requested templates")
        return response

    def handle_command(self, request):
        rospy.loginfo(str("ServiceInterface::handle_command() -- new command request for template " + request.type + "[" + str(request.id) + "]: " + str(request.command)))        
        response = AffordanceTemplateCommandResponse()
        response.status = False

        try:
            
            idx = {}
            at = self.server.at_data.class_map[request.type][int(request.id)]
            # plan first
            for ee in request.end_effectors:

                if not at.trajectory_has_ee(at.current_trajectory, ee): 
                    rospy.logwarn(str("ServiceInterface::handle_command() -- " + ee + " not in trajectory, can't plan"))
                    continue

                if request.command == request.GO_TO_START :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), backwards=True, steps=-999, direct=True)
                elif request.command == request.GO_TO_END :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), steps=999, direct=True)
                elif request.command == request.PLAY_BACKWARD :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), backwards=True, steps=-999, direct=False)
                elif request.command == request.PLAY_FORWARD :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), steps=999, direct=False)
                elif request.command == request.STEP_BACKWARD :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), backwards=True, steps=request.steps)
                elif request.command == request.STEP_FORWARD :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), steps=request.steps)
                elif request.command == request.STOP :
                    at.stop(str(ee))

                rospy.loginfo(str("ServiceInterface::handle_command() -- done planning path for " + ee))

            # execute after
            for ee in request.end_effectors :

                if not at.trajectory_has_ee(at.current_trajectory, ee): 
                    rospy.logwarn(str("ServiceInterface::handle_command() -- " + ee + " not in trajectory, can't execute"))
                    continue
                if request.execute_on_plan:        
                    rospy.loginfo(str("ServiceInterface::handle_command() -- executing path for " + ee + ", precomputed: " + str(request.execute_precomputed_plan)))
                    at.move_to_waypoint(str(ee), idx[ee])
                    wp = WaypointInfo()
                    wp.id = int(at.robot_interface.manipulator_id_map[str(ee)])
                    wp.waypoint_index = idx[ee]
                    wp.num_waypoints = at.waypoint_max[at.current_trajectory][wp.id]+1
                    response.waypoint_info.append(wp)
                else :
                    wp = WaypointInfo()
                    wp.id = int(at.robot_interface.manipulator_id_map[str(ee)])
                    wp.waypoint_index = int(at.waypoint_index[at.current_trajectory][wp.id])
                    wp.num_waypoints = at.waypoint_max[at.current_trajectory][wp.id]+1
                    response.waypoint_info.append(wp)                  

                rospy.loginfo(str("ServiceInterface::handle_command() -- done execution path for " + ee))
            response.status = True
        except:
            rospy.logerr(str("ServiceInterface::handle_command() -- error performing command!!"))
        return response




    # def parse_request(self, msg):
    #     """Parse an incoming message as Protobuf and handle it.

    #     @type msg Stringified Protobuf
    #     @param msg A Protobuf request made to the Affordance Template Server.

    #     @rtype response Stringified Protobuf
    #     @returns The response to the request.
    #     """
    #     request = Request()
    #     request.ParseFromString(msg)

    #     # for template in request.affordance_template:
    #     #     rospy.loginfo('received: ' + str(template.type))

    #     response = Response()
    #     response.success = False

    #     # respond with available templates
    #     if request.type == request.QUERY:
    #         response = self.handle_query()

    #     # add a new template and add it to the mapping
    #     elif request.type == request.ADD:
    #         response = self.handle_add(request)

    #     elif request.type == request.START_RECOGNITION:
    #         response = self.handle_start_recognition(request)

    #     # respond with a list of the running templates on the server
    #     elif request.type == request.RUNNING:
    #         response = self.handle_running()

    #     elif request.type == request.KILL:
    #         response = self.handle_kill(request)

    #     elif request.type == request.LOAD_ROBOT:
    #         response = self.handle_load_robot(request)

    #     elif request.type == request.COMMAND:
    #         response = self.handle_command(request)

    #     return response.SerializeToString()

    # def handle_query(self):
    #     """Return a protobuf Response containing the available templates."""
    #     response = Response()
    #     response.success = False
    #     # respond with available templates
    #     print "new QUERY request"

    #     try:
    #         for class_type in self.server.at_data.class_map.iterkeys():
    #             template = response.affordance_template.add()
    #             template.type = class_type
    #             template.image_path = self.server.at_data.image_map[class_type]
                
    #             for t in self.server.at_data.traj_map[class_type] :
    #                 traj = template.trajectory_info.add()
    #                 traj.name = t
    #                 for p in self.server.at_data.waypoint_map[(class_type,t)].keys() :
    #                     wp = traj.waypoint_info.add()
    #                     wp.id = int(p)
    #                     wp.num_waypoints = self.server.at_data.waypoint_map[(class_type,t)][p]

    #         for name in self.server.robot_map.iterkeys():
    #             robot = response.robot.add()
    #             robot.filename = name
    #             robot.name = self.server.robot_map[name].robot_name
    #             robot.moveit_config_package = self.server.robot_map[name].config_package
    #             robot.frame_id = self.server.robot_map[name].frame_id
    #             robot.root_offset.position.x = self.server.robot_map[name].root_offset.position.x
    #             robot.root_offset.position.y = self.server.robot_map[name].root_offset.position.y
    #             robot.root_offset.position.z = self.server.robot_map[name].root_offset.position.z
    #             robot.root_offset.orientation.x = self.server.robot_map[name].root_offset.orientation.x
    #             robot.root_offset.orientation.y = self.server.robot_map[name].root_offset.orientation.y
    #             robot.root_offset.orientation.z = self.server.robot_map[name].root_offset.orientation.z
    #             robot.root_offset.orientation.w = self.server.robot_map[name].root_offset.orientation.w
                
    #             for e in self.server.robot_map[name].end_effector_names:
                    
    #                 ee = robot.end_effectors.end_effector.add()
    #                 ee.name = e
    #                 ee.id =self.server.robot_map[name].manipulator_id_map[e]

    #                 ee.pose_offset.position.x = self.server.robot_map[name].manipulator_pose_map[e].position.x
    #                 ee.pose_offset.position.y = self.server.robot_map[name].manipulator_pose_map[e].position.y
    #                 ee.pose_offset.position.z = self.server.robot_map[name].manipulator_pose_map[e].position.z
    #                 ee.pose_offset.orientation.x = self.server.robot_map[name].manipulator_pose_map[e].orientation.x
    #                 ee.pose_offset.orientation.y = self.server.robot_map[name].manipulator_pose_map[e].orientation.y
    #                 ee.pose_offset.orientation.z = self.server.robot_map[name].manipulator_pose_map[e].orientation.z
    #                 ee.pose_offset.orientation.w = self.server.robot_map[name].manipulator_pose_map[e].orientation.w

    #                 ee.tool_offset.position.x = self.server.robot_map[name].tool_offset_map[e].position.x
    #                 ee.tool_offset.position.y = self.server.robot_map[name].tool_offset_map[e].position.y
    #                 ee.tool_offset.position.z = self.server.robot_map[name].tool_offset_map[e].position.z
    #                 ee.tool_offset.orientation.x = self.server.robot_map[name].tool_offset_map[e].orientation.x
    #                 ee.tool_offset.orientation.y = self.server.robot_map[name].tool_offset_map[e].orientation.y
    #                 ee.tool_offset.orientation.z = self.server.robot_map[name].tool_offset_map[e].orientation.z
    #                 ee.tool_offset.orientation.w = self.server.robot_map[name].tool_offset_map[e].orientation.w

    #             for ee_g in self.server.robot_map[name].end_effector_pose_map.iterkeys() :
    #                 for ee_n in self.server.robot_map[name].end_effector_pose_map[ee_g].iterkeys() :
    #                     pid = robot.end_effector_pose_ids.pose_group.add()
    #                     pid.name = ee_n
    #                     pid.group = ee_g
    #                     pid.id =  self.server.robot_map[name].end_effector_pose_map[ee_g][ee_n]
    #             robot.gripper_service = self.server.robot_map[name].gripper_service
                

    #         for object_type in self.server.recognition_object_map.keys():
    #             recognition_object = response.recognition_object.add()
    #             recognition_object.image_path = self.server.recognition_object_info[object_type].image_path
    #             recognition_object.type = object_type
    #             recognition_object.launch_file = self.server.recognition_object_info[object_type].launch_file
    #             recognition_object.package = self.server.recognition_object_info[object_type].package
    #             recognition_object.topic = self.server.recognition_object_info[object_type].topic

    #         response.success = True
    #     except:
    #         print 'Error with query for available templates'

    #     return response

    # def handle_add(self, request):
    #     response = Response()
    #     response.success = False
    #     print "new ADD request"
    #     try:
    #         ret = False
    #         for template in request.affordance_template:
    #             class_type = str(template.type)
    #             new_id = self.server.getNextTemplateID(class_type)
    #             ret = self.server.addTemplate(class_type, new_id)
    #         response.success = ret
    #     except:
    #         print 'Error adding template to server'

    #     return response

    # def handle_start_recognition(self, request):
    #     response = Response()
    #     response.success = False
    #     print "new START_RECOGNITION request"
    #     # print request
    #     try:
    #         ret = False
    #         for recognition_object in request.recognition_object:
    #             object_type = str(recognition_object.type)
    #             obj = self.server.recognition_object_info[object_type]
    #             new_id = self.server.getNextRecogObjectID(object_type)
    #             ret = self.server.startRecognitionProcess(
    #                     object_type,
    #                     obj.launch_file,
    #                     obj.package,
    #                     obj.topic,
    #                     new_id)
    #         response.success = ret
    #     except:
    #         print 'Error starting recognition object from server'

    #     return response

    # def handle_running(self):
    #     response = Response()
    #     response.success = False
    #     print "new RUNNING request"
    #     try:

    #         for t in self.server.at_data.class_map.keys():
    #             for id in self.server.at_data.class_map[t].keys():
    #                 at = response.affordance_template.add()
    #                 at.type = t
    #                 at.id = id
    #         # for id in self.server.running_templates.iterkeys():
    #         #     at = response.affordance_template.add()
    #         #     at.type = self.server.running_templates[id]
    #         #     at.id = id

    #         for id in self.server.running_recog_objects.iterkeys():
    #             ro = response.recognition_object.add()
    #             ro.type = self.server.running_recog_objects[id]
    #             ro.id = id

    #         response.success = True
    #     except:
    #         print 'Error parsing running templates'
    #     return response

    # def handle_kill(self, request):
    #     response = Response()
    #     response.success = False
    #     print "new KILL request"
    #     # print request
    #     try:
    #         for template in request.affordance_template:
    #             self.server.removeTemplate(template.type, template.id)
    #         for obj in request.recognition_object:
    #             self.server.removeRecognitionObject(obj.type, obj.id)
    #         response.success = True
    #     except:
    #         print 'Error trying to kill template'
    #     print response
    #     return response

    # def handle_load_robot(self, request):
    #     response = Response()
    #     response.success = False
    #     print "new LOAD_ROBOT request: ", request.robot.name
    #     # print request
    #     try:
    #         self.server.robot_config = self.server.loadRobotFromMsg(request.robot)
    #         self.server.robot_config.configure()
    #         response.success = True
    #     except:
    #         print 'Error trying to load robot from message'
    #     return response

    # def handle_command(self, request):
    #     response = Response()
    #     response.success = False
    #     print "new COMMAND request: ", request.robot.name
    #     # print request
    #     try:
    #         print "command: ", request.command.type
    #         print "steps: ", request.command.steps
    #         print "end_effectors: ", request.command.end_effector
    #         print "execute on plan: ", request.command.execute


    #         for template in request.affordance_template :
    #             print "template type: ", template.type
    #             print "template id:   ", template.id
                
    #             at = self.server.at_data.class_map[template.type][template.id]
    #             # plan first
    #             for ee in request.command.end_effector :

    #                 print "checking to plan for end effector: ", ee
    #                 if not at.trajectory_has_ee(at.current_trajectory, ee): 
    #                     print " not in trajectory"
    #                     continue

    #                 if request.command.type == request.command.GO_TO_START :
    #                     idx = at.plan_path_to_waypoint(str(ee), backwards=True, steps=-999, direct=True)
    #                 elif request.command.type == request.command.GO_TO_END :
    #                     idx = at.plan_path_to_waypoint(str(ee), steps=999, direct=True)
    #                 elif request.command.type == request.command.PLAY_BACKWARD :
    #                     idx = at.plan_path_to_waypoint(str(ee), backwards=True, steps=-999, direct=False)
    #                 elif request.command.type == request.command.PLAY_FORWARD :
    #                     idx = at.plan_path_to_waypoint(str(ee), steps=999, direct=False)
    #                 elif request.command.type == request.command.STEP_BACKWARD :
    #                     idx = at.plan_path_to_waypoint(str(ee), backwards=True, steps=request.command.steps)
    #                 elif request.command.type == request.command.STEP_FORWARD :
    #                     idx = at.plan_path_to_waypoint(str(ee), steps=request.command.steps)
    #                 elif request.command.type == request.command.STOP :
    #                     at.stop(str(ee))

    #             print "done planning..."


    #             # execute after
    #             for ee in request.command.end_effector :

    #                 print "checking to execute end effector: ", str(ee)
    #                 if not at.trajectory_has_ee(at.current_trajectory, ee): 
    #                     print " not in trajectory"
    #                     continue

    #                 if request.command.execute :
    #                     print "Executing!!!"
    #                     at.move_to_waypoint(str(ee), idx)
    #                     wp = response.waypoint_info.add()
    #                     wp.id = int(at.robot_config.manipulator_id_map[str(ee)])
    #                     wp.num_waypoints = idx
    #                 else :
    #                     wp = response.waypoint_info.add()
    #                     wp.id = int(at.robot_config.manipulator_id_map[str(ee)])
    #                     wp.num_waypoints = at.waypoint_index[at.current_trajectory][wp.id]

    #                 print "done executing..."

    #         response.success = True
    #     except:
    #         print 'Error trying to parse COMMAND message'
    #     return response
