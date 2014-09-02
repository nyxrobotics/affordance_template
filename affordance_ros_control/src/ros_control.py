import rospy
import zmq, json
from affordance_ros_control.msg import *
from affordance_ros_control.srv import *

def get_robot_name():
    desc = rospy.get_param('/robot_description', '')
    if 'name=' in desc:
        i = desc.index('name=')+6
        i2 = desc.index(desc[i-1], i)
        return desc[i:i2]
    return None

class ATRosControl:
    def __init__(self, hostname='localhost', port=6789):
        rospy.init_node('atros_control')
        self.blocking = False
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect('tcp://%s:%d'%(hostname, port))

        self.afford_pub = rospy.Publisher('/affordances', AffordanceList, latch=True)
        self.srv1 = rospy.Service('/change_affordance', ChangeAffordance, self.change_affordance)
        self.srv2 = rospy.Service('/affordance_cmd', AffordanceCommand, self.affordance_command)

        self.get_info()

    def get_info(self):
        info = self.send({'type': 'query'})
        self.templates = {}
        a = AffordanceList()
        
        for template in info['affordance_template']:
            t = template['type']
            self.templates[ t ] = template
            a.affordances.append(t)
        self.afford_pub.publish(a)

        self.robots = {}
        for m in info['robot']:
            self.robots[ m['filename'] ] = m
        self.name = get_robot_name()
        self.chosen_robot = self.robots[self.name + '.yaml']

        self.send( {'type': 'load_robot', 'robot': self.chosen_robot} )

    def send(self, command):
        s = json.dumps(command)

        while self.blocking:
            rospy.sleep(.1)
        self.blocking = True
        self.socket.send(s)
        s = json.loads(self.socket.recv())
        self.blocking = False
        return s

    def change_affordance(self, req):
        if req.add:
            self.send({'type': 'add', 'affordance_template': [{'type': req.affordance}]})
            
            ee = self.chosen_robot['end_effectors']
            ee_ids = [a['id'] for a in ee]
            ee_names = [a['name'] for a in ee]
            
            ee_info = self.templates[req.affordance]['waypoint_info']
            n_pts = [None] * len(ee_ids)
            
            for wp in ee_info:
                i = ee_ids.index( wp['id'] )
                n_pts[i] = wp['num_waypoints']
                        
            return ChangeAffordanceResponse(True, ee_ids, ee_names, n_pts)
        else:
            return ChangeAffordanceResponse(False, [], [])

    def affordance_command(self, req):
        templates = []
        for a,i in zip(req.affordances, req.ids):
            templates.append({'type': a, 'id': i})

        resp = self.send({'type': 'command', 'affordance_template': templates,
                         'command': {'type': req.type, 'end_effector': req.end_effectors,
                         'steps': req.steps, 'execute': req.execute}})
        ids = []
        wps = []
        for m in resp['waypoint_info']:
            ids.append( m['id'] )
            wps.append( m['num_waypoints'] )
        
        return AffordanceCommandResponse(True, ids, wps)

atrc = ATRosControl()
rospy.spin()
