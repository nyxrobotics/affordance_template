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
    def __init__(self, hostname='alien', port=6789):
        rospy.init_node('atros_control')
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect('tcp://%s:%d'%(hostname, port))

        self.afford_pub = rospy.Publisher('/affordances', AffordanceList, latch=True)
        self.srv1 = rospy.Service('/change_affordance', ChangeAffordance, self.change_affordance)
        self.srv2 = rospy.Service('/affordance_cmd', AffordanceCommand, self.affordance_command)

        self.get_info()

    def get_info(self):
        info = self.send({'type': 'query'})
        self.templates = info['affordance_template']

        a = AffordanceList()
        a.affordances = [j['type'] for j in self.templates]
        print a
        self.afford_pub.publish(a)

        self.robots = {}
        for m in info['robot']:
            self.robots[ m['filename'] ] = m
        self.name = get_robot_name()
        self.chosen_robot = self.robots[self.name + '.yaml']

        self.send( {'type': 'load_robot', 'robot': self.chosen_robot} )

    def send(self, command):
        s = json.dumps(command)
        self.socket.send(s)
        return json.loads(self.socket.recv())

    def change_affordance(self, req):
        if req.add:
            self.send({'type': 'add', 'affordance_template': [{'type': req.affordance}]})
            return ChangeAffordanceResponse(True)
        else:
            return ChangeAffordanceResponse(False)

    def affordance_command(self, req):
        templates = []
        for a,i in zip(req.affordances, req.ids):
            templates.append({'type': a, 'id': i})

        self.send({'type': 'command', 'affordance_template': templates,
                   'command': {'type': req.type, 'end_effector': req.end_effectors,
                   'steps': req.steps, 'execute': req.execute}})

        return AffordanceCommand(True)

atrc = ATRosControl()
rospy.spin()
