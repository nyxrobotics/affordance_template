import zmq
import json
import argparse
import sys
import pprint

commands = {}

# load r2.yaml RobotConfig on the server
commands['load_robot'] = {
    'type': 'load_robot',
    'robot': {
        'filename': 'r2.yaml',
        'name': 'r2',
        'moveit_config_package': 'r2_moveit_config',
        'frame_id': 'r2/robot_base',
        'root_offset': {
            'position': {
                'x': 0.40000000596,
                'y': 0.0,
                'z': -0.20000000298
            },
        'orientation': {
                'x': 0.999999701977,
                'y': 0.0,
                'z': 0.0,
                'w': 0.000796326727141
            }
        },
        'end_effectors': [ {
            'name': 'left_hand',
            'id': 0,
            'pose_offset': {
                'position': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0
                },
                'orientation': {
                  'x': 0.706825196743,
                  'y': 0.0,
                  'z': 0.0,
                  'w': 0.707388281822
                }
            }
        },
        {
            'name': 'right_hand',
            'id': 1,
            'pose_offset': {
                'position': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0
                },
                'orientation': {
                  'x': -0.706825196743,
                  'y': 0.0,
                  'z': 0.0,
                  'w': 0.707388281822
                }
            }
        }],
        'end_effector_pose_ids': [
            {
              'group': "left_hand",
              'name': "Left Hand Close",
              'id': 2
            },
            {
              'group': "left_hand",
              'name': "Left Hand Open",
              'id': 0
            },
            {
              'group': "left_hand",
              'name': "Left Hand Thumb Open",
              'id': 1
            },
            {
              'group': "right_hand",
              'name': "Right Hand Close",
              'id': 2
            },
            {
              'group': "right_hand",
              'name': "Right Hand Open",
              'id': 0
            },
            {
              'group': "right_hand",
              'name': "Right Hand Thumb Open",
              'id': 1
            }
        ]
    }
}

# get available templates the server knows about
commands['query'] = {'type': 'running'}

# kill the Wheel template (actual deletion not yet supported on server)
commands['kill'] = {'type': 'kill', 'affordance_template': [{'type': 'Wheel', 'id': 0}]}

# start handle recognition
commands['start_recognition'] = {
  'type': 'start_recognition',
  'recognition_object': [{'type': 'handle'}]
}

# step forward
# TODO: in command['type'], supported types are
# STEP_FORWARD
# STEP_BACKWARD
# PLAY_FORWARD
# PLAY_BACKWARD
# GO_TO_START
# GO_TO_END
# STOP
commands['step_forward'] = {
  'type': 'command',
  'affordance_template': [{'type': 'Wheel', 'id': 0}],
  'command': {
    'type': 'STEP_FORWARD',
    'end_effector': ['left_hand', 'right_hand'],
    'steps': 1,
    'execute': True}
}

# add Wheel AT
commands['add'] = {'type': 'add', 'affordance_template': [{'type': 'Wheel'}]}

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ats sample client')
    parser.add_argument('-c, --command', dest='command', default='query', help='{}'.format(str(commands.keys())))
    args = parser.parse_args()

    cmd = args.command

    if cmd not in commands.keys():
        print 'unrecognized command: {}'.format(cmd)
        print 'valid commands are: {}'.format(str(commands.keys()))
        sys.exit()

    request = commands[cmd]

    # set up zmq
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect('tcp://localhost:6789')

    # stringify request and send it on REQ socket..
    print 'sending..'
    pprint.pprint(request)
    socket.send(json.dumps(request))

    print
    print 'waiting for response..'
    print

    # and receive the response (synchronous call)
    response = json.loads(socket.recv())
    print 'got response..'
    pprint.pprint(response)
