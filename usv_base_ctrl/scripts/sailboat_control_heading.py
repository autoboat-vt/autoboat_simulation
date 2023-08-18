#!/usr/bin/env python

import rospy
import zmq
import msgpack
import math
import time
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates
from wind_current.srv import (
    GetSpeed as WindGetSpeed,
    GetSpeedResponse as WindGetSpeedResponse
)
from water_current.srv import (
    GetSpeed as WaterGetSpeed,
    GetSpeedResponse as WaterGetSpeedResponse
)

data = {
    'pose': None,
    'state': None,
    'map_bounds': None,
    'wind': None,
    'water': None,
}
subscribtions = {}

def show_stats(func):
    stats = {'count': 0, 'first_call_time': None, 'duration': 0}
    def wrapper(*args, **kwargs):
        if stats['first_call_time'] is None:
            stats['first_call_time'] = time.time()
        stats['count'] += 1
        t0 = time.time()
        res = func(*args, **kwargs)
        t1 = time.time()
        stats['duration'] += t1 - t0

        freq = stats['count'] / (time.time() - stats['first_call_time'])
        mean_duration = stats['duration'] / stats['count']
        print('[GYM_SERVER] [%s] freq: %f/s, duration: ~%fms' % (func.__name__, freq, mean_duration / 1000))
    return wrapper

# @show_stats
def pose_cb(msg):
    data['pose'] = msg

# @show_stats
def state_cb(msg):
    data['state'] = msg

def init_states_cb(msg):
    names = msg.name
    min_x = min_y = min_z = float('inf')
    max_x = max_y = max_z = float('-inf')
    for i, name in enumerate(names):
        # the map bounds are defined by the buoys 
        if 'buoy' in name:
            pos = msg.pose[i].position
            min_x = min(min_x, pos.x)
            min_y = min(min_y, pos.y)
            min_z = min(min_z, pos.z)
            max_x = max(max_x, pos.x)
            max_y = max(max_y, pos.y)
            max_z = max(max_z, pos.z)
    data['map_bounds'] = {
        'min_position': {'x': min_x, 'y': min_y, 'z': min_z},
        'max_position': {'x': max_x, 'y': max_y, 'z': max_z},
    }
    subscribtions['init_states'].unregister()

def get_sensors():
    position = data['pose'].pose.pose.position
    orientation = data['pose'].pose.pose.orientation
    linear_vel = data['pose'].twist.twist.linear
    angular_vel = data['pose'].twist.twist.angular

    heading_euler = tf.transformations.euler_from_quaternion(
        (orientation.x, orientation.y, orientation.z, orientation.w)
    )

    return {
        'p_boat': {'x': position.x, 'y': position.y, 'z': position.z},
        'dt_p_boat': {'x': linear_vel.x, 'y': linear_vel.y, 'z': linear_vel.z},

        'theta_boat': {'x': heading_euler[0], 'y': heading_euler[1], 'z': heading_euler[2]},
        'dt_theta_boat': {'x': angular_vel.x, 'y': angular_vel.y, 'z': angular_vel.z},

        'theta_rudder': data['state'].position[0],
        'dt_theta_rudder': data['state'].velocity[0],

        'theta_sail': data['state'].position[1],
        'dt_theta_sail': data['state'].velocity[1],

        'wind': {'x': data['wind']['x'], 'y': data['wind']['y']},
        'water': {'x': data['water']['x'], 'y': data['water']['y']},
    }

def get_info():
    return {}

def get_init_info():
    while data['map_bounds'] is None:
        print('[GYM_SERVER] waiting for map bounds...')
        time.sleep(0.1)
    return {
        'min_position': data['map_bounds']['min_position'],
        'max_position': data['map_bounds']['max_position'],
    }

def convert_act_to_instr(act):
    theta_rudder = act['theta_rudder']
    theta_sail = act['theta_sail']
    assert -math.pi <= theta_rudder <= math.pi, 'theta_rudder must be in [-pi, pi]'
    assert -math.pi <= theta_sail <= math.pi, 'theta_sail must be in [-pi, pi]'
    instr = JointState()
    instr.header = Header()
    instr.name = ['rudder_joint', 'sail_joint']
    instr.position = [theta_rudder, theta_sail]
    instr.velocity = []
    instr.effort = []
    return instr

def create_zmq_socket():
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind('tcp://*:5555')
    print('[GYM_SERVER] zmq socket listening on port 5555')
    return socket

def handle_wind_current(req):
    # for now we use a global wind
    assert data['wind'] is not None, 'wind is not initialized'
    return WindGetSpeedResponse(data['wind']['x'], data['wind']['y'], 0)

def handle_water_current(req):
    # for now we use a global wind
    assert data['water'] is not None, 'water is not initialized'
    return WaterGetSpeedResponse(data['water']['x'], data['water']['y'])

def is_ready():
    return data['pose'] is not None \
        and data['state'] is not None \
        and data['map_bounds'] is not None

def pause_simulation():
    rospy.ServiceProxy('/gazebo/pause_physics', Empty)()

def resume_simulation():
    rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()

def reset_world():
    rospy.ServiceProxy('/gazebo/pause_physics', Empty)()
    rospy.ServiceProxy('/gazebo/reset_world', Empty)()
    rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()

def reset_simulation():
    # reset simulation
    reset_world()

    data['pose'] = None
    data['state'] = None
    while not is_ready():
        print('[GYM_SERVER] preparing simulation...')
        time.sleep(0.001)

def run_worker():
    rospy.init_node('usv_simple_ctrl', anonymous=True)
    socket = create_zmq_socket()

    # zmq helpers
    def send_msg(msg):
        socket.send(msgpack.packb(msg, use_bin_type=False))
    def recv_msg():
        return msgpack.unpackb(socket.recv(), raw=True)

    # publishers
    pub_rudder = rospy.Publisher('joint_setpoint', JointState, queue_size=10)

    # subscribers
    subscribtions['pose'] = rospy.Subscriber('state', Odometry, pose_cb)
    subscribtions['state'] = rospy.Subscriber('joint_states', JointState, state_cb)
    subscribtions['init_states'] = rospy.Subscriber('/gazebo/model_states', ModelStates, init_states_cb)

    # services
    rospy.Service('/windCurrent', WindGetSpeed, handle_wind_current)
    rospy.Service('/waterCurrent', WaterGetSpeed, handle_water_current)

    # prepare services
    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.wait_for_service('/gazebo/pause_physics')
    rospy.wait_for_service('/gazebo/reset_world')

    # rate
    rate = None

    while not rospy.is_shutdown():
        try:
            msg = recv_msg()
            try:
                if 'reset' in msg:
                    data['wind'] = msg['reset']['wind']
                    data['water'] = msg['reset']['water']
                    rate = rospy.Rate(msg['reset']['freq'])
                    reset_simulation()
                    pause_simulation()
                    send_msg({'obs': get_sensors(), 'done': False, 'info': get_init_info()})
                elif 'action' in msg:
                    data['wind'] = msg['action']['wind']
                    data['water'] = msg['action']['water']
                    instr = convert_act_to_instr(msg['action'])
                    resume_simulation()
                    pub_rudder.publish(instr)
                    rate.sleep()
                    send_msg({'obs': get_sensors(), 'done': False, 'info': get_info()})
                    pause_simulation()
                elif 'close' in msg:
                    pause_simulation()
                    send_msg({'obs': None, 'done': True, 'info': get_info()})
                else:
                    raise AssertionError('Unknown message: %s' % str(msg))
            except AssertionError as e:
                print('[GYM_SERVER] Error: %s' % str(e))
                send_msg({'error': str(e)})
        except rospy.ROSInterruptException:
            rospy.logerr('ROS Interrupt Exception! Just ignore the exception!')
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr('ROS Time Backwards! Just ignore the exception!')

    # send final message
    recv_msg()
    send_msg({'obs': None, 'done': True, 'info': None})
    pause_simulation()

if __name__ == '__main__':
    run_worker()