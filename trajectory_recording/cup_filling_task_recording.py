import argparse
import scipy.io as sio

import baxter_interface
import rospy

from robot_serving.msg import Cups
from trajectory_recorder import TrajectoryRecorder

_x_target = float()
_y_target = float()
_z_target = float()


def timer_callback(event):
    rospy.loginfo("Target Pos: (" + str(_x_target) + ", " + str(_y_target) + ", " + str(_z_target) + ")")


def get_pos_callback(msg):
    if msg.cups_pos_x > 0:
        global _x_target
        global _y_target
        global _z_target

        _z_target = msg.cups_pos_z[0]
        _y_target = msg.cups_pos_y[0]
        _x_target = msg.cups_pos_x[0]


def init_ros_vars():
    # Init ROS and baxter stuff
    rospy.init_node('cup_filling_task_recording')
    rospy.loginfo('cup_filling_task_recording: init: Node initialized.')
    rospy.Subscriber('/vision_processing/cups_pub', Cups, get_pos_callback)
    rospy.Timer(rospy.Duration(2), timer_callback)


def init_baxter_vars():
    limbs = {'left': baxter_interface.Limb('left'),
             'right': baxter_interface.Limb('right')}
    return limbs


def init_recording_vars():

    rec = TrajectoryRecorder('right', 100)
    return rec


def before_record(limbs):
    starting_position = {
        'left': {
            'left_s0': -0.08,
            'left_s1': -1.0,
            'left_w0':  0.67,
            'left_w1':  1.03,
            'left_w2': -0.50,
            'left_e0': -1.19,
            'left_e1':  1.94
        },
        'right': {
            'right_s0':  0.08,
            'right_s1': -1.0,
            'right_w0': -0.67,
            'right_w1':  1.03,
            'right_w2':  0.5,
            'right_e0':  1.19,
            'right_e1':  1.94
        }
    }

    for limb_name, limb in limbs.iteritems():
        limb.move_to_joint_positions(starting_position[limb_name])


def post_record(traj, target, output_filename):
    sio.savemat(output_filename, {'traj': traj, 'target': target})    
    

def task_recording():
    # Setup parser
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--output',
                        help='Name of output file',
                        type=str,
                        required=True)
    args = parser.parse_args()
    output_filename = args.output

    init_ros_vars()

    rospy.loginfo('cup_filling_task_recording: main: Make sure gripper_control is running.')
    rospy.loginfo('Press Enter when ready.')

    raw_input()
    limbs = init_baxter_vars()
    before_record(limbs)

    rospy.loginfo('cup_filling_task_recording: main: Place cup in a visible and reachable position.')
    rospy.loginfo('Press Enter to continue.')

    raw_input()
    rec = init_recording_vars()
    rospy.loginfo('cup_filling_task_recording: main: Wait for target detection to be stable')
    rospy.loginfo('Press Enter to continue.')

    raw_input()
    target = [_x_target, _y_target, _z_target]
    trajectory = rec.record()

    post_record(trajectory, target, output_filename)


if __name__ == '__main__':
    task_recording()

