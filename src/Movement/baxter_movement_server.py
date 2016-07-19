import argparse
import sys

import rospy

from Baxter_Movement_Aux.hybrid_traj_service import HybridTrajectoryServer
from Baxter_Movement_Aux.legible_traj_service import LegibleTrajectoryServer
from Baxter_Movement_Aux.predictable_traj_service import PredictTrajectoryServer


def main():
	# Parse arguments
	parser = argparse.ArgumentParser()
	parser.add_argument('-t',
						type=str,
						required=True,
						dest='traj_type',
						help='Type of trajectory to perform based on the three approaches, '
							 'valid values: predict, legible, hybrid')
	args = parser.parse_args()

	rospy.init_node('baxter_movement_server')

	if args.traj_type == 'predict':
		traj_server = PredictTrajectoryServer()
	elif args.traj_type == 'legible':
		traj_server = LegibleTrajectoryServer()
	elif args.traj_type == 'hybrid':
		traj_server = HybridTrajectoryServer()
	else:
		rospy.logerr('Invalid movement trajectory type! Exiting...')
		sys.exit(1)
	rospy.loginfo('Baxter Movement Server Initiated')

	rospy.spin()

if __name__ == '__main__':
	main()