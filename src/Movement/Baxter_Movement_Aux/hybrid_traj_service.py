import numpy as np
import rospy
import scipy.io as sio
from promp.copromp import CoProMP
from promp.utils.utils import linear_phase
from promp.utils.utils import normalized_gaussian_basis

# import do teu state extractor
from robot_serving.srv import Movement
from Baxter_Movement_Aux.baxter_connection import BaxterConnection
from Baxter_Movement_Aux.trajectory_executor import TrajectoryExecutor


class HybridTrajectoryServer(object):

	def __init__(self):

		self._predict_service_server = rospy.Service("movement_decision_predictable", Movement,
													 self.predict_traj_srv_handler)
		self._legible_service_server = rospy.Service("movement_decision_legible", Movement,
													 self.legible_traj_srv_handler)
		self._predict_Y = []
		self._legible_Y = []
		self._predict_O = []
		self._legible_O = []
		self.load_trajs()

		self._predict_copmp = CoProMP(self._predict_O, self._predict_Y, 3, 7, o_dt=1, dt=0.001, Sigma_y=None)
		self._legible_copmp = CoProMP(self._legible_O, self._legible_Y, 3, 7, o_dt=1, dt=0.001, Sigma_y=None)
		self._predict_copmp.build(linear_phase, lambda z, dt: normalized_gaussian_basis(2, z, dt),
								  linear_phase, lambda z, dt: normalized_gaussian_basis(10, z, dt))
		self._legible_copmp.build(linear_phase, lambda z, dt: normalized_gaussian_basis(2, z, dt),
								  linear_phase, lambda z, dt: normalized_gaussian_basis(10, z, dt))

		self._baxter_connect = BaxterConnection()

	def load_trajs(self):
		N = 20
		predict_trajs = []
		predict_targets = []
		legible_trajs = []
		legible_targets = []
		for i in range(1, N + 1):
			print('clean_trajectories/predict_traj%d.mat' % i)
			predict_trajs.append(
					sio.loadmat('./trajectory_recording/clean_trajectories/predict_traj%d.mat' % i)['traj'][:, 1:7+1])
			predict_targets.append(
					sio.loadmat('./trajectory_recording/clean_trajectories/predict_traj%d.mat' % i)['target'])
			print('clean_trajectories/legible_traj%d.mat' % i)
			legible_trajs.append(
					sio.loadmat('./trajectory_recording/clean_trajectories/legible_traj%d.mat' % i)['traj'][:, 1:7 + 1])
			legible_targets.append(
					sio.loadmat('./trajectory_recording/clean_trajectories/legible_traj%d.mat' % i)['target'])

		self._predict_Y = np.hstack(predict_trajs)
		self._legible_Y = np.hstack(legible_trajs)
		self._predict_O = np.hstack(predict_targets)
		self._legible_O = np.hstack(legible_targets)

	def legible_traj_srv_handler(self, req):

		target = [req.XPos, req.YPos, req.ZPos]
		new_o = np.vstack(target)

		self._legible_copmp.condition(new_o, 1)
		ymp = self._legible_copmp.most_probable()

		time = ymp[:, 0][:, np.newaxis]
		right_traj = ymp[:, 1:7 + 1]

		te = TrajectoryExecutor(time, 10, None, right_traj)
		te.execute()

	def predict_traj_srv_handler(self, req):
		target = [req.XPos, req.YPos, req.ZPos]
		new_o = np.vstack(target)

		self._predict_copmp.condition(new_o, 1)
		ymp = self._predict_copmp.most_probable()

		time = ymp[:, 0][:, np.newaxis]
		right_traj = ymp[:, 1:7 + 1]

		te = TrajectoryExecutor(time, 10, None, right_traj)
		te.execute()
