import rospy
import baxter_interface
import time

from std_srvs.srv import Empty

from Social_Aux.BeaFace import BeaFace
from Social_Aux.screen_display_manager import PerceptionReactor
from Social_Aux.voice_manager import VoiceManager


class SocialInteraction(object):

	def __init__(self):

		self._limbs = {
			'right': baxter_interface.Limb('right'),
			'left': baxter_interface.Limb('left')
		}
		self._finished = False
		self._finish_signal_service = rospy.Service('finish_interaction_server', Empty , self._finish_signal_service)

		self._scene_manager = PerceptionReactor()
		self._voice_manager = VoiceManager()
		self._scene_manager.start()

	def happy_face(self):
		self._scene_manager.clean_face()
		self._scene_manager.happy_face()
		self._scene_manager.convert_and_publish_image()

	def neutral_face(self):
		self._scene_manager.clean_face()
		self._scene_manager.object_finder()
		self._scene_manager.convert_and_publish_image()

	def sad_face(self):
		self._scene_manager.clean_face()
		self._scene_manager.sad_face()
		self._scene_manager.convert_and_publish_image()

	def succeed_face(self):
		self._scene_manager.clean_face()
		self._scene_manager.mega_happy_face()
		self._scene_manager.convert_and_publish_image()

	def hello(self):
		self.happy_face()
		self._voice_manager.chooseGreeting()
		self._voice_manager.speakGreeting()
		self.raise_arm()
		self.wave_hello()
		time.sleep(1)
		self.neutral_face()

	def goodbye(self):
		self.wave_goodbye()
		self._scene_manager.sleepy_face()
		self._scene_manager.convert_and_publish_image()
		time.sleep(2)
		self._scene_manager.sleep_face()
		self._scene_manager.convert_and_publish_image()

	def over(self):
		return self._finished

	def handle_finish_signal(self, req):
		self._finished = True

	def raise_arm(self):
		arm_raised_position = {
			'right': {
				'right_s0': -0.3543495615966797,
				'right_s1': -0.052922337121582036,
				'right_w0': -1.4883448577453615,
				'right_w1': -1.4898788385314943,
				'right_w2': -0.1702718672607422,
				'right_e0': 1.699267215838623,
				'right_e1': 1.7452866394226076},
			'left': {
				'left_w0': 0.2634612000183106,
				'left_w1': 0.8943107983154297,
				'left_w2': -2.016034248175049,
				'left_e0': -0.8260486533325195,
				'left_e1': 1.8986847180358888,
				'left_s0': 0.7340098061645508,
				'left_s1': 0.021475731005859377}
		}

		self._limbs['right'].move_to_joint_positions(arm_raised_position['right'])
		self._limbs['left'].move_to_joint_positions(arm_raised_position['left'])

	def move_neutral(self):
		neutral_position = {
			'left': {
				'left_s0': 0.5721748332275391,
				'left_s1': 0.2784175126831055,
				'left_w0': 0.13038836682128907,
				'left_w1': 0.730558349395752,
				'left_w2': -1.0507768385009766,
				'left_e0': -1.047708876928711,
				'left_e1': 2.012199296209717
			},
			'right': {
				'right_s0': -0.4460049135681153,
				'right_s1': 0.03873301484985352,
				'right_w0': -0.18292720874633792,
				'right_w1': 0.6446554253723145,
				'right_w2': -1.5370487477050783,
				'right_e0': 1.5604419546936037,
				'right_e1': 2.1698158219848636
			}
		}

		self._limbs['right'].move_to_joint_positions(neutral_position['right'])
		self._limbs['left'].move_to_joint_positions(neutral_position['left'])

	def wave_hello(self):
		wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714, 'right_w0': -0.906,
				  'right_w1': -1.545, 'right_w2': -0.276}

		wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, 'right_e0': 1.831, 'right_e1': 1.981, 'right_w0': -1.979,
				  'right_w1': -1.100, 'right_w2': -0.448}

		for _move in range(2):
			self._limbs['right'].move_to_joint_positions(wave_1)
			self._limbs['right'].move_to_joint_positions(wave_2)

	def wave_goodbye(self):
		wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714, 'right_w0': -0.906,
				  'right_w1': -1.545, 'right_w2': -0.276}

		wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, 'right_e0': 1.831, 'right_e1': 1.981, 'right_w0': -1.979,
				  'right_w1': -1.100, 'right_w2': -0.448}

		for _move in range(1):
			self._limbs['right'].move_to_joint_positions(wave_1)
			self._limbs['right'].move_to_joint_positions(wave_2)
