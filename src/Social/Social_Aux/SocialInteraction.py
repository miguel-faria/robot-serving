#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import random
import time

import baxter_interface
import rospy
from robot_serving.msg import ManageExpression, FollowBehaviour, screen_mode, SpeechCues
from std_srvs.srv import Empty, EmptyResponse

from Social_Aux.screen_display_manager import PerceptionReactor
from Social_Aux.voice_manager import VoiceManager


class SpeechCodes(object):
	RAISE_CUP = 1
	COME_CLOSER = 2
	STRAIGHTEN_CUP = 3
	RAISE_CUP_MULTI = 4
	COME_CLOSER_MULTI = 5
	NEXT = 6
	NEXT2 = 7
	LAST_CUP = 8
	SMALL_TALK = 9


class SocialInteraction(object):

	def __init__(self, interaction_number):

		self.interaction_number = interaction_number
		self._limbs = {
			'right': baxter_interface.Limb('right'),
			'left': baxter_interface.Limb('left')
		}
		self._finished = False
		self._finish_signal_service = rospy.Service('finish_interaction_server', Empty, self.handle_finish_signal)
		self._facial_expression_mng = rospy.Subscriber('facial_expression_mng', ManageExpression, self._change_expression)
		self._speech_interaction_mng = rospy.Subscriber('speech_cues_mng', SpeechCues, self._speech_interact)

		self._scene_manager = PerceptionReactor()
		self._voice_manager = VoiceManager()
		self._scene_manager.start()
		self._log_file = open('interaction_log.txt', 'w')

	def _change_expression(self, msg):
		if msg.face_code == 1:
			self.happy_face()
			rospy.loginfo('Switching to happy face.')
			self._log_file.write("Received happy face code.")
		elif msg.face_code == 2:
			self.sad_face()
			rospy.loginfo('Switching to sad face.')
			self._log_file.write("Received sad face code.")
		elif msg.face_code == 3:
			self.neutral_face()
			rospy.loginfo('Switching to neutral face.')
			self._log_file.write("Received neutral face code.")
		elif msg.face_code == 4:
			self.succeed_face()
			rospy.loginfo('Switching to success face.')
			self._log_file.write("Received success face code.")
		elif msg.face_code == 5:
			self.looking_face()
			rospy.loginfo('Switching to looking around face.')
			self._log_file.write("Received looking around face code.")
		else:
			rospy.loginfo('Invalid code for face change.')
			self._log_file.write("Received invalid facial expression code: " + str(msg.face_code))

		self._log_file.write("\n")

	def _speech_interact(self, msg):
		interact_code = msg.speech_cue_code

		interactions = {
			SpeechCodes.RAISE_CUP: ["Por favor, levanta o copo.", "Levanta um pouco o copo",
									"O copo está muito baixo, pdoias levantar ligeiramente?"],
			SpeechCodes.COME_CLOSER: ["Por favor, aproxima mais o copo.",
									  "Não tenhas medo que não mordo, chega mais o copo.",
									  "Estás muito longe e não consigo chegar, põe o copo mais próximo."],
			SpeechCodes.STRAIGHTEN_CUP: ["Por favor, endireita o copo.",
										 "Podias pôr o copo direito? Assim ainda entorno",
										 "Endireita um pouco o copo, se puderes."],
			SpeechCodes.RAISE_CUP_MULTI: ["Pedia-vos que levantassem um pouco os copos.",
										  "Levantem ligeiramente os copos, por favor.",
										  "Não chego aos vossos copos, levantem-nos um pouco por favor."],
			SpeechCodes.COME_CLOSER_MULTI: ["Vocês estão tão longe! Podiam aproximar os copos?",
											"Por favor aproximem-se, que eu assim não chego aos copos.",
											"Não chego os copos, podiam aproximá-los de mim?"],
			SpeechCodes.NEXT: ["Pronto! Este já era, venha o próximo!", "Um copinho de água fresquinha bem cheiinho!",
							   "Já está! Para quem vai ser o próximo?"],
			SpeechCodes.NEXT2: ["Mais um bem cheio, adoro este trabalho!", "Outro já cheio! Para quem vai ser o próximo?",
								"E lá vai outro... com vocês isto vai bem depréssa!"],
			SpeechCodes.LAST_CUP: ["Pronto! Lá foi mais um.", "Ah! Este foi fácil, obrigado por colaborares!",
								   "Outro cheio, estou a apanhar o jeito a esta coisa."],
			SpeechCodes.SMALL_TALK: ["Então, contem lá, já tinham trabalhado com algum robô?",
									 "Enquanto estão aqui, posso perguntar-vos como está hoje o dia?" +
									 " É que não consigo ver lá para fora.",
									 "Desculpem estar a demorar, mas eu hoje estou um pouco cansado..."]
		}

		if interact_code in interactions:
			self._log_file.write("Received speech interaction code: " + str(interact_code) + "\n")
			self._voice_manager.speakString(interactions[interact_code][random.randrange(len(interactions[interact_code]))])
		else:
			rospy.loginfo('Invalid speech interaction code.')
			self._log_file.write("Received invalid speech interaction code: " + str(interact_code) + "\n")
			return

	def happy_face(self):
		self._scene_manager.screen_mode(screen_mode.HAPPY)
		self._scene_manager.follow_mode(FollowBehaviour.FOLLOW_LINK, 'right')
		self._scene_manager.clean_face()
		self._scene_manager.happy_face()
		self._scene_manager.convert_and_publish_image()

	def neutral_face(self):
		self._scene_manager.screen_mode(screen_mode.NEUTRAL)
		self._scene_manager.follow_mode(FollowBehaviour.DEFAULT, '')
		self._scene_manager.clean_face()
		self._scene_manager.object_finder()
		self._scene_manager.convert_and_publish_image()

	def sad_face(self):
		self._scene_manager.screen_mode(screen_mode.SAD)
		self._scene_manager.follow_mode(FollowBehaviour.FOLLOW_LINK, 'right')
		self._scene_manager.clean_face()
		self._scene_manager.sad_face()
		self._scene_manager.convert_and_publish_image()

	def succeed_face(self):
		self._scene_manager.screen_mode(screen_mode.VERY_HAPPY)
		self._scene_manager.follow_mode(FollowBehaviour.FOLLOW_LINK, 'right')
		self._scene_manager.clean_face()
		self._scene_manager.mega_happy_face()
		self._scene_manager.convert_and_publish_image()

	def looking_face(self):
		self._scene_manager.screen_mode(screen_mode.NERVOUS)
		self._scene_manager.follow_mode(FollowBehaviour.FOLLOW_RANDOM_LOOK, '')
		self._scene_manager.clean_face()
		self._scene_manager.delicate_face()
		self._scene_manager.convert_and_publish_image()

	def hello(self):
		self.happy_face()
		self._voice_manager.choose_cup_filling_greeting(self.interaction_number)
		#self.wave_hello()
		self._voice_manager.speakString()
		time.sleep(1)
		self.neutral_face()
		self.move_neutral()

	def goodbye(self):
		#self.wave_goodbye()
		time.sleep(1)
		self.happy_face()
		self.move_neutral()
		self._voice_manager.chooseGoodbye()
		self._voice_manager.speakString()
		time.sleep(2)
		self._scene_manager.sleep_face()
		self._scene_manager.convert_and_publish_image()

	def close_file(self):
		self._log_file.close()

	def over(self):
		return self._finished

	def handle_finish_signal(self, req):
		self._finished = True
		return EmptyResponse()

	def cycle(self):
		self._scene_manager.cycle()

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
		wave_1 = {'left_w0': -0.7401457293090821, 'left_w1': 0.42261170657958985, 'left_w2': 0.2807184838623047,
				  'left_e0': -2.165213879626465, 'left_e1': 1.8430779145385743, 'left_s0': -0.09127185677490235,
				  'left_s1': 0.2086213869140625}

		wave_2 = {'left_w0': -1.5604419546936037, 'left_w1': 1.3844176594848634, 'left_w2': 0.38426218692626957, 'left_e0': -2.5000051861999513, 'left_e1': 0.7094661135864259, 'left_s0': -0.10200972227783203, 'left_s1': 0.14879613625488283}

		#self.raise_arm()
		for _move in range(3):
			self._limbs['left'].move_to_joint_positions(wave_1)
			self._limbs['left'].move_to_joint_positions(wave_2)

	def wave_goodbye(self):
		wave_1 = {'left_w0': -0.7401457293090821, 'left_w1': 0.42261170657958985, 'left_w2': 0.2807184838623047,
				  'left_e0': -2.165213879626465, 'left_e1': 1.8430779145385743, 'left_s0': -0.09127185677490235,
				  'left_s1': 0.2086213869140625}

		wave_2 = {'left_w0': -1.5604419546936037, 'left_w1': 1.3844176594848634, 'left_w2': 0.38426218692626957,
				  'left_e0': -2.5000051861999513, 'left_e1': 0.7094661135864259, 'left_s0': -0.10200972227783203,
				  'left_s1': 0.14879613625488283}

		for _move in range(2):
			self._limbs['left'].move_to_joint_positions(wave_1)
			self._limbs['left'].move_to_joint_positions(wave_2)
