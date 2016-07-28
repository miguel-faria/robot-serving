import rospy
from std_msgs.msg import Int32

from Social_Aux.SocialInteraction import SocialInteraction


def main():

	rospy.init_node("social_interaction_robot_serving")
	pub = rospy.Publisher('start', Int32, queue_size=100)
	interaction = SocialInteraction()
	interaction.hello()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown() and not interaction.over():
		pub.publish(1)
		rate.sleep()

	interaction.goodbye()

if __name__ == '__main__':
	main()