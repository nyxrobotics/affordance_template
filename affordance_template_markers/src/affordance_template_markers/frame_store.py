import rospy
import geometry_msgs.msg

class FrameStore(object) :

	def __init__(self) :
		self.frame_id = ""
		self.root_frame_id = ""
		self.pose = geometry_msgs.msg.Pose()