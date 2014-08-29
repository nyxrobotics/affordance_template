import rospy
import geometry_msgs.msg

class FrameStore(object) :

	def __init__(self, frame_id, root_frame_id, pose) :
		self.frame_id = frame_id
		self.root_frame_id = root_frame_id
		self.pose = pose