from tf.transformations import translation_matrix, quaternion_matrix,\
	translation_from_matrix, quaternion_from_matrix
import numpy
from geometry_msgs.msg._Pose import Pose
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Quaternion import Quaternion
import rospy




def transformationMatrix(translation, orientation):
	translation = translation_matrix(translation)
	orientation = quaternion_matrix(orientation)
	
	return numpy.dot(translation, orientation)
#eof


def poseToMatrix(pose):
	q = pose.orientation
	t = pose.position
	
	return transformationMatrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
#eof


def markerTransformationMatrix(marker):
	q = marker.pose.orientation
	t = marker.pose.position
	
	return transformationMatrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
#eof



def transformPoseByMatrix(pose, matrix):
	return numpy.dot(matrix, pose)
#eof



def matrixToPose(matrix):
	t = tuple(translation_from_matrix(matrix))
	q = tuple(quaternion_from_matrix(matrix))
	
	return Pose(Point(t[0], t[1], t[2]), Quaternion(q[0], q[1], q[2], q[3]))
#eof



def topicName(pkgName, topicName):
	"""
	generate name of the topic based on the convention
	:param pkgName: name of the package without any slash
	:param topicName: name of the package without any slash at the begining or end
	"""
	return "serial_io/"+pkgName+"/"+topicName
#eof



def logBeginningInfo(pkgName):
	print "======= serial_io / "+pkgName+" ======="
#eof




def getStampedTime(obj):
	return obj.header.stamp.secs + (obj.header.stamp.nsecs/1000000000.0)
#eof



def logInfo(msg):
	o = '\033[1m'+rospy.get_name()+': \033[2m'+str(msg)
	rospy.loginfo(o)
#eof




def logWarn(msg):
	#o = '\033[1m'+rospy.get_name()+': \033[2m'+str(msg)
	o = str(msg)
	rospy.logwarn(o)
#eof




def logErr(msg):
	o = '\033[1m'+rospy.get_name()+': \033[2m'+str(msg)
	rospy.logerr(o)