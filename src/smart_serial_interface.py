#!/usr/bin/env python
import rospy
from app.core import util
from app.node.smart_serial_interface.SerialHandler import SerialHandler
from app.node.smart_serial_interface.SerialSeeker import SerialSeeker




def main(argv=None):
	rospy.init_node('smart_serial_interface', anonymous=True)
	util.logBeginningInfo("smart_serial_interface")
	
	ss = SerialSeeker(
	[
		SerialHandler("i", "a", "/kuka_a/effector_io/serial_interface/write_to_serial", "/kuka_a/effector_io/serial_interface/readline_to_serial"),
		SerialHandler("ii", "g", "/gripper/effector_io/serial_interface/write_to_serial", "/gripper/effector_io/serial_interface/readline_to_serial")
	])
	
	rate = rospy.Rate(0.5)
	
	while not rospy.is_shutdown():
		ss.setSerialPorts()
		rate.sleep()
#eof



if __name__ == '__main__':
	main()