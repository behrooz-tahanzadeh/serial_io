#!/usr/bin/env python
import rospy
from app.core import util
from app.node.serial_interface.controller import Controller



def main(argv=None):
	rospy.init_node('serial_interface', anonymous=True)
	util.logBeginningInfo("serial_interface")
	
	try:
		port,baudrate = getArgs()
	except:
		rospy.logfatal("Invalid port value!")
		return
	
	try:
		Controller(port, baudrate)
		rospy.spin()
	except:
		return
#eof



def getArgs():
	port = rospy.get_param('~port', None)
	baudrate = rospy.get_param('~baudrate', None)
	
	if(port is None):
		raise ValueError("No port has been defined!")

	if(baudrate is None):
		rospy.logwarn("No baudrate has been set. Default Value: 9600")
		baudrate = "9600"

	baudrate = int(baudrate)

	return port, baudrate
#eof



if __name__ == '__main__':
	main()