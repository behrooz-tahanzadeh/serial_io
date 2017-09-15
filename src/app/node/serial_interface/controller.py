import rospy
from app.core import util
from std_msgs.msg._String import String
from thread import start_new_thread
from serial import Serial
from time import sleep
from std_srvs.srv._Trigger import Trigger, TriggerResponse



class Controller:
	
	
	def __init__(self, port, baudrate, initTime = 2):
		self.port = port
		self.baudrate = baudrate
		self.initTime = initTime
		self.serial = None
		self.isSerialReady = False
		self.isReseting = False
		
		self.readlineFormSerialPublisher = rospy.Publisher(util.topicName("serial_interface", "readline_from_serial"), String, queue_size=10)
		rospy.Subscriber(util.topicName("serial_interface", "write_to_serial"), String, self.writeToSerialCb)
		
		rospy.Service(util.topicName("serial_interface", "reset"), Trigger, self.resetServiceCb)
		
		start_new_thread(self.readlineThread, ())
		
		self.restartArduino()
	#eof
	
	def resetServiceCb(self, req):
		if(self.isReseting):
			return TriggerResponse(False, "Reseting is already running!")
		else:
			start_new_thread(self.restartArduino, ())
			return TriggerResponse(True, "reseting")
	
	
	def restartArduino(self):
		if(self.isReseting):
			return
		
		self.isReseting = True
		rospy.loginfo("Start serial initialization...")
		if(self.isSerialReady):
			self.serial.close()
			self.isSerialReady = False
		
		self.isSerialReady = False
		
		try:
			self.serial = Serial(self.port, self.baudrate)
		except:
			rospy.logfatal("Can not open port! Port:"+str(self.port))
			rospy.logfatal("Check connections and serial ports...")
			raise ValueError("Invalid port")
		
		rospy.logwarn("Wait for initialization. Time (sec): "+str(self.initTime))
		sleep(self.initTime)
		self.isSerialReady = True
		rospy.logwarn("Serial port is ready. Port: "+str(self.port))
		self.isReseting = False
	#eof
	
	


	def writeToSerialCb(self, msg):
		if(self.isSerialReady):
			self.serial.write(msg.data)
		else:
			rospy.logwarn("Serial is not ready! Can not write data to serial port.")
	#eof
	
	


	def readlineThread(self):
		while(True):
			try:
				if(self.isSerialReady and not self.isReseting and self.serial.inWaiting()>0):
					data = self.serial.readline()
					self.readlineFormSerialPublisher.publish(data)
			except:
				rospy.logwarn("Error is readlineThread function!")
	#eof
