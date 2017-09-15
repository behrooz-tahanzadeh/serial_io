import rospy
from app.core import util
from std_msgs.msg._String import String
from thread import start_new_thread
from time import sleep




class SerialHandler:
	
	def __init__(self, deviceIdCmd, deviceID, writeToSerialTN, readFromSerialTN):
		self.deviceID = deviceID
		self.deviceIdCmd = deviceIdCmd
		self.serial = None
		
		rospy.Subscriber(writeToSerialTN, String, self.writeToSerialCb);
		self.readlineFormSerialPublisher = rospy.Publisher(readFromSerialTN, String, queue_size=1)
		
		start_new_thread(self.readlineThread, ())
	#eof
	
	
	def checkSerial(self, serial):
		if(serial.isOpen()):
			
			serial.write(self.deviceIdCmd)
			
			sleep(1)
			
			if(serial.inWaiting()>0):
				
				l = str(serial.readline()).strip()
				util.logInfo("Read Value:"+l)
				
				return l == self.deviceID
		return False
	#eof
	
	
	
	def setSerial(self, serial):
		if(self.serial is None or not self.serial.isOpen()):
			self.serial = serial
			return True
		else:
			return False
	#eof
	
	
	
	def writeToSerialCb(self, msg):
		if(self.serial is not None and self.serial.isOpen()):
			try:
				self.serial.write(msg.data)
			except:
				util.logErr("Error in writing to serial port")
		else:
			util.logErr("Serial port is not ready")
	#eof
	
	
	
	def readlineThread(self):
		while(True):
			try:
				if(self.serial is not None):
					if(not self.serial.isOpen()):
						util.logErr("Port is close. Set the serial port to None!")
						self.serial = None
					elif(self.serial.inWaiting()>0):
						data = self.serial.readline()
						self.readlineFormSerialPublisher.publish(data)
				else:
					sleep(2)
			except IOError:
				util.logErr("Port is close. Set the serial port to None!")
				self.serial = None
			except:
				util.logErr("Error is readline function")
				sleep(2)
	#eof