import glob
import serial
import rospy
from time import sleep
from app.core import util
from serial.serialutil import SerialException



class SerialSeeker:
	
	
	def __init__(self, serialHandlers):
		self.serialHandlers = serialHandlers
	
	
	
	def setSerialPorts(self):
		notOpen = 0
		
		for h in self.serialHandlers:
			if(h.serial is None):
				notOpen += 1
		
		if(notOpen == 0):
			return
		
		ports = glob.glob('/dev/tty[A-Za-z]*')
		
		for port in ports:
			try:
				util.logWarn("Trying port:"+port)
				s = serial.Serial(port)
				
				util.logWarn("sleep for 2 secs"+port)
				sleep(2)
				
				serialAssigned = False
				util.logWarn("Port is open! Start trying Handlers...")
				
				for h in self.serialHandlers:
					if(h.serial is None and h.checkSerial(s)):
						h.setSerial(s)
						util.logWarn("Port has been successfully assigned! Port:"+port)
						serialAssigned = True
				
				if not serialAssigned:
					s.close()
					
			except SerialException:
				pass