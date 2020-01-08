import socket
from breezycreate2 import Robot
import cv2
import numpy as np 
import time

def main():

	#establish robot connection
	bot = Robot()
	bot.playNote('A4', 50)

	# establish control connection
	host = "10.7.88.88"
	port = 9000
	s = socket.socket()
	s.bind((host,port))
	s.listen(1)
	client_socket, client_address = s.accept()
	print("Connection established from: " + str(client_address))

	# the received message
	last_message = 'i'
	while 1:
		data = client_socket.recv(2).decode('utf-8')
		datac = [char for char in data]
		print(data)

		# Stop the robot when no target is detected
		if datac[0] == 's':
			bot.setForwardSpeed(0)
		if datac[1] == 's':
			bot.setTurnSpeed(0)

		# set the iRobot to go forward
		if datac[0] == 'f':
			bot.setForwardSpeed(100)
			time.sleep(1)
			bot.setForwardSpeed(0)

		# set the iRobot to turn left
		elif datac[1] == 'l':
			bot.setTurnSpeed(25)
			time.sleep(1)
			bot.setTurnSpeed(0)

		# set the iRobot to turn right
		elif datac[1] == 'r':
			bot.setTurnSpeed(-25)
			time.sleep(1)
			bot.setTurnSpeed(0)

		last_message = datac

	client_socket.close()
	bot.close()
	pipeline.stop()

if __name__=="__main__":
    main()

