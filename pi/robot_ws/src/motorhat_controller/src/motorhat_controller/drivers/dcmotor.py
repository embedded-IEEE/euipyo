from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor
from time import sleep

mh = Raspi_MotorHAT(addr=0x6f) 
myMotor = mh.getMotor(2) #ÇÉ¹øÈ£

myMotor.setSpeed(100) #¼Óµµ

try:
	myMotor.run(Raspi_MotorHAT.FORWARD) #¹è¼±¿¡ µû¶ó ÀüÁø or ÈÄÁø
	sleep(5)
	myMotor.run(Raspi_MotorHAT.RELEASE) #Á¤Áö
finally:
	myMotor.run(Raspi_MotorHAT.RELEASE)
