import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.IN)

GPIO.setup(7,GPIO.OUT)
GPIO.output(7,0)

try:
	while True:
		if (GPIO.input(11) == 1):
			print "ON";
			time.sleep(50.0/1000.0);
		else:
			print "OFF";
			time.sleep(50.0/1000.0);

except KeyboardInterrupt:
		GPIO.cleanup()
