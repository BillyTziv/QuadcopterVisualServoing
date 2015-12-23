# RPI camera image capture using TX-RX signal

import RPi.GPIO as GPIO

SPIO.setmode(GPOI.BOARD)
GPIO.setup(11,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(7, GPIO.OUT)
GPIO.output(7,0)

try:
	while True:
		GPIO.output(7, GPIO.input(11) )

	except KeyboardInterrupt:
		GPIO.cleanup()
