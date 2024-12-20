# This file is run on startup from crontab (sudo crontab -e)

import picamera2
import time

camOne = picamera2.Picamera2()
config = camOne.create_still_configuration(
	main={"size": (960, 540)},
	raw={"size": (2304, 1296)},
	buffer_count=2,
	controls={"FrameRate": 100},
)

camOne.configure(config)
camOne.start()

while(True):
	# Time between every image, considered fast enough relative to the computing speed
	time.sleep(0.1)
	before = time.time()
	# Save location is configured as a volume in the docker image "launchros"
	out = camOne.capture_file("/home/CameraOne/image1.jpg")
	sendTime = round(time.time() - before, 4)
	print("Time taken per picture: ", sendTime)
