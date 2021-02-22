from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.rotation = 20

camera.start_preview()
sleep(10)
camera.stop_preview()
