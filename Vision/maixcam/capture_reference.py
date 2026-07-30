"""Capture one raw calibration frame on MaixCAM.

Run this file by itself in MaixVision.  The camera is warmed up for a few
frames, then the untouched frame is saved on the device and kept on screen.
"""

from maix import app, camera, display, time


REFERENCE_PATH = "/root/ball_pipe_reference.jpg"
WARMUP_FRAMES = 30


cam = camera.Camera(640, 480)
disp = display.Display()

frame = None
for _ in range(WARMUP_FRAMES):
    frame = cam.read()

frame.save(REFERENCE_PATH)
print("reference image saved:", REFERENCE_PATH)

while not app.need_exit():
    disp.show(frame)
    time.sleep_ms(100)
