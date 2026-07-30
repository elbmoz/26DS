"""Compare MaixCAM 16:9 and 4:3 output fields of view.

Run this file by itself in MaixVision.  Both outputs come from the same
GC4653 sensor session, so a missing top/bottom or left/right edge reveals
cropping.  The preview alternates every two seconds.
"""

from maix import app, camera, display, image, time


WIDE_SIZE = (1280, 720)
VGA_SIZE = (640, 480)
FPS = 60
SWITCH_MS = 2000


wide_cam = camera.Camera(
    width=WIDE_SIZE[0],
    height=WIDE_SIZE[1],
    fps=FPS,
    buff_num=2,
)
vga_cam = wide_cam.add_channel(
    width=VGA_SIZE[0],
    height=VGA_SIZE[1],
    format=image.Format.FMT_RGB888,
    fps=FPS,
    buff_num=2,
)
screen = display.Display()

print("device:", camera.get_device_name())
print("wide output:", wide_cam.width(), wide_cam.height(), wide_cam.fps())
print("vga output:", vga_cam.width(), vga_cam.height(), vga_cam.fps())
print("preview alternates every {} ms".format(SWITCH_MS))

show_wide = True
last_switch_ms = time.ticks_ms()

while not app.need_exit():
    now_ms = time.ticks_ms()
    if now_ms - last_switch_ms >= SWITCH_MS:
        show_wide = not show_wide
        last_switch_ms = now_ms

    if show_wide:
        frame = wide_cam.read()
        label = "WIDE 1280x720 16:9"
        label_color = image.COLOR_GREEN
    else:
        frame = vga_cam.read()
        label = "VGA 640x480 4:3"
        label_color = image.COLOR_RED

    frame.draw_string(12, 12, label, label_color, 1.2)
    screen.show(frame)
