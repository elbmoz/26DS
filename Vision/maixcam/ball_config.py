"""Real-device parameters for the pipe-and-steel-ball vision loop."""

# Project entry mode:
#   "track"  - normal low-latency competition output
#   "stream" - competition tracking + RTSP + Windows telemetry/control
#   "record" - synchronized bench-test video and per-frame CSV
#   "serve"  - temporary read-only HTTP download of saved test runs
APP_MODE = "stream"

# Camera.  The rigid car installation no longer needs a 640 x 480 tracking
# channel.  480 x 360 keeps the same 4:3 field of view and leaves the steel
# ball about 9--13 px in radius while cutting detector pixels by 43.75%.
REFERENCE_WIDTH = 640
REFERENCE_HEIGHT = 480
CAMERA_WIDTH = 480
CAMERA_HEIGHT = 360
# The current GC4653 firmware exposes practical 30/60 FPS sensor modes.
# Requesting 50 falls back to the 30 FPS mode, so use the 60 FPS mode and keep
# the independent UART deadline at 50 Hz.
CAMERA_FPS = 60
# The current camera + RTSP encoder pipeline requires one capture buffer.
# Two buffers make VENC initialization fail on this MaixCAM image.
CAMERA_BUFFER_COUNT = 1
CAMERA_WARMUP_FRAMES = 30


def _sx(value):
    return int(round(float(value) * CAMERA_WIDTH / REFERENCE_WIDTH))


def _sy(value):
    return int(round(float(value) * CAMERA_HEIGHT / REFERENCE_HEIGHT))


def _area(value):
    return int(
        round(
            float(value)
            * CAMERA_WIDTH
            * CAMERA_HEIGHT
            / (REFERENCE_WIDTH * REFERENCE_HEIGHT)
        )
    )


def _point(x, y):
    return (_sx(x), _sy(y))


def _roi(x, y, width, height):
    return (_sx(x), _sy(y), _sx(width), _sy(height))


# Preserve the old 640-wide UART error scale so lowering vision resolution
# does not silently change the STM32 controller gain.
CONTROL_OUTPUT_SCALE = float(REFERENCE_WIDTH) / CAMERA_WIDTH

# Keep None during color calibration.  A short manual exposure is enabled
# after adequate diffuse lighting is installed and tested.
CAMERA_EXPOSURE_US = None
CAMERA_GAIN = None

# Competition-car calibration is stored in the original 640 x 480 reference
# coordinate system and scaled once here for the 480 x 360 detector channel.
# The camera is rigidly mounted to the chassis.  The pipe rotates around an
# almost fixed image-space centre, so the colour blob estimates angle while
# the mechanical centre and travel length keep the control scale stable.
# 2026-07-30 live re-calibration after the endpoint screws were replaced by
# black tape.  These fallback endpoints are the two tape centres in the
# 640x480 reference image; the live right endpoint is updated below.
ROI = _roi(30, 115, 495, 48)
AXIS_START = _point(35, 132)
AXIS_END = _point(519, 144)
TARGET_POSITION = 0.50

# The camera, pivot and pipe travel are mechanically fixed.  Detect only a
# short central section of the green pipe and use its long edge for angle;
# the calibrated centre and full travel length remain constant.
PIPE_POSE_ENABLED = True
REQUIRE_VALID_PIPE_POSE = True
PIPE_POSE_MODE = "right_tape"

# The former endpoint screws have been removed.  The left black tape marker
# is camera-fixed and the right marker follows pipe rotation.  Black pixels
# on the right merge into the dark motor/cable background, so detect the
# green-to-black boundary immediately inside the right tape instead.  Its
# right edge is the stable tape endpoint.
PIPE_TAPE_LEFT_ENDPOINT = AXIS_START
PIPE_TAPE_RIGHT_ENDPOINT = AXIS_END
PIPE_TAPE_RIGHT_SEARCH_ROI = _roi(425, 95, 105, 105)
PIPE_TAPE_LAB_THRESHOLDS = (
    (5, 90, -55, -12, -15, 40),
)
PIPE_TAPE_DETECT_INTERVAL_FRAMES = 3
PIPE_TAPE_MIN_WIDTH_PX = _sx(30)
PIPE_TAPE_MAX_WIDTH_PX = _sx(150)
PIPE_TAPE_MIN_HEIGHT_PX = _sy(8)
PIPE_TAPE_MAX_HEIGHT_PX = _sy(48)
PIPE_TAPE_MIN_PIXELS = _area(60)
PIPE_TAPE_X_STRIDE = 4
PIPE_TAPE_Y_STRIDE = 3
PIPE_TAPE_EXPECTED_RIGHT_X = _sx(519)
PIPE_TAPE_MAX_RIGHT_X_DISTANCE_PX = _sx(16)
PIPE_TAPE_MIN_AXIS_LENGTH_PX = _sx(430)
PIPE_TAPE_MAX_AXIS_LENGTH_PX = _sx(535)
PIPE_TAPE_ENDPOINT_FROM_BLOB_RIGHT_EDGE = True
# The endpoint crop itself ends at the tape centre, so its clipped green
# component right edge already represents the calibrated control endpoint.
PIPE_TAPE_ENDPOINT_X_OFFSET_PX = 0

PIPE_FIXED_SEARCH_ROI = True
PIPE_SEARCH_ROI = _roi(225, 120, 135, 42)
PIPE_LAB_THRESHOLDS = (
    (5, 90, -55, -12, -15, 40),
)
PIPE_DETECT_INTERVAL_FRAMES = 6
PIPE_MIN_LENGTH_PX = _sx(55)
PIPE_MAX_LENGTH_PX = _sx(155)
PIPE_MIN_WIDTH_PX = _sy(6)
PIPE_MAX_WIDTH_PX = _sy(35)
PIPE_MIN_ASPECT = 2.5
PIPE_MIN_PIXELS = _area(100)
PIPE_MERGE_BLOBS = False
PIPE_MERGE_MARGIN = 0
# The narrow fixed crop excludes the green tabletop and almost all PCB area.
# Prefer the longest narrow component; no wide-area centre search is needed.
PIPE_EXPECTED_CENTER = _point(292, 139)
PIPE_MAX_CENTER_DISTANCE_PX = 0
PIPE_MAX_ABS_ANGLE_DEG = 8
# Use the detected component only for angle.  Its visible ends change with
# glare, the ball and hands, but the physical pipe centre/length do not.
PIPE_FIXED_AXIS_CENTER = _point(289, 139)
PIPE_FIXED_AXIS_LENGTH_PX = _sx(450)
# The detector-scale pipe is about 335 x 23 px.  A 4 x 3 stride still leaves
# roughly 84 x 8 samples while cutting the wide green pass nearly in half.
PIPE_X_STRIDE = 5
PIPE_Y_STRIDE = 3
# Kept for the generic detector path; fixed-search mode never builds this
# adaptive ROI and never enters a broad fallback.
PIPE_POSE_SEARCH_ALONG_MARGIN_PX = _sx(12)
PIPE_POSE_SEARCH_LATERAL_MARGIN_PX = _sy(30)
PIPE_SMOOTHING_ALPHA = 0.55
# The reference green minimum-area rectangle is about 423 px long while the
# calibrated control axis is about 415 px, so a 4 px inset at each end keeps
# the existing control scale without hard-coding the pipe's screen position.
PIPE_AXIS_INSET_PX = 0
# This is the blue control/search box shown by the Windows preview.  Keep it
# close to the physical moving pipe: the fixed axis already represents the
# usable pipe length, and the ball needs only a modest radius allowance above
# and below it.  Pose acquisition has its own larger search margins above.
PIPE_ROI_ALONG_MARGIN_PX = 0
PIPE_ROI_LATERAL_MARGIN_PX = _sy(16)
PIPE_MAX_STALE_FRAMES = 9
# Ignored in fixed-search mode; retained for the reusable generic class.
PIPE_BROAD_RETRY_INTERVAL_UPDATES = 2

# The car recording shows that L<=60 keeps only disconnected dark islands on
# the mirrored ball, while the smaller end screw remains a clean blob.  Keep
# the neutral-colour gate, but include the ball's mid-bright reflections so
# its full 25--30 px silhouette wins the geometry test.
BALL_LAB_THRESHOLDS = (
    (0, 85, -22, 22, -20, 20),
)

# Native LAB blob search and geometry rejection.
LOCAL_SEARCH_WIDTH_PX = _sx(80)
# Once locked, follow both predicted x and y.  This prevents a tilted pipe's
# tall axis-aligned bounding box from making every local search nearly full.
LOCAL_SEARCH_HEIGHT_PX = _sy(48)
# A confirmed track can safely coast through one isolated LAB miss.  Broad
# search on the second consecutive miss avoids full-pipe work on reflections
# that disappear again on the next frame.
LOCAL_FALLBACK_INTERVAL_MISSES = 2
BLOB_MIN_WIDTH = _sx(12)
BLOB_MAX_WIDTH = _sx(50)
BLOB_MIN_HEIGHT = _sy(12)
BLOB_MAX_HEIGHT = _sy(40)
# Empty-pipe vehicle captures produce a persistent 7--8 px-wide blob on the
# printed left-end scale.  The real ball was never below 11 px radius in the
# labelled 640-wide runs (about 8 px after scaling), so a 9 x 9 detector-pixel
# minimum removes that nuisance without clipping the ball silhouette.
BLOB_MIN_PIXELS = _area(50)
BLOB_MAX_PIXELS = _area(900)
BLOB_MIN_DENSITY = 0.14
BLOB_MAX_ASPECT = 2.8
# Do not merge nearby neutral blobs: the ball's dark body already forms one
# component, while merging joins it to the adjacent black pipe rail.
BLOB_MERGE_BLOBS = False
BLOB_MERGE_MARGIN = 3
# The steel ball remains 18--26 detector pixels across.  A 3 x 3 sparse LAB
# pass still samples it densely while reducing the full-pipe acquisition work
# by 56% versus the former 2 x 2 scan.
BLOB_X_STRIDE = 3
BLOB_Y_STRIDE = 3
# In the rigid vehicle lighting, the accepted neutral LAB island is the dark
# right-hand part of the mirrored ball rather than its geometric centre.
# Four manually aligned points from stream_20260730_131248 measured a
# consistent 12--15 detector-pixel right bias.  Correct only strong blobs and
# do it along the live pipe axis, so weak fixture fragments remain untouched.
BLOB_CENTER_BIAS_ALONG_AXIS_PX = _sx(18)
BLOB_CENTER_BIAS_MIN_QUALITY = _area(70)

# Native Hough-circle recovery for the mounted car.  The 25--30 px steel ball
# has radius 12--16 px at 640x480.  The two permanent endpoint screws are
# smaller than 10 px.  A circle alone is not sufficient: the green pipe
# texture produces many Hough peaks, so sampled RGB chroma must also look
# neutral/metallic before a circle is handed to the tracker.
CIRCLE_RECOVERY_ENABLED = False
CIRCLE_THRESHOLD = 1100
CIRCLE_MIN_RADIUS = _sx(11)
CIRCLE_MAX_RADIUS = _sx(18)
CIRCLE_X_STRIDE = 2
CIRCLE_Y_STRIDE = 2
CIRCLE_RADIUS_STEP = 2
CIRCLE_X_MARGIN = 10
CIRCLE_Y_MARGIN = 8
CIRCLE_R_MARGIN = 6
# Full-ROI Hough is deliberately disabled.  The live vehicle logs show
# 180--400 ms stalls and many unrelated peaks whenever the track is absent.
# Native LAB still scans the narrow pipe ROI every frame and is the safe,
# deterministic acquisition path.
CIRCLE_ACQUIRE_ENABLED = False
CIRCLE_ACQUIRE_INTERVAL_FRAMES = 8
# At an endpoint the LAB blob may disappear.  Hough is therefore retained
# only for a confirmed track whose predicted point is already inside the
# endpoint zone, and it scans only the small predicted ROI.
CIRCLE_TRACK_INTERVAL_FRAMES = 6
CIRCLE_TRACK_ENDPOINT_ONLY = True
CIRCLE_COLOR_FILTER_ENABLED = True
CIRCLE_MAX_CHROMA = 40
CIRCLE_DARK_VALUE = 75
CIRCLE_MIN_NEUTRAL_SAMPLES = 8
# Weak endpoint fragments used to count as "a blob" and suppress Hough even
# though the tracker would reject them.  Require ball-level blob evidence
# before skipping native circle recovery.
CIRCLE_TRIGGER_MIN_QUALITY = _area(70)
CIRCLE_TRIGGER_MAX_AXIS_DISTANCE_PX = _sy(12)
# When the ball touches a stop, Hough locks onto the overlapping screw-side
# arc.  Move only endpoint circle candidates inward; their untouched raw
# coordinates remain attached to the tuple for fixture rejection.
CIRCLE_ENDPOINT_POSITION = 0.12
CIRCLE_ENDPOINT_INWARD_BIAS_PX = 0
CIRCLE_LEFT_ENDPOINT_INWARD_BIAS_PX = 0
CIRCLE_RIGHT_ENDPOINT_INWARD_BIAS_PX = 0
# The dynamically updated ball ROI is centred on the pipe axis.  Ignore
# circular ruler/PCB detail below it while retaining the ball above the axis.
CIRCLE_MAX_ABOVE_ROI_CENTER_PX = _sy(24)
CIRCLE_MAX_BELOW_ROI_CENTER_PX = _sy(9)

# Alpha-beta 1-D tracker.
# The ball rides on the upper wall of the trough, about 24--27 px above the
# green component's colour centroid in the mounted-car view.  Below the axis
# are PCB/rail fixtures, so keep that side deliberately tighter.
# Labelled car runs put the real ball centre between -11 and +6 reference
# pixels from the pipe axis.  The old 30/18 band admitted the white ruler
# above the pipe, where Hough repeatedly found stable false circles.
MAX_AXIS_DISTANCE_PX = _sy(12)
MAX_BELOW_AXIS_DISTANCE_PX = _sy(12)
MAX_FRAME_JUMP_PX = _sx(60)
# Legacy calibration overhang margins.  They remain useful if dynamic pipe
# pose or explicit endpoint insets are disabled; with the competition's
# non-zero endpoint insets below, the physical exclusion zone takes priority.
ACQUIRE_POSITION_MARGIN = 0.0
TRACK_POSITION_MARGIN = 0.0
# A ball center cannot physically sit on the green material's exact endpoint.
# Reflections do.  On the mounted car, however, the real ball at the right
# stop produces Hough centres up to position 0.962.  A 2% startup exclusion
# still rejects both outboard screws while allowing that physical end state.
ACQUIRE_ENDPOINT_INSET = 0.02
TRACK_ENDPOINT_INSET = 0.015
# A fresh target still needs two consecutive frames at quality 60.  Once the
# ball is confirmed, geometry, continuity and the short trusted-memory window
# are more reliable than blob quality during motion blur, so quality alone
# must not break an existing track.
ACQUIRE_MIN_QUALITY = _area(60)
TRACK_MIN_QUALITY = _area(20)
POSITION_ALPHA = 0.72
VELOCITY_BETA = 0.14
LATERAL_ALPHA = 0.55
CONFIRM_FRAMES = 2
COAST_FRAMES = 2
TRACK_MEMORY_FRAMES = 8

# The two endpoint screws are rigidly attached to the pipe.  Express their
# centres in pipe coordinates so the exclusions rotate with the measured
# pipe angle.  The 14 px reference radius masks only the screw/Hough halo;
# the nearest physical ball centres remain about 20--25 reference pixels away.
FIXTURE_EXCLUSION_ZONES = ()
# Strong blob evidence may cross a fixture core.  Hough-only peaks (the
# normal screw false positive) still cannot acquire there.
# The final fixed-car empty-pipe run proved that the endpoint screw itself
# can produce a strong LAB blob.  Do not override fixture rejection by blob
# quality; a real endpoint ball is recovered by the inward-corrected Hough
# candidate outside the small raw-coordinate fixture core.
FIXTURE_BLOB_OVERRIDE_QUALITY = 0
# Outside each small hard core, apply only a continuous cost.  This breaks a
# track's attraction to the screw halo while still letting a real ball
# candidate win as it approaches or overlaps the endpoint.
FIXTURE_SOFT_RADIUS_SCALE = 2.6
FIXTURE_SOFT_PENALTY_PER_PX = 2.0

# Once the ball physically reaches a stop, exact sub-pixel position is not
# useful to the controller.  Multiple reflective arcs otherwise make the
# measurement jump among nearby Hough peaks.  After two endpoint samples,
# report the calibrated stop centre until the raw ball leaves a wider
# hysteresis boundary.
ENDPOINT_SNAP_LEFT_POSITION = 0.050
ENDPOINT_SNAP_RIGHT_POSITION = 0.935
ENDPOINT_SNAP_ENTER = 0.105
ENDPOINT_SNAP_EXIT = 0.145
ENDPOINT_SNAP_CONFIRM_FRAMES = 2

# MaixCAM custom UART1: A19 TX -> STM32 PC7/USART6_RX,
# A18 RX <- STM32 PC6/USART6_TX. Both ends use 115200 8N1.
UART_ENABLED = True
UART_DEVICE = "/dev/ttyS1"
UART_TX_PIN = "A19"
UART_RX_PIN = "A18"
UART_BAUD = 115200
TELEMETRY_HZ = 50
# Disabled because the camera channel above is the single timing source.
CONTROL_LOOP_HZ = 0

# Preview is deliberately slower than detection so JPEG streaming to
# MaixVision cannot throttle the control-rate loop.
DISPLAY_ENABLED = True
PREVIEW_HZ = 5
CONSOLE_HZ = 2

# Competition debug stream.  Detection keeps its 480 x 360 channel and
# control rates; this lower-rate secondary channel prevents RTSP/VENC/network
# work from saturating the single CPU while the operator console is open.
STREAM_WIDTH = 448
STREAM_HEIGHT = 336
# Windows renders at 20 FPS, so encoding 30 FPS only competes with the
# 60-FPS control channel without improving the visible dashboard motion.
STREAM_FPS = 20
STREAM_BITRATE = 1000000
# Keep the RTSP camera channel at the minimum supported buffering.  The
# Windows receiver also discards stale decoded frames instead of letting a
# preview backlog build up.
STREAM_BUFFER_COUNT = 1
STREAM_RTSP_PORT = 8554
# Subscribers register by sending a UDP handshake to STREAM_CONTROL_PORT.
# Optional fixed destinations can be added here for a known lab network.
STREAM_TELEMETRY_TARGETS = ()
STREAM_TELEMETRY_PORT = 42101
STREAM_CONTROL_PORT = 42102
STREAM_TELEMETRY_HZ = 30
STREAM_STATUS_HZ = 1
STREAM_CAMERA_RETRIES = 5
STREAM_CONTROL_TOKEN = "pipe-ball-local"
STREAM_LOCAL_PREVIEW = False
STREAM_LOCAL_PREVIEW_HZ = 2

# Dynamic bench-test recording. The background recorder keeps video encoding
# outside the Python tracking loop; stop the run normally so MP4 is finalized.
RECORD_OUTPUT_ROOT = "/root/ball_tests"
RECORD_VIDEO_WIDTH = 448
RECORD_VIDEO_HEIGHT = 336
RECORD_FPS = 30
RECORD_VIDEO_BITRATE = 1500000
RECORD_LOG_FLUSH_FRAMES = 30
RECORD_MAX_SECONDS = 0
RECORD_CAMERA_RETRIES = 3
