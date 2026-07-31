# Selected vision run logs

Raw recordings, UDP dumps, FFmpeg logs, and routine sessions stay local.
Only compact runs that are useful for reviewing or reproducing a performance
result are committed.

## `stream_20260730_144332`

Final long-running sample for MaixCAM release `637c886f6b26f59d`:

- detector channel: 480 x 360;
- RTSP preview: 320 x 240 at 30 FPS;
- fixed-center, fixed-length pipe pose with a 180 x 44 angle crop;
- 204.18 seconds and 5,791 analyzed tracking frames;
- mean detector rate 46.75 FPS;
- detector P95 25 ms and loop interval P95 49 ms;
- ball output validity 100%;
- pipe-pose validity 100%.

`tracking.csv` is sufficient to rerun:

```powershell
python Vision\tools\analyze_tracking_log.py `
  Vision\captures\stream_sessions\stream_20260730_144332\tracking.csv
```

## `stream_20260731_151006`

Task 9 motor-position / relative X-angle calibration run:

- 41.63 seconds and 755 decoded video frames;
- 207 unique Q9 frames, of which 205 contain valid IMU and position data;
- robust linear fit using 177 settled samples;
- approximately 499.43 motor counts per degree;
- settled-sample RMSE 0.179 degrees and R-squared 0.999308;
- raw video, telemetry, video timestamps, cleaned samples, fit summary, and
  key-frame contact sheet are committed together.

See
[`Q9_MOTOR_ANGLE_MAPPING.md`](stream_sessions/stream_20260731_151006/Q9_MOTOR_ANGLE_MAPPING.md)
for the equations, validated range, timing caveat, and reproduction command.
