# Motor-effort safety characterization

`examples/effort_safety_characterization.py` is a hardware-only data collector for tuning the STS3215 current/load safety interlock and estimating software stop latency.

It records one CSV row per motor per sample with:

- raw `Present_Current`
- signed `Present_Load`
- configured current/load threshold for that motor
- all five arm joint positions
- normalized gripper position
- estimated joint/gripper velocity between samples
- whether the sample exceeded a threshold
- whether the effort interlock was latched
- time spent reading the telemetry sample

A companion `.summary.json` reports max, median, and 95th-percentile absolute current/load by motor plus the first sampled threshold crossing, observed interlock latch, and estimated settle time.

## Recommended sequence

Run multiple repetitions and keep each physical condition in a separate file.

### 1. Powered idle

```bash
python examples/effort_safety_characterization.py \
  --port /dev/ttyACM0 \
  --mode observe \
  --condition powered-idle
```

### 2. Unloaded joint motion

Start with a small, slow move and repeat for each joint in both directions.

```bash
python examples/effort_safety_characterization.py \
  --port /dev/ttyACM0 \
  --mode joint \
  --condition unloaded-elbow-positive \
  --joint elbow_flex \
  --delta-deg 5 \
  --joint-speed-deg-s 5
```

Run again with `--delta-deg -5` for the other direction. Repeat with expected payloads attached so normal payload effort is represented before choosing final trip limits.

### 3. Gentle contact trial

Use a compliant obstruction such as foam. Do not use a hand or body part as the obstacle.

```bash
python examples/effort_safety_characterization.py \
  --port /dev/ttyACM0 \
  --mode joint \
  --condition foam-contact \
  --joint elbow_flex \
  --delta-deg 5 \
  --joint-speed-deg-s 3 \
  --prompt-before-motion
```

The SDK should latch the effort trip and hold the measured position. The script does not clear the trip or retry the motion automatically.

### 4. Gripper contact

```bash
python examples/effort_safety_characterization.py \
  --port /dev/ttyACM0 \
  --mode gripper \
  --condition foam-gripper-contact \
  --gripper-start 1.0 \
  --gripper-target 0.0 \
  --gripper-speed 80 \
  --prompt-before-motion
```

This is useful for selecting a gripper-specific threshold that can be lower than the shoulder/elbow thresholds.

## Baseline collection without the interlock

For normal-motion characterization only, the software effort interlock can be disabled while telemetry is still recorded:

```bash
python examples/effort_safety_characterization.py \
  --port /dev/ttyACM0 \
  --mode joint \
  --condition unloaded-baseline \
  --joint elbow_flex \
  --delta-deg 5 \
  --disable-interlock
```

Use this mode only in a clear workspace with conservative motion limits and immediate access to power. Existing workspace, joint-limit, following-error, and controller safety checks still apply, but current/load contact protection is disabled for that trial.

## Measuring feedback cadence

The current controller checks effort through the same feedback path used for active motion monitoring. Compare repeated trials at the default `0.10 s` feedback interval and a tighter interval such as `0.05 s`:

```bash
python examples/effort_safety_characterization.py \
  --port /dev/ttyACM0 \
  --mode joint \
  --condition foam-contact-50ms \
  --joint elbow_flex \
  --delta-deg 5 \
  --joint-speed-deg-s 3 \
  --feedback-interval 0.05 \
  --sample-hz 20 \
  --prompt-before-motion
```

Watch `sample_read_duration_s` in the CSV. If serial reads take longer than the requested sample period, the external collector cannot achieve the requested rate and its latency estimate is correspondingly coarse.

## Interpreting the summary

The most useful values are the per-motor normal-motion distributions and the gap between normal/payload effort and deliberate contact effort. Choose thresholds with margin above normal motion rather than simply setting them near one observed contact value.

`observed_crossing_to_trip_ms` and `observed_trip_to_stop_ms` are experimental estimates. They include external serial polling effects and are not certified emergency-stop reaction times. For tighter reaction-time characterization, the next step is to timestamp the threshold sample and hold command inside the backend itself and compare those internal timestamps with high-rate position or video measurements.
