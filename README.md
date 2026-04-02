# Project-Ilya
Autonomous Air Hockey Robot

## Puck Tracker

Run the live puck tracker:

```bash
python puck_tracker.py
```

Useful options:

```bash
python puck_tracker.py --camera 0 --width 640 --height 480 --min-area 500
python puck_tracker.py --send-stdout
python puck_tracker.py --udp-host 127.0.0.1 --udp-port 5005
```

Coordinate packet format (JSON):

```json
{
	"timestamp": 1712083200.123,
	"detected": true,
	"x": 312,
	"y": 209,
	"x_norm": 0.4875,
	"y_norm": 0.4354,
	"radius": 18.2,
	"frame_width": 640,
	"frame_height": 480
}
```

`detected=false` means no puck was found in that frame (`x`/`y` become `-1`).

## Velocity + Intercept Estimator (Standalone)

This stage is a separate process that receives tracker packets and outputs enriched puck state.

Run estimator (reads tracker UDP on 5005):

```bash
python puck_state_estimator.py --listen-port 5005 --output-stdout
```

Forward enriched state to your control process over UDP:

```bash
python puck_state_estimator.py --listen-port 5005 --intercept-y 420 --output-udp-host 127.0.0.1 --output-udp-port 5006
```

Estimator packet format:

```json
{
  "timestamp": 1712083201.456,
  "source_timestamp": 1712083201.430,
  "detected": true,
  "x": 312.0,
  "y": 209.0,
  "vx_px_s": -180.2,
  "vy_px_s": 245.8,
  "speed_px_s": 304.8,
  "intercept_valid": true,
  "intercept_x": 146.7,
  "intercept_y": 420.0,
  "time_to_intercept_s": 0.86
}
```

## Recommended Runtime Process Split

Use three independent processes:

1. Perception process (`puck_tracker.py`)
	- Reads camera frames.
	- Detects puck center each frame.
	- Publishes raw coordinates (`x`, `y`, `detected`).

2. State-estimation process (`puck_state_estimator.py`)
	- Receives raw coordinates.
	- Computes velocity (`vx_px_s`, `vy_px_s`) from recent history.
	- Predicts intercept point on your robot defense line (`intercept_y`).

3. Control process (your robot striker planner)
	- Receives enriched state.
	- Chooses striker target from `intercept_x` and `time_to_intercept_s`.
	- Sends motor commands.

## What Should Receive Coordinates?

Your robot control/planning process should receive the enriched packets from `puck_state_estimator.py`, not raw camera pixels directly.

Why this is better:

1. Isolation of responsibilities
	- Camera/vision can fail or restart without rewriting control logic.
2. Stable interface for control
	- Control always reads the same fields (`intercept_x`, `time_to_intercept_s`, velocity).
3. Easier tuning
	- HSV tuning and velocity smoothing can be changed without touching motor code.
4. Lower coupling
	- You can later replace vision model and keep the same control receiver protocol.

Minimum receiver behavior for your control process:

1. Listen on UDP `5006` (or your configured estimator output port).
2. Parse JSON packet.
3. Reject packet if:
	- `detected == false`, or
	- `intercept_valid == false`, or
	- packet age too large (for example older than 100 ms).
4. If valid, map `intercept_x` from camera pixels to table/actuator coordinates.
5. Plan striker motion to reach intercept position before `time_to_intercept_s`.

Suggested startup order:

1. Start control receiver process.
2. Start `puck_state_estimator.py` with UDP output to control port.
3. Start `puck_tracker.py` with UDP output to estimator input port.

## Automated Tests

Run all unit tests:

```bash
python -m unittest discover -s tests -v
```

Current coverage includes:

1. `puck_tracker.py`
	- Puck detection from synthetic mask images.
	- Tracker packet format and normalization values.
2. `puck_state_estimator.py`
	- Velocity calculation from sample history.
	- Intercept validity, time-to-intercept, and X clamping behavior.

## Synthetic Estimator Feed (No Camera Needed)

Send synthetic tracker packets to the estimator:

```bash
python synthetic_puck_feed.py --host 127.0.0.1 --port 5005 --mode toward --seconds 6
```

Modes:

1. `toward`: puck moves toward intercept line (should produce `intercept_valid=true`).
2. `away`: puck moves away from intercept line (should produce `intercept_valid=false`).
3. `zigzag`: abrupt lateral changes for stress testing.

## Camera Test Workflow (Laptop or Pi)

Use this to verify full real-camera behavior end-to-end.

1. Start estimator (Terminal A):

```bash
python puck_state_estimator.py --listen-host 127.0.0.1 --listen-port 5005 --output-stdout --intercept-y 420
```

2. Start tracker with UDP output (Terminal B):

```bash
python puck_tracker.py --camera 0 --width 640 --height 480 --udp-host 127.0.0.1 --udp-port 5005
```

3. In the tracker window:
	- Place puck in view.
	- Click puck once to auto-set HSV range.
	- Adjust trackbars only if needed.

4. Confirm tracker output visually:
	- `TRACKING x:... y:...` appears on puck.
	- Yellow circle and red center dot follow puck.
	- `NO PUCK DETECTED` appears when puck leaves frame.

5. Confirm estimator output in Terminal A:
	- `detected` toggles correctly.
	- `vx_px_s`/`vy_px_s` change when puck moves.
	- `intercept_valid` becomes `true` only when motion heads toward `intercept_y`.
	- `time_to_intercept_s` generally decreases as puck approaches the intercept line.

6. Quick diagnostics if it does not track:
	- Increase lighting consistency (reduce glare/shadows).
	- Re-click puck color.
	- Lower `--min-area` in tracker if puck appears too small.
	- Verify both processes use same UDP host/port pair.
