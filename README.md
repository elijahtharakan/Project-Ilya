# Project-Ilya
Autonomous Air Hockey Robot

## Full Pipeline

The system is split into independent processes so planning and motor control remain decoupled:

1. puck_tracker.py
   - Reads camera frames.
   - Detects puck position by HSV color segmentation.
   - Publishes raw tracker packets over UDP (default port 5005).

2. puck_state_estimator.py
   - Receives tracker packets.
   - Estimates puck velocity.
   - Predicts intercept on a configured image line.
   - Publishes enriched packets over UDP (default port 5006).

3. planner_motor_bridge.py
   - Receives estimator packets.
   - Converts pixel observations into planner table coordinates.
   - Runs air_hockey_planner decision logic.
   - Publishes motor target intent packets over UDP (default port 5007).

4. Motor controller process (your code)
   - Receives target intent packets.
   - Performs actuator-specific control loops and hardware commands.

## Run Commands

Tracker:

```bash
python puck_tracker.py --camera 0 --width 640 --height 480 --udp-host 127.0.0.1 --udp-port 5005
```

Estimator:

```bash
python puck_state_estimator.py --listen-port 5005 --output-udp-host 127.0.0.1 --output-udp-port 5006 --intercept-y 420
```

Planner bridge:

```bash
python planner_motor_bridge.py --listen-port 5006 --motor-host 127.0.0.1 --motor-port 5007 --output-stdout
```

Planner bridge using planner-owned prediction:

```bash
python planner_motor_bridge.py --listen-port 5006 --motor-host 127.0.0.1 --motor-port 5007 --prediction-source planner --output-stdout
```

Synthetic estimator input (no camera):

```bash
python synthetic_puck_feed.py --host 127.0.0.1 --port 5005 --mode toward --seconds 6
```

Mock motor controller receiver:

```bash
python mock_motor_controller.py --listen-host 127.0.0.1 --listen-port 5007
```

## No-Hardware End-to-End Test

Run this full pipeline on a laptop with no camera and no motor hardware:

One command launcher:

```bash
python run_no_hardware_demo.py
```

Optional launcher flags:

```bash
python run_no_hardware_demo.py --prediction-source planner --mode zigzag --feed-seconds 10
python run_no_hardware_demo.py --stay-alive
```

1. Terminal A (mock motor receiver):

```bash
python mock_motor_controller.py --listen-host 127.0.0.1 --listen-port 5007
```

2. Terminal B (planner bridge):

```bash
python planner_motor_bridge.py --listen-host 127.0.0.1 --listen-port 5006 --motor-host 127.0.0.1 --motor-port 5007 --prediction-source estimator --output-stdout
```

3. Terminal C (estimator):

```bash
python puck_state_estimator.py --listen-host 127.0.0.1 --listen-port 5005 --output-udp-host 127.0.0.1 --output-udp-port 5006 --intercept-y 420
```

4. Terminal D (synthetic tracker feed):

```bash
python synthetic_puck_feed.py --host 127.0.0.1 --port 5005 --mode toward --seconds 8
```

Expected result:
- Terminal A prints paddle target packets.
- Terminal B prints mode/target JSON packets.
- No camera or motor controller hardware is required.

## Packet Summary

Tracker packet fields:
- timestamp
- detected
- x, y
- x_norm, y_norm
- radius
- frame_width, frame_height

Estimator packet fields:
- timestamp
- source_timestamp
- detected
- x, y
- vx_px_s, vy_px_s, speed_px_s
- intercept_valid
- intercept_x, intercept_y
- time_to_intercept_s

Planner bridge output packet fields:
- timestamp
- source_timestamp
- command_type (paddle_target)
- target_x_m, target_y_m
- mode
- reason
- stale_data, clamped, safety_limited
- estimated_puck_vx_m_s, estimated_puck_vy_m_s
- debug_summary

## Automated Tests

Run all tests:

```bash
python -m unittest discover -s tests -v
```

No-hardware integration coverage includes:
- `tests/test_no_hardware_pipeline.py` which starts `planner_motor_bridge.py`, sends a synthetic estimator packet, and verifies a motor target packet is emitted.
