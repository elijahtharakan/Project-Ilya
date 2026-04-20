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

## Raspberry Pi 5 Setup

Recommended baseline on the Pi:

1. Flash Raspberry Pi OS 64-bit and update packages.
2. Enable the camera in `raspi-config` if needed.
3. Verify the camera shows up as a capture device before running the repo.
4. Install Python dependencies with either apt packages or a virtual environment.

Suggested install commands:

```bash
sudo apt update
sudo apt install -y python3 python3-pip python3-venv python3-opencv python3-numpy
```

If you are using the Pi Camera Module, also install Picamera2 support:

```bash
sudo apt install -y python3-picamera2
```

If you prefer a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install numpy opencv-python
```

Camera sanity check:

```bash
python puck_tracker.py --camera 0 --width 640 --height 480 --no-gui --max-frames 30 --send-stdout
```

For the Pi Camera Module specifically:

```bash
python puck_tracker.py --capture-backend picamera2 --width 640 --height 480 --no-gui --max-frames 30 --send-stdout
```

If that prints packets, the camera is working.

One-command Pi demo launcher:

```bash
python run_pi_demo.py
```

Top-down visual demo with a virtual paddle:

```bash
python run_pi_demo.py --visualizer
```

For Pi Camera Module use:

```bash
python run_pi_demo.py --capture-backend picamera2
```

If the Pi is headless or you want no windows:

```bash
python run_pi_demo.py --no-gui
```

Combine both for a headless Pi Camera Module run:

```bash
python run_pi_demo.py --no-gui --capture-backend picamera2
```

If your camera is not index `0`, change it with `--camera N`.

## Run Commands

Tracker:

```bash
python puck_tracker.py --camera 0 --width 640 --height 480 --udp-host 127.0.0.1 --udp-port 5005
```

Visual camera verification (with velocity arrow overlay):

```bash
python puck_tracker.py --camera 0 --width 640 --height 480
```

If the camera preview looks mirrored or upside down, add one of:

```bash
python puck_tracker.py --camera 0 --width 640 --height 480 --flip-horizontal
python puck_tracker.py --camera 0 --width 640 --height 480 --flip-vertical
```

Notes:
- Click the puck once in the Puck Tracker window to auto-set HSV bounds.
- Move the puck around and watch the blue arrow and velocity text update in real time.
- Use `--velocity-arrow-scale` to tune arrow length (default `0.15`).
- A pink vertical line shows the robot goal/intercept line (`--intercept-x-px`), and a pink dot shows predicted crossing point.
- The overlay text `INTERCEPT y:... t:...s` updates as you move the puck.

Example with explicit intercept line and larger arrow:

```bash
python puck_tracker.py --camera 0 --width 640 --height 480 --intercept-x-px 120 --velocity-arrow-scale 0.2
```

Estimator:

```bash
python puck_state_estimator.py --listen-port 5005 --output-udp-host 127.0.0.1 --output-udp-port 5006 --intercept-x 120
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
python synthetic_puck_feed.py --host 127.0.0.1 --port 5005 --mode showcase --seconds 12
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

Top-down visual demo with synthetic puck motion and a virtual paddle:

```bash
python run_no_hardware_demo.py --visualizer
```

The default synthetic mode is `showcase`, which walks through multiple puck behaviors so the planner can visibly switch between defense, recovery/home, and attack-style responses.

Optional launcher flags:

```bash
python run_no_hardware_demo.py --prediction-source planner --mode zigzag --feed-seconds 10
python run_no_hardware_demo.py --visualizer --mode attack --feed-seconds 8
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
python puck_state_estimator.py --listen-host 127.0.0.1 --listen-port 5005 --output-udp-host 127.0.0.1 --output-udp-port 5006 --intercept-x 120
```

4. Terminal D (synthetic tracker feed):

```bash
python synthetic_puck_feed.py --host 127.0.0.1 --port 5005 --mode showcase --seconds 12
```

Expected result:
- Terminal A prints paddle target packets.
- Terminal B prints mode/target JSON packets.
- No camera or motor controller hardware is required.

## Visualizer Demo

`table_visualizer.py` listens to planner output packets and renders a top-down air hockey table view.
It shows:
- planner mode and reasoning
- puck position and velocity
- predicted intercept marker
- target paddle command
- a simulated virtual paddle moving toward the commanded target

You can run it by itself if another process is already publishing planner packets:

```bash
python table_visualizer.py --listen-host 127.0.0.1 --listen-port 5007
```

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

Camera hardware verification test:

```powershell
$env:RUN_CAMERA_TESTS="1"
python -m unittest tests/test_camera_pipeline.py -v
```

Notes:
- This test requires a webcam connected and available as camera index 0.
- The test runs tracker in `--no-gui` mode and verifies that real camera packets are emitted over UDP.
