# Project-Ilya
Autonomous Air Hockey Robot

## Current integration

- `puck_tracker.py` reads webcam frames, tracks the puck by color, and now feeds timestamped puck detections into `air_hockey_planner.py`.
- The planner returns safe paddle target coordinates plus debug/status information.
- The current camera-to-planner mapping is a temporary pixel-space bridge for testing before full table calibration is added.
