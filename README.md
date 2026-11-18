# Washer Vision & CNC XY Alignment — Team 9

## Status and firmware verification

**Phase 19 completed one physically confirmed washer-centering trial.** Repeatability and automatic pickup have **not** been validated. Phase 20 continuous alignment is **not implemented** here. The camera-to-machine mapping remains provisional (`verified: false`).

The supplied `arduino/WasherVision_Phase7_Z_BottomLimit_Fix.ino` implements the serial commands and detailed status fields expected by Phase 19: `MODE AUTO`, `RESETSTOP`, `HOME XY`, `MOVE`, `STATUS`, and `STOP`. The code was reviewed for command compatibility, **but it has not been confirmed byte-for-byte as the sketch currently flashed on the Arduino, nor retested after this upload**. Do not flash or replace working firmware solely because this file is present. Confirm with the original operator and perform controlled bench checks before use.

**Do not use `joystick_serial_bridge.ino` for Phase 19 motion.** It is an older diagnostic/manual bridge with incompatible commands. It remains at the repository root for reference.

## Repository layout

Most Python files, model weights and tests are at the **repository root**. The supplied Phase 7 Arduino sketch is under `arduino/`. The camera script defaults to looking for `models/best.pt`, so the explicit `--model best.pt` option shown below is necessary for this layout.

## Included components

- `phase19_corrected_xy_alignment.py` — verified-to-execute supervised XY controller (one successful physical centering trial)
- `phase5_supervised_controller.py`, `phase10_supervised_xy_alignment.py`, `phase12_integrated_xy_alignment_go_to_view.py`, `phase5_bridge_calibrated.py` — Phase 19 Python dependencies
- `phase13_vision_auto_feed.py` — YOLO camera feed and fusion CSV producer
- `vision_utils.py`, `dimension_check.py`, `dimension_calibration.py`, `class_fusion.py` — vision dependencies
- `best.pt` — trained YOLO weights
- `calibration.json`, `camera_machine_mapping_phase19_y_reversed.json`, `camera_machine_mapping_calibrated.json` — local calibration, **not portable without recalibration**
- `arduino/WasherVision_Phase7_Z_BottomLimit_Fix.ino` — **supplied candidate compatible firmware; verify against flashed sketch and test safely**
- `joystick_serial_bridge.ino` — older diagnostic/manual bridge only; incompatible with Phase 19
- `test_*.py` — existing software tests; not a hardware safety certification

## Environment (macOS)

```bash
conda env create -f environment.yml
conda activate washer_vision
```

Check webcam access, the Arduino device path, machine-specific calibration and actual firmware before running any hardware commands.

## Camera feed (separate Terminal)

```bash
python phase13_vision_auto_feed.py --model best.pt --width-mm 101.6 --height-mm 127 --auto-save-seconds 1
```

This loads the uploaded `calibration.json` and periodically writes `washer_coordinates_fusion.csv` when a single calibrated washer is detected. The `--auto-save-seconds 1` option is required for periodic CSV updates; the script's default is 0 (disabled). Check camera image size, calibration corners and workspace dimensions against the actual setup; the above command is a starting point, **not a verified launch record from the successful trial**. Keep the feed running while using `PREVIEW` or `ALIGN XY`.

## Phase 19 controller (ONLY after installed firmware and safety are verified)

```bash
python phase19_corrected_xy_alignment.py --port /dev/cu.usbmodem1301
```

Opening serial resets the Arduino. In `Phase19>` use `STATUS`, `MODE AUTO`, `RESETSTOP`, `HOME XY`, `GO TO VIEW`, `PREVIEW`, then `ALIGN XY`, checking results at each stage. The controller requires a fresh home and VIEW for each alignment. Confirm all three operator prompts only after inspecting the **entire** X-then-Y path. Each command moves at most 25 steps, and the controller checks reported positions and issues STOP at completion or error. No Z commands are issued.

**Safety:** Z raised; gripper detached; joystick centered; physical emergency stop available; no people or obstructions in the travel envelope. Inspect physical alignment independently. Do not rely on serial STOP as a physical emergency stop. Do not use this project unattended.

## Handoff and next development

1. Confirm the Phase 7 sketch matches the **actual firmware currently running on the Arduino**, including pinout, switch polarity and motion protocol; do not flash the diagnostic bridge. Review and bench-test any firmware changes before powering motors.
2. Confirm camera calibration and machine coordinate mapping on the specific hardware.
3. Run five supervised XY repeatability trials and log targets, final counters and physical centering in `phase20_repeatability_results.csv`.
4. Develop Phase 20 as a separate controller to reuse the verified current XY position without homing between washers; maintain STOP, freshness, movement and position checks.
5. Integrate Z-axis and vacuum pickup only after separate risk assessment and verification.

No secrets or credentials should be committed to this public repository. The included calibration is machine-specific and must not be assumed safe on another machine.
