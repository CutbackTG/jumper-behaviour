# Po: stationary attention prototype

The first implementation reuses the exported posture policy. It generates body
twist, pitch and roll commands, with zero walking velocity and the contract's
neutral height. It does not drive joints directly or change the robot model.

`deploy/po.json` holds deliberately small prototype tuning values, not measured
hardware limits. Opt-in checks them against the loaded policy's training ranges.
Pitch is nose-down positive: looking up requests negative pitch. No head joint
exists in the current robot; the body aims the camera.

## Run on Windows

Complete the environment setup in [AGENT_SETUP.md](AGENT_SETUP.md) first. Use
this clone's `.venv`, not the design environment. From the repository root:

```powershell
.\.venv\Scripts\Activate.ps1
& "$env:USERPROFILE\.cargo\bin\cargo.exe" build --manifest-path deploy/fsm/Cargo.toml --release --no-default-features --features py --locked
python scripts/play.py --app out/bundle_example/jumper --controller deploy/fsm/target/release/mjrl_fsm.dll --passive-demo deploy/po.json --backend native --device cpu
```

The explicit `--controller` is a development override. It uses the newly built
extension with the example app's existing policies; it does not replace any
example runtime or claim to be a rebuilt upload bundle. Without the two new
options, app playback behaves as before.

For a bounded recording, add `--headless --steps 6000 --video outputs/po.mp4`.
The demo uses a synthetic direction that varies sideways and upward. Its bearing
is recomputed from the simulated body attitude, so the error changes as Po turns.
During seconds 16 through 30 of each cycle there are no target observations.
This exercises brief fixation after loss, return to neutral, and idle scanning.
It does not prove camera tracking. The synthetic coordinate frame is aligned to
the body; the live camera adapter will need calibrated camera-to-body geometry.

Attention starts once the normal measured-pose ramp has finished. A key press,
active gamepad control, non-neutral command, mode change, stale operator or motor
feedback, or safety/interlock state cancels it. Releasing a control does not
restart it: restart the demo to opt in again. Safety and manual takeover may
interrupt a posture immediately; ordinary target loss uses a gradual return.

## Integration and checks

`deploy/fsm/src/passive.rs` owns the deterministic timing and bounded commands.
`Controller::tick` selects the passive command after operator selection and before
command shaping, observation, policy inference and the existing joint decoder.
Python, browser and device adapters expose opt-in and face-observation methods.
Face observations do not count as operator heartbeat.

The face input is azimuth (positive left), elevation (positive up), confidence
in `[0,1]`, exposure time and reception time in the controller's monotonic
microsecond clock. The host selects a target and transforms image coordinates
into calibrated bearings. Nonfinite, future, stale and reordered frames are
rejected. One exposure cannot repeatedly integrate a correction.

```powershell
& "$env:USERPROFILE\.cargo\bin\cargo.exe" test --manifest-path deploy/fsm/Cargo.toml --no-default-features --lib --locked
& "$env:USERPROFILE\.cargo\bin\cargo.exe" check --manifest-path deploy/fsm/Cargo.toml --target wasm32-unknown-unknown --no-default-features --features web --locked
```

These checks cover command direction, rate and range bounds, target loss, operator
arbitration, and cancellation. They are not physical stability or fit tests.

Initial checks on this Windows machine: 189 portable Rust tests passed, the
browser interface compiled, and the updated Python extension ran 800 actual
posture-policy ONNX inferences with finite outputs using static synthetic motor
feedback. Positive target elevation produced negative pitch (down to the chosen
-0.20 rad cap), and Escape cancelled attention. Static feedback cannot establish
that the robot physically follows a face or remains stable. The complete Python
simulation environment, device build and visual replay still need verification.

## Next milestones

1. Validate the existing posture policy and this attention demo in simulation.
2. Add a live camera adapter with stable face selection, calibrated bearings,
   exposure timestamps, measured latency and target-loss handling. Do not assume
   a desktop webcam demonstrates the robot's own camera path.
3. Validate supported single-front-arm gestures separately before combining them
   with tracking. Approach, retreat, startle and recovery are not implemented here.
4. Define board opt-in and perception transport, then rebuild every runtime and
   package through the repository's deploy workflow as `po.app`. The current
   prototype is not an upload-ready app or hardware-tested controller.

Po's appearance remains in the separate `jumper-design` repository as a `.skin`.
There is no skin importer in this training repository.
