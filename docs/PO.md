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
$env:PYTHONUTF8 = "1"
& "$env:USERPROFILE\.cargo\bin\cargo.exe" build --manifest-path deploy/fsm/Cargo.toml --release --no-default-features --features py --locked
python scripts/play.py --app out/bundle_example/jumper --controller deploy/fsm/target/release/mjrl_fsm.dll --passive-demo deploy/po.json --backend native --device cpu
```

The explicit `--controller` is a development override. It uses the newly built
extension with the example app's existing policies; it does not replace any
example runtime or claim to be a rebuilt upload bundle. Without the two new
options, app playback behaves as before.

For a bounded recording, add `--headless --steps 6000 --video outputs/po.mp4`.
For a front three-quarter view, also add `--video-azimuth 140`.
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
that the robot physically follows a face or remains stable. A subsequent full environment check passed all six setup gates, including the
device build and runtime load. The five-iteration GPU training smoke test also
completed; it is an environment check, not a newly trained behaviour policy.

Native MuJoCo replay on 2026-10-24 completed an eight-second neutral baseline
and a 30-second synthetic attention run. After the first three seconds, the
baseline stayed within 0.9 degrees of level with 2.1 mm maximum horizontal
displacement. Attention reached 9.0 degrees nose-up, with 11.6 mm maximum
horizontal displacement. It entered holding at 16.4 seconds, returning at 17.4
and scanning at 21.4; the controller remained in locomotion throughout. These
are single-run simulation measurements, not hardware stability guarantees.
The replay uses the standard Jumper appearance and no live camera input.

## Next milestones

1. Review the recorded attention motion and tune its timing in simulation.
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

## Submission target: po.app

The deliverable is the `.app` produced by `scripts/deploy.py`, following
`deploy/BUNDLE.md` and the deploy skill. Do not rename a source archive or ship
an `--allow-incomplete` build. The Po `.skin` remains a separate Design artifact.
Before the release, compare this checkout's bundle protocol and schema with
KingKongRobotics/jumper main; upstream may have changed since this fork.
These are repository checks, not a claim about an unpublished submission rubric.

Release requirements, still pending:

- Make attention configuration and explicit opt-in available through the bundled
  controller on board, web and simulation. The current `--passive-demo` reads an
  external JSON file and sends synthetic observations; it is a development harness.
  Adding a loose `po.json` to the archive is insufficient: the bundle schema,
  packer and readers must agree on how configuration is carried and consumed.
- Define the robot camera/perception transport, face selection, calibrated bearings
  and monotonic exposure timestamps. Verify lost/stale frames and manual takeover
  using the actual board integration. Device API methods alone do not supply frames.
- Create a named `po` deployment manifest using verified posture exports and an
  explicit control scheme. Preserve safety, joint limits and training ranges.
- Build with the normal deploy workflow: fresh aarch64 controller, ONNX and
  matching RK3576 RKNN models, web runtime and supported playback extensions.
  Docker is required for the board cross-build and RKNN conversion on Windows.
- Pass bundle schema/file validation and controller loading for every declared
  host. Add the generated English manual and checked Chinese translation through
  the bundle-manual skill; translation repacks the archive.
- Replay reference vectors through the bundled runtimes. Observation and target
  error must be within 1e-6 with no mode mismatch; assess RKNN action error
  separately against the converter's tolerance. Default reference vectors alone
  do not exercise passive tracking: separately exercise acquisition, tracking,
  target loss, scanning, cancellation and safety through the shipping runtime.
- Replay the final `.app` directly with `scripts/play.py --app <path-to-po.app>`
  using its own extension, without `--controller` or external passive config.
  Exercise every shipped mode and verify no missing runtime, model or stub.
- Extract the final archive to a clean directory, check ZIP integrity, root-level
  `bundle.json`, exact file inventory and every recorded file digest. Check the
  board binary's executable permission. Run the board's read-only compatibility
  preflight, `--dry-run` and `--check-reference` when a board is available.
- Record the source commit, build/tool versions, checks and final archive SHA256
  outside the bundle. Test and hash after translation; any repack invalidates
  the previous release hash. Report board checks as pending if no board was used.

Do not call the prototype submission-ready until these checks pass on the actual
archive. If the submission portal publishes additional limits or rules, check
those against the finished artifact as well.
