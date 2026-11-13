# E2E test environment handoff — 2026-10-08

## Current assessment

Robot preparation prototype is implemented and documented in
`ROBOT_PREPARATION.md` (linked from README and the Prepare robot UI).
Test settings now exposes Prepare robot, environment discovery, geometry capture,
cached bundle loading and independent fixture review into editable drafts.
Existing CLI onboard remains CLI-only; reach-map generation is not exposed here.
The CLI and host recorder share extracted SDK mesh-fitting code.
Both Handy and G1 geometry bundles are complete, and all eight live snapshot
binding checks passed. Handy has independently generated clear/colliding arm
candidates. The G1 123-pair gap was a fixed-link identity mismatch, not missing
meshes. Fixed upstream in Grove's vendored g1_29dof/g1_23dof MJCF: named massless
fixed children for head_link, logo_link and pelvis_contour_link, fusestatic=false.
Fresh preparation now covers all pairs in all three groups. Sampled contexts
still yield colliding candidates; coverage alone does not prove a clear pose.
The regression test and docs are in the robot repo. Before/after checks verified
unchanged joint order, DOFs, geom poses, mass, mass matrices and bias forces for
both variants and the full staged scene. Validated XML was also copied into the
container for offline preparation; running simulation was not restarted. Vendor
submodule source edits remain uncommitted/unpublished.
No fixture settings were saved by automated browser checks, and preparation sent
no robot commands. SDK/dashboard self-tests: 441 passed. Browser review and
responsive layouts passed; a separate synthetic MuJoCo check verified direct
queries despite disabled dynamics contact masks and deterministic binary replay.
Bundles persist in ignored dashboard configurations; job IDs do not survive a
server restart. Load recorded bundles to reuse them.

Latest full G1 run: `20261008-211841-8ea18cde` — 110 live passed,
0 failed/errors, 46 not applicable, 19 untested; 300 offline SDK passed.
Status is **incomplete**, because 19 prerequisites remain missing. Breakdown:
5 navigation checks (current pose lacks 0.75 m known-free map clearance),
1 watch-stop condition, 1 model-change fixture, 3 missing geometry snapshots,
3 known-clear configurations, 3 known-colliding configurations and 3 collision
performance budgets. Full artifacts and SUMMARY.md are in the dashboard runs
folder. Simulated hands were activated for this run and returned to inactive
afterward. Precision multiplier 1.75 and clearance 0.75 m were the saved settings;
generated group probes used 0.05 rad / 0.01 rad tolerance / 2 s per waypoint.

User clarified that the earlier negative assessment was unintended. Work is unfinished; do not describe the
environment as ready just because dashboard/unit tests pass. Resume by reviewing
the actual latest G1 failures and the UI with the user in mind.

The goal is universal SDK compatibility testing for any ROS robot with a profile.
Do not introduce Handy/base101/G1-specific assumptions into shared checks.

## Running state

- Dashboard: http://127.0.0.1:8765. No test run active at handoff.
- G1 agent: ws://127.0.0.1:10102, container `ros_dev_jazzy`.
- Launch: `ros2 launch g1_bringup bringup.launch.py headless:=false cameras:=head,chest mode:=mapping nav:=true`.
- Latest focused group retest: `20261008-211350-g1-group-retest-9fd948`, passed,
  18 latest check results including targeted retries. Previous broader run:
  `20261008-204737-632ad46c`, failed, profile shown as `loaded: grove-g1`.
- Earlier failed runs: `20261008-194849-27fe08ce`, `20261008-191546-fc495d64`.
- Saved results live under the dashboard runs directory; inspect their actual
  failures and missing prerequisites before assigning blame to profiles or harness.
- Stack left running. No ROS restart, robot movement, or commit performed in the
  final UI/profile edits.

## Latest profile change — activated during G1 retest

The reported error was `CapabilityNotSupported: ... has no 'status' capability`.
Added `status: {}` to both:

- `/home/cdr/Work/Robots/grove-g1/workspace/src/g1_bringup/profiles/grove_g1.yaml`
- `/home/cdr/Work/Robots/grove-g1/profiles/grove_g1_mapping.yaml`

Both validate with the agent profile loader and advertise status. The empty
section exposes agent emergency-stop state; no battery topic is invented.
The agent was subsequently relaunched for the requested G1 retest. Its Welcome
advertises status, and SDK reads returned estop_engaged=False and battery=None.
The installed launch profile symlinks to the edited source file.

## Implemented, but still needs live qualification

- Subsequent automatic group-recipe implementation: Read-only / Hold / Small
  movements / Named poses; generated probes use measured baselines, joint types,
  native-unit budgets, explicit group eligibility, and legal command limits.
- Read-only preview UI and endpoint; execution regenerates and records the
  recipe, model SHA and baseline. Arrival checks require every probed joint to
  show observed movement; existing fixture restoration remains verified.
- Handy: 15 live read-only group checks passed. Head/lifts generated previews
  work. Both arm elbows are outside declared limits by roughly 0.0016–0.0019
  rad in the current measured state, so automatic arm probes refuse readiness.
  Do not silently expand limits to hide this discrepancy.
- Grove was subsequently restarted at the user's explicit retest request.
  Latest results: 18 group checks passed (including targeted retries): nine
  read-only checks, three holds, three generated trajectories and three
  cancellations. Every declared arm/hand joint showed measured motion. These
  are simulation results, not hardware or whole-suite certification.
- G1 validation used 0.05-rad probes and an explicit 0.01-rad acceptance tolerance.
  A 0.001-rad draft tolerance failed arm holding at roughly 0.0063-rad error.
- Fixed arrival/hold checks to sample within the active session, before another
  controller takes over. Numerical hand baseline noise around 2e-8 rad now uses
  the existing 1e-5 limit allowance, with recorded legal-bound clamping.
- Both simulated Dex3 hardware components start inactive and needed explicit
  activation. SDK sessions leave hand controllers active if no conflicting
  controller was replaced. Cleanup explicitly returned hand controllers and
  hardware components to their original inactive states. Arm freeze restored.
- Results, initial trial XML and retries are saved in
  `ringfour-sdk/test-results/dashboard/20261008-211350-g1-group-retest-9fd948`.
- The `status` profile change is now live: estop_engaged=False, battery=None.
- ROS launch required putting `/opt/ringfour` after the ROS/workspace entries in
  AMENT_PREFIX_PATH: its resource index otherwise shadows installed slam_toolbox
  with a prefix missing libexec. Keep this environment issue distinct from SDK
  compatibility failures. The simulation was left running.

- Default compatibility suite separated from qualification, diagnostics,
  software regressions, and offline SDK coverage.
- Dashboard now has two separate sidebars: Suites, then Categories, with the
  test view on the right. Tablet widths put the test view below the sidebars;
  phones stack navigation. Suite descriptions and icons are present.
- The first sidebar attempt lacked its CSS and displayed oversized icons.
  Fixed that omission; then split suites/categories into separate panels.
- Test settings exposes group trajectories, sessions, collision examples,
  tools, watches, navigation goals, map workflows, model-change commands,
  snapshots, custom actions, secondary transport, and runtime timeout.
- Untested results link to their relevant requirements form.
- Reading live joint state captures editable draft targets, native units,
  limits, and full measured state. It does not move the robot. Two identical
  captured baseline poses must be edited to exercise movement.
- Saving preserves other fixtures, validates targets against live limits, and
  binds group fixtures to the model SHA.
- Custom action checks now invoke configured arguments and require measured
  feedback change; optional restoration is checked. Missing fixtures remain
  untested, not passed.
- Earlier changes include YAML profile upload, movement precision tolerance,
  larger text, default debug tests, and known-free map goal selection. Navigation
  preview alone is not proof of successful live navigation.

## Evidence and limits

- SDK and dashboard self-tests: 416 passed, two existing deprecation warnings.
- Browser checked suite/category selection, bounded icons, and no horizontal
  overflow at 1440, 1200, 1000, 800, and 390 pixels after the split.
- Earlier browser check covered readonly G1 joint capture and requirements
  links; seven left-hand joints and 43 full-state joints were returned.
- These checks do NOT certify the current live G1 suite. Latest live run failed.
- No live rerun after adding status. No current claim that all failures are
  profile issues or that all advertised operations work.

## First steps tomorrow

1. Inspect the latest run's full failure/untested details, not just totals.
2. Activate and verify the status profile change without unnecessarily resetting
   the simulation. Confirm the failing check's status requirement is justified.
3. Compare loaded bindings with the selected independent golden YAML; report
   exactly which failures are profile, backend, harness, or missing fixtures.
4. Review sidebar usability visually and simplify requirement configuration
   where the UI exposes too much machinery.
5. Run targeted checks before another full live run. Physical commands need the
   existing explicit command enablement and reviewed targets/environment.

## Workspace caution

There is extensive uncommitted work across agent, SDK, docs, and tests, including
other kinematics work. Preserve it; do not reset, broadly stage, or overwrite it.
Dashboard and several e2e files are currently untracked. No commit was requested.
Related audit: `COMPATIBILITY_AUDIT.md`; test instructions: `README.md`.
