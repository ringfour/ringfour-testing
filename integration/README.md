# Live robot conformance

This suite tests the Ringfour contract advertised by **the loaded agent**.
It contains no robot-name or locomotion whitelist. Connection, capability,
sensing, description, and joint-group checks use live metadata. Every camera,
group, range sensor, vendor sensor, wrist sensor, and power lane is collected
as its own test instance. Arms, lifts, pointing groups, arbitrary joint names,
and arbitrary joint counts use the same shared tests.

A humanoid, quadruped, wheeled robot, or aircraft can expose different subsets
of that contract. Missing capabilities are **not applicable**, not failures.
Advertised interfaces that fail their contract remain failures. A capability
with no shared test contract (for example a future `flight` capability) is
reported **untested**, never silently counted as validated.

## Local console

From the repository root:

```bash
ringfour-sdk/.venv/bin/python ringfour-testing/integration/dashboard/server.py
# http://127.0.0.1:8765
```

The local console follows mod101's configurator and Mr Handy's Ringfour Viz:
dark panels, teal accents, shared typography, and monospaced telemetry. It uses
Python's built-in HTTP server and the existing SDK environment (`pytest`,
`pyyaml`, SDK); no additional web framework is needed.

Enter the agent address and **Inspect running robot**. The default profile
selection uses live discovery; its displayed YAML is a handshake snapshot,
not the original ROS interface configuration. To check an independent profile,
add its directory when starting the console:

```bash
ringfour-sdk/.venv/bin/python ringfour-testing/integration/dashboard/server.py \
  --profiles-dir /home/cdr/Work/mr_handy/profiles
```

Repeat `--profiles-dir` for other robot workspaces. Profiles under this
checkout's `robots/*/profiles/` are discovered automatically. Selecting a YAML
does not reload the agent. Its name/model must match the loaded agent; the
profile check also compares version, capability sections, group names,
controller mappings, explicitly declared joint ordering, and camera names.

Choose a category and press the play button in the top toolbar. **Read checks
are the default.** Enable commands in that toolbar to include movement tests.
Straight drive, Figure eight, Braking time, and Odometry spin use built-in
defaults automatically; no scenario file or configuration steps are required. The suite launches no ROS, Docker, simulator,
robot, or agent processes, and never tears down your running stack.

Hardened agents advertise connection grants in `Welcome.features.access`.
TCP is read-only by default; enabling commands in this console does not grant
agent authority. Use the operator's Unix connection or an agent explicitly
started with `--trusted-tcp` within a trusted simulator/private deployment.
That flag grants operator access to all TCP peers, not authenticated/encrypted
remote identity. Recovery remains restricted to verified local operators.
Motion and simulated fault checks lacking required grants are **untested / not
configured**, with an access setup reason. Older agents retain their existing
behavior. See [hardening behavior](../ringfour-sdk/docs/hardening-progress.md)
for measured-stop policy and recovery prerequisites.

Results stream per test, with durations, failure/skip details, search and
status filters. History includes the loaded handshake, exact profile/scenario
copies, events, runner output, and JUnit reports in
`ringfour-sdk/test-results/dashboard/<run-id>/`. Closing the browser does not
cancel a run. One runner is allowed across all tabs. Cancellation interrupts
pytest and escalates to termination after 15 seconds; **Cancel tests is not a
robot e-stop**. A normal console shutdown cancels its runner. Abrupt server
death may leave pytest running; restart marks that saved run interrupted.

The console binds to localhost. `--port` selects another local port.

## Command-line runs

Start any robot stack separately and select its running agent:

```bash
export RINGFOUR_AGENT_TCP=ws://robot.local:10101
# Or: export RINGFOUR_AGENT_UNIX=/run/ringfour.sock
export RINGFOUR_PROFILE=/path/to/robot/profiles/my_robot.yaml  # optional
ringfour-sdk/.venv/bin/python -m pytest ringfour-testing/integration/test_*.py -q
```

Supply TCP and Unix addresses only if they refer to the same agent. The full
TCP address is preserved; it is never replaced with localhost. Discovery is
read-only and happens during collection to produce instance-specific names.
A missing/unreachable target is a setup/collection failure, not an all-skipped
passing run. Use `--collect-only` to inspect what would be exercised. Live
checks have a default 180-second wall-clock budget per test; adjust
`RINGFOUR_TEST_TIMEOUT` for slow hardware.

`ringfour-testing/platform_tests/` remains the offline SDK suite. Startup rejection and
profile migration regressions use synthetic fixtures there, without loading a
specific robot’s files. The dashboard's own offline
checks are under `dashboard/tests/` and can be run without an agent:

```bash
ringfour-sdk/.venv/bin/python -m pytest ringfour-testing/integration/dashboard/tests -q
```

## Physical scenarios belong to the robot

A profile specifies available interfaces, not where it is safe to drive in
this room, what object blocks a gripper, or which collision configurations
have been independently validated. Those facts belong in a per-robot
scenario file, supplied with `RINGFOUR_CONFORMANCE` for custom CLI runs.
The simple movement checks have built-in defaults and optional edits in
**Test settings**. The shared suite never invents targets from a robot's name or guesses
that a continuous/prismatic/revolute joint can move by some universal amount.

The versioned scenario schema is shown in
[conformance.example.yaml](conformance.example.yaml). Every recipe binds to
the loaded profile name and model; optional version and description SHA make
that binding stricter. Joint configurations are **name -> value mappings**,
converted to live controller order. Unknown groups, missing/extra joints,
non-finite numbers, stale SHA bindings, and misspelled fields fail validation.
Physical targets are checked against live joint limits before commands. The
runner saves and executes a frozen copy of the reviewed recipe.

```bash
export RINGFOUR_CONFORMANCE=/path/to/robot/conformance.yaml
export RINGFOUR_TEST_MOTION=1
ringfour-sdk/.venv/bin/python -m pytest ringfour-testing/integration/test_*.py -q
```

- `groups.<name>.waypoints` exercises joint trajectory convergence, progress,
  and cancellation for any joint count. Only that group is restored afterwards.
- `allow_sessions` enables controller switching/locking/hold checks without
  supplying motion waypoints. Switching can affect balancing robots, so it
  is an explicit scenario choice.
- `groups.<name>.tool` enables open/close checks against the declared endpoints.
  A `blocked_target` must describe a known obstruction **inside joint limits**;
  the suite never deliberately drives past a mechanical hard stop.
- `teleop` describes commands, watchdog, duration, and measured acceptance
  limits. Optional `watch_stop` verifies bridge-side halting while a separate
  process keeps sending commands and never polls its watch.
- `navigation` provides explicit goals in their actual frame and independently
  known unreachable goals. There is no arbitrary 2D offset, map resolution,
  footprint, or assumption that the robot is a differential-drive base.
- `mapping.save_localize` enables a map save/localize journey and removes only
  artifacts created by the test. It requires mapping mode as the initial state.
- Clear/colliding configurations and performance budgets are optional read-only
  kinematics scenarios. Missing recordings are untested, not guessed.
- `description_change` exercises model rebuild events using explicit argv
  commands for the selected stack, with a mandatory restore command.
- `debug: true` explicitly enables the agent's test-only fault-injection APIs.

These tests exercise the APIs the SDK actually has. Its existing teleop command
axes are `linear`, `angular`, and `lateral`; its current navigation implementation
is backed by Nav2. This refactor does not add takeoff/landing, altitude control,
gait controllers, or balance recovery. Aerial/legged-specific interfaces need
corresponding backend contracts and tests. An aircraft's cameras, IMU, exposed
joint groups, and other existing interfaces can still be tested now; unimplemented
flight coverage stays visibly incomplete.

## Coverage and failures

The console distinguishes:

| Result | Meaning |
|---|---|
| Passed | The advertised interface was exercised and met its assertion |
| Failed / error | A contract assertion, advertised interface, setup, or teardown failed |
| Not applicable | This agent does not expose the interface/instance |
| Untested | A recipe, recording, independent YAML, dependency, or motion opt-in is missing |

A run with untested checks is **incomplete**, even when pytest exits 0. Missing
capabilities alone do not make a run incomplete. Tests compare actual motion
and readings, not just reported success states. FK comparisons use each group’s mounting frame, so articulated ancestors and
floating robot roots do not get mistaken for that group’s motion. The current SDK freezes independently controlled joints at measured positions
for FK/IK. Tests reuse one snapshot across numerical operations. An older agent
without external-joint snapshots is explicitly untested and needs updating;
separate per-group descriptions remain a separate backend limitation.

Physical limit and FK tolerances
can be explicitly specified in scenarios; the suite does not loosen them to
hide an incompatible or misconfigured robot.

When a live assertion fails, check the running robot's controllers, sensors,
profile, and description before changing the SDK. A passing conformance run is
an assertion about the interfaces tested under the chosen scenarios; it is not
a certification of every behavior that robot might perform.

## Reading and selecting checks

The console opens on **Tests**. The sidebar has icon tabs for Movement,
Manipulation, Cameras, Sensors & status, and the remaining categories. Select
a tab to show its checks and run that category, or select **All tests** for the
full robot and offline SDK suite. Shared checks run once. Category counts
describe test definitions; execution expands each check over the applicable
cameras, groups, or sensors on the loaded robot. On narrow screens the sidebar
becomes a scrollable row of category tabs.

Results include large counters for passed, failed, not applicable, and untested
checks; click a counter to filter results. While a run is active, a temporary
panel shows the current check, its instance, explanation, robot effects, and
run progress. The panel disappears when the run finishes.

The **Test guide** is available before running anything. Expand a check to read
what it asserts, why it matters, its effects on the robot, and what to inspect
if it fails. Relevant profile and scenario fields appear beside the explanation.
The same description is attached to each result, with its named robot instance.
Use **Run this check** to run an individual definition across applicable instances.
Raw test IDs and assertion output are under Technical details.

**Robot profile** contains connection discovery and the YAML editor. Choose an
original profile file, edit a draft, validate the agent's schema, and review the
diff before saving. Saving preserves a backup and rejects a stale file version.
It does not reload the running agent: reload that agent separately, inspect it,
and rerun the affected checks. The live handshake snapshot is read-only.
Some failures concern the robot model, calibration, backend, or SDK rather than
a profile field; each description identifies the relevant source.

**Previous runs** opens saved results. New runs retain their original test guide
alongside the logs and XML report so later source edits do not rewrite their
explanations.

Operator descriptions live in every selectable test's docstring. When adding a
check, include its title and the structured fields `Categories`, `What it checks`,
`Why it matters`, `If it fails`, `Profile fields`, `Scenario fields`, and
`Robot effects`. Use `None.` for an empty field list. The offline dashboard checks
validate guide completeness and category selection without importing test code.

## Movement defaults and optional edits

Movement checks are ready to run when the agent advertises mobility and
teleop. The straight-drive default follows the previous base101 check:
0.3 m/s for two seconds, at least 0.2 m of travel, a 0.5-second watchdog,
and a stopped-speed threshold of 0.05 m/s.

The built-in profiles include:

- **Straight drive:** movement, stopping, watchdog silence, and killed-client checks.
- **Figure eight:** two circles in opposite directions, checking loop closure,
  measured turns, total path length, and stopping. Defaults use 0.3 m/s,
  0.6 rad/s, and a 0.5 m circle radius.
- **Braking time:** drive at 0.3 m/s for two seconds, then measure stopping time
  and distance. The default time limit is 2.5 seconds.
- **Odometry spin:** one revolution at 0.6 rad/s, checking accumulated yaw and
  maximum position drift. It checks odometry consistency, not external calibration.

Durations use odometry's ROS timestamps, with a wall-clock guard for stalled
feedback. Circle and spin presets finish at the measured rotation rather than
assuming commanded speed times wall time produces that angle. A stalled,
reversed, or excessively slow response still fails. Rates and both clocks are
attached to the results. Defaults respect command limits when the matching
independent profile is available; explicitly saved settings remain operator choices.

Select **Movement**, enable commands, and press play. To change defaults, open
**Test settings**, expand a card's **Edit settings**, change its values, and
**Save changes**. Settings are saved for the connected robot's name/model and
apply automatically to subsequent runs. **Restore defaults** resets them.
There is no file selection, draft application, or separate validation step.
The server validates edits when saving and rejects stale updates.

Each run retains its exact settings internally alongside its results. CLI
runs use built-in defaults unless `RINGFOUR_CONFORMANCE` explicitly supplies
custom scenarios. Custom group targets, navigation goals, and other
robot/environment-specific recipes continue to use that optional format.

The point-in-time [suite audit](AUDIT.md) separates current harness limitations,
agent version mismatches, missing scenarios, and measured robot results.

Live FK/TF comparisons use a recorded TF timestamp and the measured joint
snapshot from that same publication. Comparing the latest 100 Hz joint sample
against interpolated 20 Hz TF can otherwise produce a false model error.
Missing old-agent support or unavailable synchronized feedback is **untested**,
with a reason; missing/non-finite required joint positions is a failure. The
agent retains a bounded history and never invents absent position or velocity.

Navigation is **not applicable** when the independent profile declares no
navigation action. A robot's mobility interface alone does not promise Nav2.
Reachable/unreachable goals remain explicit environment fixtures; reset or
relocalization can require reviewing their coordinates. No morphology supplies
safe arm paths, camera resolutions, gripper endpoints, or flight procedures.

The map journey retains the loaded graph and existing places during cleanup,
using `robot.slam.start(preserve_graph=True)`. Older agents are checked before
mutation and report this journey untested if they cannot preserve the graph.
Group and tool cleanup verifies measured restoration before releasing control.
A group recipe can set `restore_timeout` for its reviewed restoration deadline.

An optional debug tool-stall fixture requires both `stall_command` and
`stall_restore_command`. The latter stops the remote publisher before the tool
is restored; terminating only a Docker CLI is insufficient. Results identify
this as an inferred actuator stall, **not a physical grasp or force test**.
Generic tests never install this fixture automatically.

Recorded fixtures can bind `profile.description_sha`; the dashboard retains
that binding and rejects stale fixtures after a model change. Pure movement
presets remain independent of robot names and joint counts. A successful run
covers its selected interfaces and supplied fixtures, not unspecified behaviors
such as takeoff, walking balance, or manipulation of a real object.

Trajectory arrival errors identify the failing joint's URDF units, including
prismatic joints in mixed groups. Missing or non-finite commanded-joint feedback
cannot be confirmed as arrival. Physical trajectory checks require both action
feedback and SDK state reads to show motion; reaching a final target equal to
the initial position is insufficient.


## Compatibility and optional suites

The dashboard defaults to **Robot compatibility**, with 54 live check definitions.
The suite verifies profile bindings, actual reads and measured commands, stopping,
ownership, cancellation and supported kinematics. Commands still require the
existing opt-in and reviewed robot-specific fixtures. Missing fixtures for these
operations remain untested; selecting a read-only run does not certify motion.

The suite sidebar shows an icon and description for each suite. It also offers Robot qualification (5 checks, including figure
eight, braking budgets, spin drift, tool stall and collision performance),
Diagnostics & fault injection (7), Live software regressions (22), SDK offline,
and All suites. Checks merged into acceptance remain individually accessible in
Live software regressions for focused diagnosis. No regression coverage was deleted.

Live command paths now share their progress assertions with the arrival tests.
Joint-state/model/kinematics limits, camera payload transport and retrieval,
SLAM mode/backend/grid, and group/tool metadata are checked together. Distinct
watchdog, disconnected-client, cancellation and watch-stop paths remain separate.
Custom actions without reviewed execution fixtures are explicitly untested even
when discovery succeeds. Joint watches must receive finite measured feedback.

Test settings exposes the prerequisites for the selected robot: named joint
targets and collision examples, group sessions, tool fixtures, watch conditions,
navigation goals, map workflows, model change/restore commands, snapshots,
custom action arguments and measured feedback, and transport/runtime settings.
An untested result's **Configure requirements** button opens the relevant section.
Reading current joint positions only captures a draft; edit the targets before
using them for a movement check. Joint targets use the robot's declared names
and native units and are checked against live limits before saving. Settings
preserve other configured fixtures and bind group fixtures to the description SHA.
Snapshot generation and missing backend features remain explicit prerequisites;
the settings page explains them rather than pretending a checkbox supplies them.

### Automatic joint group recipes

For reusable collision geometry and independently labelled numerical fixtures,
see [Robot preparation](ROBOT_PREPARATION.md). The **Prepare robot** UI in Test
settings resolves ROS meshes in a Docker environment, records an SDK snapshot,
and optionally samples a separate MuJoCo model. Candidates remain unreviewed
until selected and accepted into draft settings. This shares the existing CLI
onboarding mesh fitter; reach-map generation still uses the CLI.

Each group offers Read-only, Hold position, Small movements, and Named poses.
Read-only needs no group recipe. Hold and Small movements require explicit
group eligibility confirmation plus the run's existing Enable commands switch.
Eligibility means the robot is supported/balanced and controller takeover and
the requested movements are suitable in the current environment. The harness
does not infer that from morphology or group names.

Small movements generate named targets from fresh measured joint positions at
run time. The default budgets are 0.05 radians for rotational joints and 0.005
metres for prismatic joints, with a 0.001 native-unit error tolerance. Each
joint is exercised individually, returning to the baseline between probes.
The generator selects an available direction within declared limits and rejects
unknown joint types, invalid feedback, out-of-limit baselines and movements
too small to distinguish from tolerance. **Preview generated movements** reads
state and displays targets without commanding the robot. A preview is illustrative;
the actual recipe is regenerated from the state at execution time.

Numerical excursions within the existing 0.00001 native-unit joint-limit
allowance are clamped to legal command bounds and recorded alongside the raw
measured baseline. Larger excursions require an explicit calibrated allowance
or corrected state. Arrival and hold feedback are checked while the trajectory
session still owns the group; releasing it can hand control to another controller.

Generated recipes, baselines, policies and model SHA are recorded in test result
properties. The arrival check requires observed movement for every probed joint;
the fixture restores and verifies the baseline after execution. Joint-limit
validation does not certify collision clearance or humanoid balance. Intentional
collision examples, obstruction fixtures and meaningful large poses remain
optional qualification inputs. Existing manual named poses remain supported.
A separate save/resolve/delete places check verifies writes without navigation.
Controller restoration is labelled indirect: matching session transitions and
audit events do not independently measure every ROS controller's state.

Robot result counters exclude offline SDK tests. Mixed historical runs show
separate robot and offline counts; repeated identical unsupported reasons for an
instance are grouped in the UI, with original check IDs in the details. Raw JUnit
and run artifacts retain all cases. N/A cards count unsupported groups, while
completion and per-scope artifact counts retain individual check instances.

CLI runs of this directory also default to compatibility:

```bash
RINGFOUR_AGENT_TCP=ws://127.0.0.1:10102 RINGFOUR_PROFILE=/path/to/effective-profile.yaml \
  ringfour-sdk/.venv/bin/python -m pytest ringfour-testing/integration --ignore=ringfour-testing/integration/dashboard
```

Use `--robot-suite=qualification`, `diagnostics`, `regression`, or `all` for
extended runs. Explicitly named `file.py::test_name` checks remain selectable in
any suite. Offline SDK and dashboard self-tests are unaffected by this filter.


## Movement precision and navigation defaults

Robot profile has a **Movement precision tolerance** slider (1–5×), saved per
robot. It scales only figure-eight closure/rotation error and spin drift/rotation
error. It preserves command speeds, turn count, minimum observed movement,
watchdogs, stopping thresholds and braking budgets. The multiplier and effective
limits are frozen with each run and recorded in result properties. A 1.5× setting
makes the built-in spin drift allowance 0.15 m. Editing base preset values and
changing the multiplier do not compound the stored base values.

Debug checks are enabled by default, matching the agent's existing debug API.
Synthetic task and payload tests do not require motion opt-in. Emergency-stop
injection still requires commands enabled and restores the original stop state.
The Test settings checkbox can explicitly disable debug checks.

When the selected effective profile declares a Nav2 action, navigation defaults
to map-based goals. Before each navigation test, the harness selects two nearby
cells reachable through connected known-free occupancy cells, with an editable
clearance region along the grid route. Occupied, partially occupied, unknown and
out-of-map cells are excluded; rotated map origins are supported. Nav2 supplies
actual planning and obstacle avoidance. Preview goals in Test settings without
moving the robot. Clearance must suit the robot footprint; the default is 1 m,
with a saved 0.75 m setting for the inspected G1 whose Nav2 radius is 0.45 m.

No goals are invented when the current pose lacks clearance or mapped space.
Manual reviewed goals remain supported. Known-unreachable goals and map
save/localization qualification still require their separate fixtures/opt-in.
The map strategy applies to the declared planar Nav2 interface, not undeclared
flight or gait control. Main descriptions now use 16 px text, test names 17 px,
and supporting text at least 14 px.

### Strict stopping in the hardened agent

Source agents on `review_and_hardening` require a reviewed shared
`safety.stop_verification` policy (angular rad/s and linear m/s limits, dwell,
timeout), or appropriate per-group overrides, plus fresh finite measured
feedback. Missing contracts reject before motion; the test environment must
report this setup requirement rather than loosen thresholds or fabricate data.
See [the implemented contract](../ringfour-sdk/docs/hardening-progress.md).
`Welcome.features.strict_measured_stops` distinguishes the production strict
path from internal legacy synthetic fixtures. These thresholds need integration
review; simulator feedback does not commission physical stopping.


## Execution hardening across robots

`test_execution_hardening.py` is part of the shared **Diagnostics & fault
injection** suite. Its tests use discovered groups and reviewed per-group
waypoints/probes, never a robot-name whitelist. It covers observer/wait lifetime,
persistent joint guards, healthy renewals during slow reads, live-socket lease
expiry, killed group owners, feedback-loss quarantine with explicit local
recovery, and network recovery denial. It appears automatically in the console
catalog and category selection.

Lease-expiry and killed-client checks require the corresponding boolean under
`groups.<name>.hardening`. Feedback loss additionally requires `debug: true` and
reviewed disable/restore/recovery argv commands. The shared recovery helper takes
an explicit local URI and resource; where it executes belongs to the robot
fixture. Restoration is attempted even if a fault command changes state then
fails. Quarantine is never cleared merely to finish a test or restoration.
Absent policy, authority or fixture is reported as untested, not passed.

Milestone C adds three Diagnostics checks: lost trajectory reply reconciliation
(with dropped terminal notifications), durable terminal evidence across an agent
restart, and measured body stopping after killing a drive client. The lost-reply
check uses the existing reviewed group motion fixture. Restart and mobility faults
use explicit reviewed `lifecycle` and `mobility_fault` scenario sections; commands
are argv lists, never shell strings. The restart helper runs under the deployment's
authenticated local identity; unverified TCP cannot inherit another connection's
records. Input restoration runs even after a failed mobility test. The suite never
clears quarantine just to make restoration or subsequent tests pass.

The SDK suite also catalogs `test_submission_reconciliation_contract.py` for
repeatable fake-actuator reply loss, concurrent duplicate/conflict, expiry/eviction,
crash unknown outcomes, principal ownership, clock rollback, cancelled observers,
storage failure and shutdown-drain failure. Every case has operator guidance and
is selectable in the dashboard; no prior run needs to be imported into history.
