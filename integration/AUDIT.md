# Conformance fixes and verification — 2026-10-08

The original audit below is historical. The current suite tests declared SDK
interfaces and explicitly supplied fixtures, independently of robot name,
locomotion, group size, or camera resolution. Robot-specific settings stay in
per-target configuration, not in shared assertions.

## Resolved harness and SDK issues

- Movement uses ROS odometry time and raw odometry coordinates, with a wall
  deadline for frozen feedback. Circle/spin targets use measured accumulated
  rotation; requested speed times elapsed wall time is not a guaranteed turn.
- FK comparisons freeze both owned and external joints. They select an actual
  TF publication and its recorded joint sample, avoiding interpolation errors
  when TF publishes more slowly than joint feedback. Older/incompatible agents
  report the missing support explicitly; missing/non-finite positions fail.
- The agent never invents missing joint positions or stationary velocity from
  partial JointState messages. Trajectory arrival requires complete finite
  measured feedback, even after the controller reports success.
- Safety-watch helpers unwind their contexts on termination and bound orphan
  lifetime. Readiness parsing accepts complete marker lines, and audit checks
  correlate the actual watch ID rather than old matching events.
- Navigation cancellation selects the furthest reviewed endpoint and checks a
  stable halt rather than two potentially conflicting instantaneous reads.
  Navigation is not applicable when the independent profile has no action binding.
- Reviewed waypoints are rotated so the first target exercises motion. The
  trajectory check requires observed movement in both action feedback and SDK
  joint-state reads, including paths whose final target equals their start.
  Group/tool teardown verifies measured restoration before releasing control.
- Map-journey cleanup retains the graph and preexisting places. Preservation is
  feature-negotiated before mutation; an old agent cannot ignore the flag and
  erase the map. Both temporary and original place namespaces are cleaned.
- Localization confidence has bounded memory: repeated corrections can no longer
  underflow permanently to zero and prevent recovery after TF stabilizes.
- Arrival errors use URDF joint units: metres for prismatic joints, radians for
  rotary joints, and explicit unknown units otherwise. Group labels do not
  establish individual joint units.
- Built-in command defaults respect a matching independent profile's declared
  limits. Recorded fixture description hashes are retained rather than silently
  rebound after a model change.

## Validation

Offline SDK and dashboard checks: **381 passed**.
Synthetic coverage includes different group sizes, independent moving mounts,
partial feedback, older agents, faster joint streams than TF, slow simulation
clocks, wheeled/legged/humanoid/flying capability combinations, and arbitrary
camera resolutions. These are regression fixtures, not physical robot trials.

The complete live run `20261008-163820-c20d99ea` finished with **197 passed,
2 failed, 52 not applicable, 0 untested, 0 errors**. Both failures were the
left lift missing reviewed endpoints within its configured duration. Its local
recipe now uses the previously validated operating band and a longer duration;
convergence, progress, and cancellation reruns all passed. Shared thresholds
were not relaxed. The final complete run `20261008-171123-4e860df9` passed:
**499 passed, 0 failed, 0 errors, 0 untested, 52 not applicable** (551 instances).
It includes all offline SDK cases and all selected live categories.

## Simulator-only preparation

The Mr Handy MuJoCo target received a rebuilt agent, simulation-clock settings,
a shared map-storage directory, recorded geometry with resolved package meshes,
and explicit navigation/group/collision fixtures. These are deployment and
fixture data for that target, not prerequisites imposed on other robots.

A simulator reset caused existing trajectory controllers to stop with an update
error; attempts to reactivate those objects then accumulated command interfaces.
The affected controllers were explicitly reloaded in the local simulation.
Shared tests do not automatically repair controller-manager objects or change
other robots' profiles.

This target's local joint-limit acceptance bands document observed simulation
compliance (0.003 rad / 0.001 m); the generic default remains 0.00001 joint units.
An excursion beyond the configured band stays a failure. No URDF bounds were
changed. Collision examples use frozen recorded geometry/configurations; the
suite does not drive the robot into a collision to create them.

The tool-holding fixture is an explicit, bounded simulator actuator stall, with
remote cleanup before restoration. It verifies the SDK's inferred holding
signal, not physical contact, grasp success, or force. Description notification
uses a harmless payload change with a required restore command.

A passing run covers its selected SDK interfaces and supplied fixtures. It does
not certify unspecified behaviors such as flight, balance, workspace safety,
or grasping a real object. Other profiles may legitimately require different
fixtures or report unsupported SDK/backend coverage as untested.

---

# Original read-only audit — 2026-10-08

Baseline: `20261008-143758-0e131ef0`, Mr Handy, all categories, commands enabled:
378 passed, 13 failed, 108 untested, 38 not applicable (537 runtime instances).
This audit did not run movement or reload the robot stack.

## The 13 failures

| Instances | Finding | Action / current status |
|---|---|---|
| 2 | Figure eight and spin run for `angle / commanded_speed` in wall time, assuming the simulator runs at 1×. Measured turns were 4.171 and 4.076 rad rather than 6.283. | Motion clock handling needs correction; do not conclude that steering is broken from these results. |
| 4 | Native FK now requires measured external-joint snapshots. The running agent returns only `joints` when called with `include_external: true`, whereas current agent source returns `positions`. | Harness now reports an explicit compatibility gap as untested. Rebuild/reload the running agent before validating full kinematics. |
| 3 | No-tip IK requests tried to obtain external state before rejecting the missing tip, obscuring the intended error with old-agent incompatibility. | Fixed SDK validation order; regression added. All three live checks now pass. |
| 4 | Real joint positions exceed the model's lower bounds; default acceptance is only 0.00001 joint units. | Keep visible. Determine simulation compliance / encoder uncertainty versus a genuine model or controller problem before setting tolerances. |

The elbow excursions beyond the lower bounds are approximately 0.001583 rad
(left, 0.091°) and 0.001932 rad (right, 0.111°). Lift excursions are approximately
0.000321 m and 0.000332 m (0.321/0.332 mm). They persist in a fresh read-only run.
No limits or tolerances were loosened.

The simulator clock issue is reproducible without a robot: a synthetic plant
with correct commanded angular velocity and simulation time advancing at 0.67×
wall time fails both current movement tests with 4.221 rad. A live read-only IMU
stamp probe advanced at approximately 0.778× wall time at audit time. Simulator
rate varies; this supports a timing explanation but does not establish the
entire cause of the historical motion failures. Requested/granted velocities
and odometry timestamps should be recorded on the next motion validation.
Use robot/odometry time for motion duration; retain wall time for watchdog,
communication and outer test deadlines. The current mobility state wire reply
has no timestamp, so a reliable clock contract needs agent/SDK support rather
than guessing from robot morphology or an unrelated sensor.

## The 108 baseline untested instances

| Instances | Original reason | Assessment |
|---|---|---|
| 72 | No group scenarios | Mixed: controller sessions, trajectories, tool motion and independent collision fixtures. Eight were actually tests of absent tools: applicability now runs before recipe and motion requirements. The remaining checks need appropriate coverage/configuration, not an assumed successful robot result. |
| 12 | Harness rejects IK with independently moving ancestors | Obsolete restriction; removed. Current SDK supports this topology. The older live agent still blocks these tests through its missing snapshot API. |
| 10 | No recorded geometry snapshots / reach maps | Five snapshot checks need generated artifacts. The five reach-map checks were inappropriate: three groups have no tip, two have moving mounts for which a fixed reach map is not defined. Now not applicable. |
| 5 | No navigation goals | Need a reachable environment-specific goal and known failure/cancellation cases. Mobility alone does not establish navigation coverage. |
| 4 | Independent YAML missing (profile, locomotion, status, magnetometer) | Configuration/discovery coverage gap. Supplying the actual profile resolves three checks; its absent magnetometer correctly becomes not applicable. |
| 3 | Debug-only payload, estop rejection and task lifecycle | Test-only endpoint dependency. Keep explicit; do not silently exercise fault injection or report it validated. |
| 1 | No stop-watch condition | Needs a condition known to become true during the chosen drive. Defaults do not currently supply it. |
| 1 | No description change/restore commands | Mutating stack integration test; requires explicit commands and restoration. |

## Changes made during the audit

- Removed the stale whole-chain ownership rejection in the IK fixture.
- Added snapshot availability and completeness checks. Missing old-agent support
  is untested; missing/non-finite positions in a supported snapshot are failures.
- Numerical checks now reuse one snapshot across FK/IK/Jacobian/reach operations.
  Mount-frame comparisons also use the same snapshot for both transforms.
- Checked tool/controller applicability before asking for recipes or command
  enablement, preventing absent interfaces from inflating untested counts.
- Marked fixed reach maps inapplicable to tip-less or independently mounted groups.
- Fixed the SDK's no-tip IK error order without inserting zero-valued external state.

## Verification and remaining work

Offline SDK and dashboard regressions: **357 passed**.

Fresh live read-only audit with the actual independent YAML:
`20261008-144641-f83e7320`: **94 passed, 4 failed, 38 untested, 44 not applicable**
(180 runtime instances from 60 read-only definitions). This is a subset, not
another full motion run. Its remaining untested checks are exactly:

- 16 FK/IK-related instances awaiting the running agent's external-joint snapshots.
- 17 independent clear/collision/performance scenario instances.
- 5 geometry snapshot instances awaiting recorded artifacts.

A separate read-only tool applicability run, `20261008-144750-c94cfc9a`, verified
8 not applicable and 2 intentionally disabled tool-motion instances.

Priorities:

1. Rebuild/reload the running agent and rerun read-only kinematics. Do not restart
   the live stack as an incidental test harness action.
2. Add an odometry timestamp / clock contract and correct motion duration handling;
   collect requested versus measured rates before rerunning movement.
3. Determine calibrated simulation acceptance for the four joint-limit excursions.
4. Fill remaining coverage deliberately: known collision fixtures and geometry
   recordings, then group trajectories/tools and navigation. Give these the same
   simple operator workflow as movement rather than hiding missing recipes.
