> Migrated contract/history: dated deployment evidence belongs to the original checkout. Local run archives were not copied. See [current migration verification](../VERIFICATION.md); this move does not establish new robot acceptance.

# Implemented hardening behavior

This page describes the incremental implementation on `review_and_hardening`.
The repository's `HARDENING.md` is a proposed v1 specification, not a description
of guarantees already shipped. See `HARDENING_LOG.md` for acceptance evidence.

Milestones A and [B](milestone-b.md) are implemented and validated on the Handy
simulator. The final B agent image matches the SDK/agent Python sources in this
checkout. Operations/results, model-bound commands, measurements, persistent
guards, route/tool execution and naming migration have offline and live evidence.
Milestone C now adds distributed lifecycle and retained evidence; see [its contract](milestone-c.md).
D and physical commissioning remain separate. See the B page for exact counts,
fixture dependencies and retained failed attempts.

## Admission and ownership

Every registered RPC needs an admission descriptor declaring effect, permission,
logical resource dependency, ownership policy, inhibit policy, parameter schema
and completion model in the agent's admission inventory. An unknown registration
prevents startup. Navigation,
teleop, group acquisition, trajectories, hold and tool commands pass the central
motion gate. Profile-defined vendor motion additionally checks its declared
effect at dispatch. Vendor effects are also resolved from the profile at central
admission. All wire payloads reject non-finite JSON; schemas reject unexpected
fields, missing required fields and invalid types. Scalar velocity, watchdog,
tool inputs, waypoints and navigation geometry reject booleans and strings.
Navigation goals require an explicit finite framed Pose with a unit quaternion;
resolve named places before submission. Group coverage, positional bounds, feedback and current scope
ownership remain handler checks and are rechecked at dispatch where mutable.

Required permissions are enforced before handler dispatch. `Welcome.features.access`
reports the connection principal, authentication status and granted permissions.
Unix identity comes from Linux kernel peer credentials: the agent's UID receives
local operator/recovery authority; other UIDs receive observation only. TCP is
read-only by default. `--trusted-tcp` explicitly grants operator permissions to
every TCP peer in a trusted simulator/private gateway deployment; it does not
authenticate or encrypt those peers and never grants quarantine recovery.
`PermissionDenied` preserves the denied permission and principal in `.details`.
The SDK sends no authority in RPC parameters.

This changes deployment defaults: network motion/configuration now requires an
explicit trusted deployment policy. Existing installed agents are unaffected
until updated. Model revision binding and persistent guard admission are implemented in B.
Remote authenticated principals/TLS/gateway identity remain release work.

Group sessions have opaque IDs. Commands identify the current session, so a
closed handle cannot command a replacement session. Counted acquisitions ensure
closing or failing to open one group cannot release another group. Controller
lifecycle changes are serialized across worker threads. Acquisition records
attempted load/configure/switch effects before calling the adapter, so a lost
acknowledgement after applying a change still triggers rollback. Navigation also holds
an acquisition for its accepted task; teleop has its own acquisition.

Close/cancel targets the session generation, including across a replacement
scope. Update SDK and agent together: older clients that do not send session IDs
cannot submit group motion to the hardened agent. New clients retain supported
legacy read/notification behavior when connecting to an older agent.

Acquisitions record joint/controller footprints. Teleop and navigation reserve
the same mobility adapter and cannot overlap, even for one owner. Vendor motion retains exclusive robot-wide ownership and requires an explicit
observable resource and stop payload. Strict raw-controller entry requires a
declared role alias; unknown controller semantics reject before effects. Raw
controllers must read back matching group joint claims.
SLAM start/stop/load/localize/relocalize reserve mobility dependencies, preventing
configuration from invalidating accepted navigation. Disjoint joint groups can
retain separate acquisitions under the same whole-robot owner.

The current ROS integration stores one trajectory goal at a time. Concurrent
group trajectories are explicitly rejected with `ResourceBusy`; they do not
silently replace another group's goal. Group cancellation targets only that
group's task records. Concurrent disjoint trajectory execution is not supported.

## Completion and cleanup

Waypoints require matching width, finite numbers, positive finite duration and
positional bounds supplied by the integration. This does not prove interpolated
velocity or acceleration limits.

Trajectory success requires every commanded joint's finite measured position.
Both source age (ROS clock) and reception age (agent monotonic clock) must satisfy
the profile's `groups.<name>.feedback_max_age`, default **0.5 seconds**. These are
separate clock comparisons; paused simulation time does not conceal dead
feedback. Failure to establish freshness raises `CompletionUnverified`.

Arrival tolerances use each joint's native units. `goal_tolerance` is the group
fallback; `goal_tolerances: {joint_name: value}` supplies individual overrides,
especially for groups mixing revolute and prismatic joints. Effective values are
reported in `Welcome.features.group_feedback_policy`.

Session cleanup attempts every undo step, reads back controller states, and
preserves a report. Failed restoration or cancellation acknowledgement raises
`CleanupFailed` and quarantines subsequent motion. Quarantine persists after
ownership release and reconnect. Quarantine conservatively inhibits all robot
motion; recovery clears only its explicitly verified resource. There is no
automatic recovery. With the agent state directory configured (the CLI default),
quarantine is atomically checkpointed under the robot identity and model revision.
An unfinished boot adds a restart quarantine; a changed model adds a separate
restart-model quarantine. Corrupt state or checkpoint ownership conflicts prevent
startup. This checkpoint does not persist operation results.

Production motion requires usable measured-stop contracts. Teleop stop, end,
disconnect, watchdog and shutdown publish zero and verify full body rest before
successful cleanup. Navigation reconciles failed submission/wait paths through
cancellation and measured stopping; cancellation acknowledgement alone is
insufficient. Mobility stop attempts independent cancellation and zero paths,
drains affected work, and retains failures. Failed verification quarantines
motion after ownership release. Concurrent teleop cleanup waits for the same
in-flight verification rather than releasing early.

Tools retain unique acquisitions throughout positioning and cleanup. They need
fresh finite position/velocity feedback and native-unit thresholds, verify target
residual/rest, and hold current measured position on failure or revocation.
Failure to verify that hold quarantines the tool. Contact does not silently count
as target arrival. Tool positioning has an independent finite execution deadline (default 30s);
the profile stopping timeout governs measured-rest verification. Convenience
wait deadlines cancel/join cleanup, independently of target travel.

Operator inhibit and persistent guards latch inhibition before stopping accepted
work, teleop and vendor motion, draining tasks, and restoring controller scopes.
Guard cleanup runs off the monitoring loop. Inhibit reset never resumes previous
work and is refused while quarantine remains. Required stale or unknown guard signals trip inhibition under their age budget.
Named watches are observational; effectful stop watches are retired.

**Controller acknowledgement does not establish physical rest.** Measured-rest
reports explicitly say `physical_stop: uncommissioned`; missing evidence says
`unverified` and raises a typed failure. Existing stop RPCs add evidence to their
results where available; the public v1 `StopReport` migration remains separate.

## Reviewed measured-stop policy and local recovery

The default agent is strict: unconfigured motion rejects before actuation.
Declare one reviewed `safety.stop_verification` policy for the robot; groups may
override it with `groups.<name>.stop_verification`. This avoids repeating policy
for every joint group. Example values below are **illustrative**, not universal
safe limits:

```yaml
safety:
  stop_verification:
    angular_velocity_limit: 0.01  # rad/s; example only, review for the integration
    linear_velocity_limit: 0.001 # m/s; required for body motion
    dwell: 0.2                  # seconds of advancing stationary evidence
    timeout: 2.0                # per-check budget, maximum 15 seconds
```

There are no guessed morphology thresholds. Required joint units come from the
live model; unknown units or missing thresholds cannot verify stopping. Source
and receive timestamps must be fresh and advance for every joint throughout the
dwell. Finite measured velocities and velocities derived from successive measured
positions must both satisfy their native-unit thresholds. Frozen feedback and
paused source clocks cannot finish the dwell. `StopUnverified` retains failure
evidence and quarantines affected group motion.

Configured scope cleanup verifies measured rest before controller restoration
and again afterwards; restoring a controller can apply an old target. Each
check has the configured timeout, so stop verification alone can consume twice
that budget, plus bounded adapter cancellation/restoration service calls. The
60-second client cleanup wait is a separate budget. Configured trajectory
cancellation verifies the criterion before producing terminal cancellation;
failed evidence produces a retained failed `StopUnverified` outcome. Policy and
eligible groups are advertised in `Welcome.features.group_stop_policy` and
`measured_stop_groups`. `body_stop_policy` advertises the shared policy and
`strict_measured_stops` states whether strict admission is active. The internal
`allow_unverified_stops=True` construction input exists for legacy synthetic
fixtures; it has no production CLI switch, advertises strict mode as false, and
never establishes A measured-stop acceptance. Profile YAML cannot enable it.

Body verification requires both m/s and rad/s limits and all six twist axes.
Full 3D position and quaternion changes independently check reported velocity;
a zero-filled twist cannot conceal movement. Equivalent quaternion signs do not
invent rotation. Odometry source and agent receipt clocks must both progress.
Navigation success additionally requires fresh measured pose in the goal frame,
aligned to measured odometry time. Full 3D metre/radian residuals use
`mobility.arrival_position_tolerance` and `arrival_orientation_tolerance` (both
0.1 by default). No goal or free map location is inferred.

`safety.recover({resource: group_name})` (or `tool:<group>`) is an operator RPC
for group/tool quarantines,
not a mission motion command. It requires verified local identity with `recover`
permission, an engaged inhibit, no active tasks/acquisitions, a retained controller
checkpoint and a configured stop policy. It reattempts cancellation, checks
controller states, verifies measured rest, and checks states again. Failed
verification leaves quarantine intact. Success clears only that group, keeps the
inhibit engaged and preserves earlier failed task/cleanup records. Inhibit reset
is separately privileged and rejects while any quarantine remains. Recovery of
vendor quarantine is not supported. Mobility recovery is available to the same
local recovery principal: cancellation and zero dispatch must succeed, fresh
full-body velocity must meet the configured measured-rest policy, and eligibility
is checked again before clearing only mobility. A reconnect does not recover it.

Unclean-boot recovery uses `safety.recover({resource: "restart"})`. It requires an
explicit `safety.restart_recovery_controllers` baseline, idle inhibited execution,
trajectory-only groups without tools, and no opaque vendor motion. Recovery checks
the complete loaded controller state against that baseline, cancels orphaned
trajectory/navigation action goals, measures every declared group and mobility at
rest, then checks controller state and eligibility again. It clears only restart
and keeps inhibition engaged. It does not recover model changes, resume old work,
or establish a previous operation's outcome. Restart recovery is implemented;
XLeRobot live restart recovery passed on November 7: body and all three groups
met the configured progressive dwell. A subsequent Docker stop exited zero and
saved `active_boot: false`; the next boot retained no restart quarantine.

Measured verification reports `physical_stop: uncommissioned`; sensor accuracy,
hardware stopping performance and emergency-stop behavior still require separate
commissioning. This feedback contract is software evidence, not physical safety certification.
Milestones A/B software implementation is complete; evidence is tracked in
`HARDENING_LOG.md`. D and physical commissioning remain separate release gates; C acceptance is tracked in [milestone C](milestone-c.md).

## Scoped observation and cleanup

In sync and async APIs, breaking trajectory/route iteration only stops observing.
Retaining or garbage-collecting an iterator has no cancellation effect. Use an
explicit trajectory/route scope, close the handle, or leave its control session
to cancel unfinished work. Canonical `start_*` operations submit eagerly; legacy `follow`/`navigate`
iterators have been removed.

`group.control()` defaults to trajectory behavior and creates a lazy opener;
discarding it creates neither a coroutine warning nor remote authority. Sync
`.open()` and async `await opener.open()` acquire explicitly. Scope cleanup is
shielded from caller cancellation with a finite 60-second client budget. Body
exceptions retain any additional cleanup failure as `.cleanup_error` and an
exception note. This bounded client wait does not establish physical stopping.

The previous `TrajStatus.worst_residual` property was removed: it conflated
missing evidence with zero error and compared radians with metres. Inspect the
per-joint `.residual` and its native units instead.

## Task queries and errors

`task.get_status()` queries authoritative state through the originating
connection. `Welcome.features.task_status` advertises support. Running tasks and
terminal outcomes share a bounded record store: 1024 records, with terminal
records retained for 300 seconds. Capacity exhaustion rejects new submissions
rather than evicting terminal results before their promised retention expires.

Cancellation and status inspection check the originating client. Cancellation
of a retained terminal task is idempotent and never rewrites its outcome.
Feature-aware clients query after missing updates while waiting. Agents without
the feature keep the existing notification behavior and reject explicit queries
clearly. Reconnect/principal reconciliation remains unimplemented.

Connection loss preserves the last known execution state and raises
`ConnectionLost`; it does not fabricate completion or cancellation. Intermediate
buffered notifications are bounded and cannot rewrite a terminal result.

All reconstructed SDK errors retain the wire data in `.details` and the original
type in `.wire_type`, including unfamiliar integration errors. Arrival residuals
and cleanup reports are no longer discarded during error reconstruction.

## Verification commands

From the repository root, with its existing environment:

```sh
ringfour-sdk/.venv/bin/python -m pytest ringfour-testing/platform_tests -m 'not integration and not hardware' -q
ringfour-sdk/.venv/bin/python -m pytest ringfour-testing/platform_tests -m contract -q
ringfour-sdk/.venv/bin/python -m pytest ringfour-testing/platform_tests -m protocol -q
PYTHONPATH=ringfour-agent/src/ringfour_agent ringfour-sdk/.venv/bin/python ringfour-sdk/scripts/gen_protocol.py
PYTHONPATH=ringfour-agent/src/ringfour_agent ringfour-sdk/.venv/bin/python ringfour-sdk/scripts/gen_types_reference.py
```

These offline tests do not contact a robot. Protocol cases use isolated temporary
Unix/TCP sockets and production clients/dispatchers with a recording physical
adapter. Live tests need explicit target and motion configuration. Hardware is
never automatically discovered or selected.

## Vendor actions

Both sync and async action calls return `DispatchReceipt(action, dispatched,
completion="unverified")`. Dispatch means the integration accepted publication;
it does not verify movement, tool contact or completion. Topic and payload
receipt fields are retained in integration audit diagnostics rather than the
mission result. A client refuses before dispatch when an older agent does not
advertise `vendor_dispatch_receipts`.

Action parameters are validated and rate limited before acquisition. Failed
publication stops uncertain motion before releasing its acquisition. Successful
motion actions retain ownership until release/disconnect/shutdown. Strict motion
needs `motion_resource: mobility` or `group:<declared group>` and an explicit
`stop_arguments` dictionary matching the action's parameters. These stop payloads
bypass normal invocation rate limits/inhibition; they do not grant new motion.
The declared resource's measured-stop contract verifies the result. Unknown
resources, missing payloads or unusable feedback reject before publication.
A vendor completion receipt remains unverified; stopping evidence does not prove
that the original vendor task completed.


Latest continuation evidence is recorded in `HARDENING_LOG.md` and the separate
`test-results/hardening-20261013-a4` artifacts. Live head validation uses a
temporary profile with an explicit measured-stop policy, not a modified canonical
Handy profile. Four simulator joint-limit failures and an unavailable SLAM map in
the empty world remain visible. The harness treats missing connection permissions
as setup requirements; it never suggests profile tuning repairs denied authority.

The final A run passed **636 offline tests**, all **five reviewed Handy head
checks**, and stationary-body teleop stop/end. Final source hashes matched the
checkout. Live compatibility still exposes four measured joint-limit violations
and the empty-world SLAM-map limitation; these were not hidden. The installed
agent was restored after source testing and is not automatically upgraded.


## November continuation: network payloads and finite connection leases

NET is implemented as lossless raw numeric arrays in tagged binary WebSocket
frames, not JSON/base64 and not a replacement command protocol. TCP clients
negotiate NET after the handshake; Unix clients default to SHM. Select explicitly
with `Robot(uri, transport=Transport.NET)` (`Transport` is in `ringfour.enums`).
Legacy agents retain their existing default when no transport is explicitly
requested. Explicit unsupported selection fails before any sensing request.

NET covers RGB, depth, synchronized RGB-D, camera clouds, lidar and occupancy
maps, including streams. Payload IDs belong to one connection. The client checks
header/descriptor agreement, numeric dtype, dimensions and exact byte count;
returned arrays are read-only. Pending buffers are bounded at 64 payloads and
192 MiB, evicting oldest first. Evicted descriptors raise FrameExpired. Each
binary message is capped at 64 MiB including its tag. There is no image codec or lossy conversion; the existing WebSocket handshake
can negotiate lossless deflate compression. Remote authentication/TLS and hard
real-time throughput guarantees are not established by this change.

An optional explicit `safety.lease` policy declares positive finite `lifetime`
and `renew_interval` seconds; renewal must be more frequent than half the lifetime.
For example, an integrator may review 10s lifetime / 1s renewal for a simulator;
these are not physical timing guarantees. Lease IDs bind the principal, connection,
robot handshake and random boot epoch. Expiry cannot be renewed or reacquired by
the revoked connection. Revocation precedes task cancellation and shared bounded
session cleanup. The SDK renews authority, never drive intent; teleop watchdog
freshness remains independent. A failed renewal closes the client connection.
Unconfigured profiles retain prior disconnect/watchdog behavior and do not claim
finite lease coverage. Legacy clients cannot keep a configured lease alive.

Submission reconciliation and durable operation results are now implemented;
see [milestone C](milestone-c.md) for command validity, principal binding, retention,
crash unknown outcomes, explicit retries and measured checkpoint recovery.
Authenticated encrypted remote deployment remains a separate v1 release requirement.
The Docker entrypoint executes the agent directly so SIGTERM can reach its graceful
shutdown handler; clean versus unclean restart acceptance must be tested against
the deployed image, not inferred from offline checkpoint tests.
Physical commissioning and second-configuration motion conformance are not implied.


Cancellation arriving during a trajectory's final stop verification joins the
verification worker before publishing a terminal state. If that worker fails,
the operation remains failed with the original stopping error; a cancelled wait
cannot discard its outcome. Verified cancellation records the worker's stopping
evidence. This covers the race between ordinary controller completion and an
explicit cancellation while feedback is lost.


November 7 post-fix acceptance: the Linux SDK suite passed 518 tests with one
optional kinematics skip; 61 portable harness/catalog checks passed. All seven
selected shared execution-hardening diagnostics passed in one XLeRobot head batch,
including feedback-loss quarantine/recovery and retention of the failed operation.
The earlier killed-base-client stop deadline failure remains separate and unresolved.
This was the pre-reconciliation increment. Current C acceptance and retained failed attempts are indexed in `HARDENING_LOG.md`.


## C completion — November 7

[C software and XLeRobot simulator acceptance](milestone-c.md) are complete:
542 SDK tests passed (one optional skip), 172 dashboard/harness checks passed,
and all ten selected shared live diagnostics passed. Final deployed Python sources
match the checkout. Reconciliation, durable terminal queries, principal checks,
finite leases and explicit measured recovery are implemented. D, authenticated
encrypted remote release deployment and physical commissioning remain separate.


## Ringfour migration — November 13

SDK and agent are now separate sibling repositories; shared tests and the dashboard live in `ringfour-testing`. Milestone D is **Validate and document the platform** and follows the split/rename. Two-robot integration qualification is a separate suite, not a platform release gate. See [migration notes](../MIGRATION.md) and the [current plan](../HARDENING.md). A–C contracts are preserved; D remains incomplete.
