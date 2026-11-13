# RoboCore v1: a coherent API and enforced execution contract

Implementation evidence is tracked in [HARDENING_LOG.md](HARDENING_LOG.md).
Current implemented behavior is described in
[hardening-progress.md](docs/hardening-progress.md).
The proposed requirements below remain release gates until explicitly verified.

**Status:** Proposed pre-release specification, updated 11 October 2026 with infrastructure-boundary and public-surface pruning requirements.  
**Audience:** SDK and agent implementers, robot integrators, documentation authors.  
**Basis:** Review of the local RoboCore SDK and agent. All APIs introduced below are proposed, not claims about currently shipped behavior.  
**Compatibility decision:** Breaking changes are permitted before first public release. Migrate examples, course material and tests together; do not carry competing public APIs merely to preserve unreleased conventions.

## 1. Product objective

A programmer should be able to read a short RoboCore program and understand:

1. Which robot resource it affects.
2. When it starts and finishes.
3. What evidence supports success.
4. Who may control or stop it.
5. What happens when execution, cleanup or communication fails.

Preserve semantic robot APIs, framed geometry, robot-independent clients and composable mission code. Make ordinary code short by moving shared policy into the agent and robot profile. Do not make short code depend on unstated cleanup, ownership or timing assumptions.

“MUST” identifies a v1 release requirement. “SHOULD” identifies a preferred implementation with a documented alternative. This specification defines software contracts, not a physical safety certification.

### 1.1 Persona and abstraction boundary

The default user is a technically capable domain expert: an engineer, student or programmer learning robotics. Preserve the current direct, Arduino-like library shape. Technical robotics concepts are welcome; knowledge of ROS infrastructure is not a prerequisite.

IK/FK, joint configurations, trajectories, frames, twist, wrench, SLAM, odometry, Jacobians and manipulability MUST NOT be removed merely for being technical. Calculating, observing and commanding remain visibly distinct. Do not add a separate simplified mission language or workshop-specific facade.

Existing named watch helpers, enums and useful singular accessors should be preserved where possible. New API spellings below are proposals; equivalent existing spellings may satisfy the contract. Correctness requirements are mandatory, whereas a new noun or rename requires a concrete ergonomic benefit. LLM-generated code is an important consumer: canonical examples and actual behavior must agree.

Strings are appropriate for text and explicit external identifiers. Built-in choices use enums; execution uses validated values or scoped references rather than implicit magic-name interpretation. Runtime identity remains necessary, but names must be resolved/validated at an explicit boundary. No invented button or universal pick API is introduced.

## 2. The public vocabulary

Use six concepts consistently:

| Concept | Meaning | Examples |
|---|---|---|
| Robot | Connection and discovered robot identity | `Robot(...)` |
| Resource | A named robot facility | arm, base, camera, tool |
| Observation | Immutable measured value with provenance and freshness | pose, joint state, image |
| Control session | Agent-granted authority to command a resource for a bounded lifetime | `arm.control(...)`, `robot.teleop(...)` |
| Operation | Accepted finite work with an ID, progress and terminal outcome | navigation, trajectory, tool positioning |
| Guard | Agent-enforced condition that inhibits covered motion | low battery, stale required feedback |

Keep picking, exploration, retry strategies and delivery missions in user code. A generic pick verb is outside v1. Do not add a framework for expressing entire missions.

### 2.1 Naming and cost rules

- Properties MUST be local reads. They may expose immutable handshake metadata or an explicitly documented local cache; they MUST NOT silently perform RPC.
- `get_*()` obtains one observation or current remote state. `stream()` observes repeated values.
- Ordinary finite action verbs block until verified completion and return a typed result.
- `start_*()` submits finite work and returns an `Operation`; its return means acceptance, not completion.
- `control()` and `teleop()` create lazy scope objects. Entering the scope performs acquisition. A separately named `open()` is available for manual lifetime management.
- Starting work MUST NOT depend on starting iteration. Iteration only observes already accepted work.
- Sync and async forms MUST have identical effects and lifetime rules. Async changes `with` to `async with`, iteration to `async for`, and awaiting of remote methods; it MUST NOT change cancellation semantics.
- Optional capabilities fail with `CapabilityNotSupported`; singular accessors fail with `AmbiguousCapability` rather than choosing an arbitrary instance.

Examples of final naming:

| Current family | v1 form |
|---|---|
| Remote `robot.pose` | `robot.get_pose()` |
| Remote `slam.mode` | `slam.get_state().mode` |
| Blocking navigation | `robot.navigate_to(goal)` |
| Task-returning navigation | `robot.start_navigation(goal)` |
| Route iterator | `robot.start_route(waypoints)` returning an operation with progress |
| Blocking trajectory | `control.run(trajectory)` |
| Trajectory iterator | `control.start_trajectory(trajectory)` returning an operation |
| Motion watch with `stop=True` | `robot.guard(condition)` |

Cached `robot.name`, `robot.capabilities`, resource mappings and model metadata remain properties. Refresh is explicit.

## 3. Reference programming experience

### 3.1 The ordinary path

```python
from ringfour import Robot

with Robot() as robot:
    arrival = robot.navigate_to(target_pose, timeout=60.0)
    print(arrival.final_pose)
```

`target_pose` is an explicit validated Pose in the intended reference frame, obtained from geometry or a prior observation. No implicit place-name resolution is performed.

The convenience method acquires transient control, submits the goal, waits for verified completion and releases its acquisition. The user does not manually manage a task to perform one ordinary action.

### 3.2 Observe progress and leave deliberately

```python
with robot.start_navigation(target_pose) as op:
    for update in op.updates():
        if should_abort(update):
            break
    # Breaking iteration only stops observation.
# Leaving the operation scope cancels unfinished work and performs bounded cleanup.
```

The context boundary, not garbage collection or an iterator finalizer, defines cleanup. Keeping the operation alive while closing an observation stream does not cancel it.

### 3.3 Compose arm behavior

```python
from ringfour import JointTrajectory
from ringfour import kinematics as kin

arm = robot.arm
approach = kin.ik(arm, approach_pose)  # JointConfiguration
contact = kin.ik(arm, contact_pose)
trajectory = JointTrajectory.from_waypoints(
    [approach, contact], step_duration=2.0
)

with arm.control() as control:
    result = control.run(trajectory, timeout=10.0)
```

In this proposed default, the agent resolves the execution implementation. If selecting a control behavior is necessary, expose a documented robotics mode with an enum; do not require a ROS controller name or lifecycle role.

The same session can submit sequential operations without repeated controller switching. Tool commands follow the same authority and completion rules; a convenience tool command acquires a transient child acquisition for the same client. Physical tool contact/holding is a separate observation from achieving a jaw-position target.

### 3.4 A persistent condition during an operation

```python
condition = robot.signals.battery.level.below(10.0)
with robot.guard(condition) as guard:
    robot.navigate_to(target_pose)
```

Signal units are exposed in metadata; battery level remains percentage points from 0 to 100. Entering the guard confirms installation and an initial fresh evaluation before returning. An already unsafe or unknown required condition prevents entry and therefore prevents the body from running. If a guard trips after entry, it stops covered work and prevents new covered motion until explicitly reset or removed by an authorized owner under the rules below.

## 4. One operation lifecycle

Public operation states are:

```text
ACCEPTED -> RUNNING -> SUCCEEDED
    |          |----> FAILED
    |          |----> CANCELLING -> CANCELLED
    |                            -> FAILED
    |----> CANCELLING -> CANCELLED / FAILED
    |----> FAILED
```

A fast operation may transition directly from ACCEPTED to SUCCEEDED. Terminal states never change. A cancellation request racing with completed work returns the existing terminal outcome; it does not rewrite success as cancellation.

Transport knowledge is separate: a disconnected client may not know the latest operation state. Do not invent a terminal CANCELLED or FAILED state merely because the observer lost its connection.

### 4.1 Required operation API

```python
op.id                       # stable operation ID
op.snapshot                 # immutable locally cached snapshot; may be stale
op.get_status()             # authoritative query, includes sequence and observed age
op.updates()                # progress stream, typed for this operation
op.result(timeout=None)     # typed result or typed exception
op.cancel()                 # CancelReceipt; request accepted or already terminal
op.close()                  # bounded cancel-and-cleanup if unfinished; idempotent
```

`Operation[TResult, TProgress]` is generic at the Python surface. Results and progress are typed public objects, not arbitrary dictionaries. Internal wire payloads may be dictionaries after schema validation.

`result()` raises `OperationCancelled` for cancellation, a specific execution error for failure, and `WaitTimeout` if only the wait expires. It never returns `None` as an ambiguous cancellation result.

`CancelReceipt` records the operation ID and whether cancellation was accepted or the operation was already terminal. It is not evidence of physical rest.

CANCELLED means execution has ended through cancellation and the operation’s required stop criterion has been verified. If cancellation cannot establish that criterion within its budget, transition to FAILED with `StopUnverified` and quarantine the affected resource. A result can be terminal while the physical condition remains explicitly unverified; terminal never implies safe by itself.

Terminal records include completion evidence, timestamps, model revision and any stop/cleanup report relevant to the outcome. Progress fractions are optional; if supplied, expose their basis (elapsed-time estimate, remaining distance, or measured completion). Do not present elapsed time as measured physical progress.

### 4.2 Timeouts and Python cancellation

| API event | Required behavior |
|---|---|
| `op.result(timeout=...)` expires | Raise `WaitTimeout`; operation continues; handle remains usable |
| Blocking convenience motion call times out | Request cancellation, perform bounded cleanup, then raise `OperationTimeout` containing operation ID and cleanup/stop outcome |
| Submission reply is lost | Raise `SubmissionUnknown` with command ID and agent epoch; do not blindly resubmit |
| Async observer coroutine is cancelled | Stop that observation; do not implicitly cancel an independently owned operation |
| Async blocking convenience action is cancelled | Attempt bounded remote cancellation and cleanup before propagating Python cancellation |
| Operation scope exits | Cancel unfinished work and attempt bounded cleanup, whether exit is normal or exceptional |
| Connection dies | Agent follows the lease policy in section 5; client reports loss of knowledge |

`timeout` on convenience actions bounds execution waiting, not physical cleanup. Any additional cleanup budget MUST be finite, advertised in policy and documented. Async cleanup MUST be protected from ordinary caller cancellation for that bounded interval; a killed client process still relies on agent-side enforcement.

If the scope body already raised, cleanup failures MUST remain attached to that exception or be grouped with it; never silently replace or discard either failure. Without a body exception, failed cleanup raises `CleanupFailed` with its report.

## 5. Ownership and resource lifetime

### 5.1 v1 policy: one motion writer per robot

Choose a conservative, understandable baseline: one client control owner per robot. That owner may hold several child sessions. Different clients cannot independently command separate limbs in v1. Capability and state reads do not acquire motion ownership.

- Ownership MUST be represented by an opaque agent-issued lease ID, tied to a principal, client session, robot identity and agent boot epoch.
- Every child acquisition has its own ID and counted lifetime. Releasing one acquisition MUST NOT release another.
- Failed acquisition unwinds only changes and acquisitions made by that attempt.
- Agent mutations of ownership and controller state MUST be serialized or equivalently synchronized, including operations delegated to worker threads.
- Conflicting physical resource footprints cannot execute concurrently, even for one owner. The profile and adapter expose which resources/controllers are shared. Unsupported parallelism rejects with `ResourceBusy`; it never silently preempts an existing operation.
- Session calls include the session ID/generation, not just a group name. A stale handle cannot command a newly opened session.
- Each task has a resource set and an adapter goal handle. Cancelling a group's task MUST NOT cancel every task of that kind.
- Session revocation invalidates further submissions immediately. State checks and resource acquisition MUST be atomic with respect to acceptance; no check-then-use window may admit a revoked owner.

Navigation convenience calls and tool convenience calls participate in the same ownership system as explicit group and teleop sessions.

### 5.2 Disconnect and expiry

v1 has no detached motion. Navigation, trajectory execution and teleoperation MUST stop accepting commands and enter the configured stop/cleanup path on detected disconnect or lease expiry. A reconnection can inspect prior results but cannot silently resume motion.

Lease lifetime and teleop command freshness are separate:

- Lease expiry detects loss of the controlling client and reclaims authority.
- Teleop watchdog expiry detects stale motion commands even while a connection remains healthy.
- Heartbeats MUST NOT refresh the age of the last teleop command.
- The server clamps requested lifetimes to integrator-defined policy and returns the granted values. No hidden client background task may keep resending a stale drive command as if it were fresh intent.

Exact times are robot-specific, visible in the handshake, and tested against the deployment. No arbitrary universal millisecond budget is imposed by this API spec.

### 5.3 Cleanup outcomes

Session exit proceeds through stopping, controller restoration and release. Its `CleanupReport` describes each step as confirmed, failed or unverified.

If stopping or restoration cannot be verified, the agent MUST quarantine the affected resource and reject new motion on it. Releasing the vanished client's ownership MUST NOT clear that quarantine. Recovery requires an authorized, explicit action that rechecks the required state; reconnect and controller renaming are not recovery.

## 6. Uniform admission and stop enforcement

Replace the hand-maintained list of motion method names with mandatory method descriptors. Every registered RPC declares:

- effect class: read, motion, stop, configuration or power;
- required permission;
- resource resolver;
- ownership requirements;
- guard/e-stop behavior;
- validation schema;
- completion model.

An unclassified method MUST prevent agent startup. Profile-defined vendor methods follow the same system. Raw controller names bypass naming portability only; they MUST NOT bypass access control, ownership, input validation or stop policy.

Before accepting motion, the agent MUST verify protocol validity, permission, resource availability, ownership, model revision, numeric validity, required feedback freshness and active inhibits. It MUST revalidate mutable conditions at execution dispatch. A guard/e-stop event revokes the eligibility of already accepted affected work and initiates stopping.

Stop requests do not require acquisition of the motion lock. Authorized stop remains possible when another client owns motion. Emergency stop release/reset and quarantine recovery are separate privileged operations. A software SDK stop is not named or documented as a substitute for a hardware emergency-stop function.

Power-off and configuration writes may affect motion or stopping. They require explicit resource dependencies and policy; never treat them as harmless because they do not submit a trajectory.

### 6.1 Stop contract

`resource.stop(timeout=...)` requests stopping of all operations within that resource's declared effect scope, waits for bounded verification and returns `StopReport` only when the advertised stop criterion is verified. Otherwise it raises `StopUnverified` carrying the partial report and the resource remains inhibited.

`robot.stop()` covers the whole robot. A narrower stop may broaden to a declared shared-controller footprint, but that footprint MUST be discoverable and reported. Silent broadening is forbidden.

The report distinguishes request dispatch, controller acknowledgement, observed motion state and controller restoration. Verification uses robot-specific fresh feedback and thresholds. For some resources, safe holding requires continued actuation; “stopped” does not universally mean zero torque or zero power.

### 6.2 Agent scheduling and failure boundaries

Admission, lease expiry, guard evaluation and stop dispatch MUST remain responsive while ordinary adapter calls are slow. Blocking ROS/controller work cannot run on the monitoring event loop. Adapter calls and cleanup steps have bounded waits, and overload behavior is explicit; queued stale motion is rejected rather than executed late.

The agent cannot enforce its own software timers while its process or host is stalled. Each robot integration documents the independent controller/hardware watchdogs it relies on, what they cover, and what happens on agent loss. The advertised timing contract separates agent detection, controller acknowledgement and measured physical stopping.

## 7. Observations and evidence of success

### 7.1 Observation envelope

Measurements carry an immutable envelope (working name `Measurement[T]`; this is distinct from the current speculative synchronized-policy bundle named `Observation`, which is deferred in section 16):

```python
Measurement[T](
    value=...,
    source_stamp=...,
    clock_domain=...,
    received_at=...,          # agent monotonic clock within agent epoch
    age=...,                  # agent-computed conservative age at response
    validity=...,
    robot_id=...,
    model_revision=...,
)
```

Source freshness and receive freshness MUST be distinguished. A freshly received replay of old sensor data is not a fresh physical observation. Unknown source age is represented explicitly. Never compare unrelated client, ROS simulation and agent clocks without a declared conversion.

`get_*` methods take `max_age` and a bounded `timeout` where meaningful. If required freshness cannot be established, return an explicit unavailable/invalid observation for inspection or raise `StateUnavailable`/`StateStale` when the caller requested a usable value. Agent execution checks do not rely on client assertions of freshness.

Payload arrays MUST have documented lifetime and mutability: retained frames remain valid for their stated lifetime; shared-memory reuse cannot silently mutate a retained immutable observation. Expiry raises a typed error, and explicit copying is available.

### 7.2 Completion evidence

SUCCEEDED means the operation's declared postcondition was verified:

- Trajectory: every required joint has fresh, finite measured feedback; correct model revision; target residual within per-joint tolerance.
- Navigation: fresh localization in the goal's map/frame, within declared position/orientation tolerances, plus any specified settling criterion.
- Tool positioning: declared position/contact postcondition. Reaching a jaw position does not prove an object is held.
- Configuration/controller operations: requested state is read back and confirmed.

No missing joint may be silently omitted from a success check. No stale sample, absent sample or NaN may be converted to zero error. Revolute and prismatic joints use their own units and tolerances; continuous-angle comparisons respect the declared joint semantics.

A controller's success message starts verification; it does not replace it. If evidence cannot be obtained within the verification budget, fail with `CompletionUnverified`, preserve the available evidence, and apply the resource's stop policy. Robot integrations unable to support a promised verified operation MUST advertise it as unsupported rather than weakening success silently.

## 8. Strong types where mistakes matter

Introduce `JointConfiguration` containing robot identity, group identity, model revision and a mapping from joint name to value. Joint units derive from the model: radians for revolute joints and metres for prismatic joints. Canonical controller ordering is generated internally.

`kin.ik` returns this type. `JointTrajectory` contains configurations and strictly increasing finite times from start. It validates matching identities, exact joint coverage, finite values and positional bounds. The agent repeats validation and applies the adapter's enforceable trajectory constraints. Do not claim interpolation velocity/acceleration guarantees from endpoint checks alone.

Keep ordinary scalar metres/seconds/radians at convenient boundaries; v1 does not need a universal units algebra. Apply strong identities where confusing two equally shaped values can move the wrong mechanism.

Model changes invalidate configurations and relevant sessions. Old revisions fail explicitly with `ModelChanged`; they are not silently rebound. Frame/capture/map identity is preserved end to end.

## 9. Watches and guards are different tools

`watch(condition)` observes and reports. It never moves or stops the robot by itself.

`guard(condition)` installs a server-side stop-and-inhibit policy. Default scope is the whole robot for v1. Triggering latches the inhibit, requests stop and records the event. Continuing unsafe readings keep motion inhibited even after the original task ends.

- A required signal becoming stale or unavailable trips the guard by default.
- Initial evaluation occurs before protected work is admitted.
- The owner can explicitly reset a tripped guard only after a fresh safe evaluation. Reset permits future submissions but never restarts previous work.
- Explicit removal requires ownership/permission and is audited. Removal does not restart work or clear other inhibits.
- Exiting the scope first completes bounded stopping of affected unfinished work before removing the guard. If cleanup is unverified, the inhibit/quarantine remains.
- Disconnect revokes the owner's motion lease; guard removal MUST NOT race ahead of stopping and verified cleanup.
- Profile-installed guards cannot be removed by an ordinary client.
- Sampling cadence, source-age policy and measured detection/dispatch latency are exposed. Documentation MUST NOT promise instantaneous physical stopping.

Use typed signal references for built-in signals, such as `robot.signals.battery.level`. Existing named helpers such as `robot.watches.battery_level` remain viable entry points; the proposed signal expression is not a requirement to replace them. Dynamic vendor signals use discovered, validated handles at the public boundary; internal wire paths may remain strings. Units/type are exposed.

## 10. Identity and permissions without an enterprise platform

v1 supports a trusted local mode and an authenticated remote mode.

- Default local access uses a Unix socket with restrictive permissions and a principal derived from the OS peer identity or a verified local gateway. A random connection ID is never presented as authentication.
- Network listening is explicit. Non-local access requires authenticated encrypted transport or a documented trusted gateway that supplies verified identity. A bounded development-only insecure mode must be opt-in and conspicuous in connection metadata.
- A minimal permission set covers read, motion, stop, configuration and recovery. Capability discovery and granted permissions are separate fields.
- Task IDs and lease IDs are references, not sufficient authority. The agent checks owner/principal on inspection, cancellation, release and reset. Deliberate cross-owner stopping uses the stop permission and is audited as such.
- Credentials and secrets MUST NOT appear in command audit payloads.

An identity-management UI, organization model, OAuth provider and fleet roles are outside v1. Deployment policy can map one local principal to all permissions while retaining the same enforcement path.

## 11. Lost replies, audit and event delivery

Every mutating submission carries a client-generated command ID, distinct from the transport request ID. The agent atomically binds `(agent epoch, principal, command ID)` to its canonical validated request and acceptance record before dispatch.

Same ID and same request returns the original acceptance/result. Same ID and different request raises `CommandConflict`. Two concurrent duplicates MUST NOT execute twice. `robot.commands.get(command_id)` reconciles a lost reply.

Retention is advertised. Use a bounded command validity window and retain deduplication records through that window; expired IDs are rejected rather than treated as fresh commands after eviction. A changed agent epoch invalidates retries from the earlier epoch. After restart, prior physical outcome may be unknown; do not claim exactly-once physical execution across crashes. v1 does not automatically resume interrupted motion.

Operation terminal records MUST be queryable for the advertised retention period even if a progress notification is lost. Intermediate updates may be coalesced with sequence numbers. Terminal outcomes MUST NOT depend solely on a bounded drop-oldest queue. Observers can query authoritative status after gaps.

Errors carry stable code, human-readable message, robot/resource identity, command/operation ID where applicable, and structured details. Unknown wire error types preserve their original type and detail; the client must not discard residuals, limits or cleanup reports during reconstruction. Retry guidance distinguishes safe retry, reconcile-first and operator recovery.

Audit separates ACCEPTED from SUCCEEDED, cancellation requested from stopped, and restoration attempted from verified. Records include principal, session, resource, operation and causal guard/stop IDs.

## 12. Documentation and discovery

The handshake MUST expose identity, protocol version/features, capabilities, resource topology, model revisions, granted permissions, effective lease/watchdog policies and supported completion/stop verification. This describes the contract, not raw ROS topic configuration.

Generate reference signatures, enums, errors and wire schemas from the same typed definitions wherever practical. Handwritten material explains behavior and intent. CI checks examples and references against the public API and flags generated drift.

The course MUST teach, in order: discovery, framed observations, one blocking action, operation observation/cancellation, scoped control, guards, composition and recovery. Move integrator-only ROS setup to an explicit prerequisite guide. Remove the stale pick verb, future-phase claims about shipped sessions and old directory names.

Every motion example MUST demonstrate a lifetime boundary. At least one example demonstrates an uncertain outcome and one demonstrates a cleanup failure without pretending the robot has stopped.

## 13. Implementation sequence and release gates

### Milestone A — Make existing guarantees true

Implement mandatory method descriptors, group/tool admission checks, acquisition accounting, explicit iterator cleanup contracts, complete feedback checks and truthful teardown quarantine. These are blockers independent of the public rename.

**Implementation status (13 October 2026):** A software contracts are implemented and verified on `review_and_hardening`; see [the final evidence and acceptance trace](HARDENING_LOG.md#full-a-final-evidence-and-completion-boundary). This does not mark B/C/D, the full v1 acceptance matrix, or physical commissioning complete.

### Milestone B — Unify the public contract

Introduce operations/results, the naming rules, typed joint trajectories, observations and guards. Migrate sync and async together. Remove superseded unreleased interfaces in the same change series.

**Implementation status (13 October 2026):** B software and Handy simulator acceptance are complete; see [the contract/migration and acceptance evidence](docs/milestone-b.md) and [the completion log](HARDENING_LOG.md#b-completion-evidence-and-boundary). The deployed agent matches checkout Python sources. C/D, the full v1 release matrix and physical commissioning remain separate.

### Milestone C — Close distributed-lifecycle gaps

Implement finite leases, submission reconciliation, terminal-result queries, minimal identity/permissions and explicit recovery. Test reply loss and concurrency with a fake adapter before robot integration.

**Implementation status (7 November 2026):** C software and XLeRobot simulator acceptance are complete. See [the distributed lifecycle contract](docs/milestone-c.md) and [C completion evidence](HARDENING_LOG.md#2026-11-07--milestone-c-completion). D, authenticated encrypted remote deployment, the full v1 release matrix and physical commissioning remain separate.

### Milestone D — Validate and document the platform

Validate the SDK, agent and adapter contracts using controlled platform fixtures, and update the reference, runnable examples and course. Keep contract and fault-injection checks persistent and independently selectable in the testing dashboard. Complete D after the repository split and Ringfour rename so it validates the maintained package layout.

Platform acceptance must not depend on a particular robot's ROS 2 stack. Use deterministic fake adapters and isolated real client/agent transports; add a minimal controlled ROS 2 fixture where adapter semantics require ROS. Robot integration conformance remains a separate, explicitly selected suite. Qualification of two robot configurations is not a D or platform-release gate. Physical timing and stop/arrival claims still require evidence for each deployment that advertises them.

**Scope decision (13 November 2026):** A–C provide the existing boundary for the split; do not reopen API design during migration. Split and rename SDK, agent and testing together, verify builds and cross-package communication, then finish D. D remains incomplete; this scope change is not acceptance evidence.

### Required acceptance scenarios

| ID | Scenario | Required result |
|---|---|---|
| A01 | Unclassified vendor or built-in RPC | Agent refuses startup |
| A02 | E-stop/inhibit active; submit navigation, arm, hold or tool motion | All affected paths reject before controller dispatch |
| A03 | One client holds two child sessions, closes one | Other acquisition remains valid and excludes competing clients |
| A04 | Opening an additional session fails halfway | Only that attempt unwinds; earlier sessions remain owned |
| A05 | Retain an update iterator, then break | Observation ends; operation continues until explicit cancel/scope exit |
| A06 | Leave unfinished operation scope | Bounded cancel/cleanup; no dependency on GC |
| A07 | Missing, partial, stale or non-finite joint feedback | Never reports verified arrival |
| A08 | Controller says success, measured residual exceeds tolerance | Typed failure preserving residuals and units |
| A09 | Restoration fails | Failed report, affected resource quarantined, later motion rejected |
| A10 | Guard fires, then caller submits another command while condition persists | Command rejected until authorized reset/removal under guard rules |
| A11 | Required guard signal disappears | Guard trips according to declared age budget |
| A12 | Client dies or connection becomes half-open | Lease/watchdog policy initiates stop within measured deployment budget |
| A13 | Session A is closed; new session B opens; A's handle is reused | Stale session rejected |
| A14 | Submission executes but response is lost; duplicate arrives | Original operation returned; no duplicate dispatch |
| A15 | Agent restarts before retry | Old epoch rejected; no automatic motion replay |
| A16 | Task owned by another principal is cancelled through owner API | Permission denied; authorized global stop still works |
| A17 | Cancel one group's operation | No unrelated goal cancelled unless declared shared footprint requires it |
| A18 | Terminal event lost under backpressure | Query still returns authoritative terminal outcome |
| A19 | Model changes after IK | Old configuration rejected before dispatch |
| A20 | Same mission in sync and async | Equivalent effects, outcomes and cleanup behavior |
| A21 | Body exception plus cleanup failure | Both failures retained and inspectable |
| A22 | Joint vector names correct but revision/group wrong | Validation rejects; controller never receives it |
| A23 | Two racing acquisitions/submissions or cancel versus completion | One serialized, consistent outcome; no stale authority or rewritten terminal record |
| A24 | Stop lacks fresh verification | No successful stop report; resource remains inhibited |
| A25 | Duplicate command after retention/validity expires | Explicit expiry/unknown outcome, never new execution under old ID |

Use deterministic fake-controller tests for protocol and state-machine behavior; integration tests for adapter semantics; deployment measurements for latency and physical verification. A passing unit suite alone does not establish a stop-time bound.

### Additional public-surface acceptance scenarios

| ID | Scenario | Required result |
|---|---|---|
| A26 | Inspect ordinary SLAM/map API | No backend selector/name in mission surface; saved-format compatibility is handled internally |
| A27 | Discover a group or enter control | No need to name/load/activate a ROS controller; lifecycle bookkeeping stays in integration diagnostics |
| A28 | Invoke a vendor operation | Result states domain outcome or dispatch-only meaning; no public ROS topic/payload receipt |
| A29 | Missing model or failed backend service | Primary error describes unavailable robotics function; structured diagnostics retain original cause without speculative diagnosis |
| A30 | Alias and canonical frame refer to the same frame | Identity is reconciled consistently; examples do not depend on base_link spelling |
| A31 | Inspect every stable exported type/field/error | A current producer/consumer or deliberate standalone value contract exists; no permanently empty feedback placeholders |
| A32 | Call supported power APIs from sync and async clients | Matching registered endpoints; sync returns values; unknown source remains unknown |
| A33 | Run public examples and scan generated reference | No retired aliases, old dotted-watch examples, nonexistent pick/execute/buttons or contradictory phase claims |
| A34 | Launch mission from a different directory | Advertised model-artifact availability does not depend on the integrator checkout layout |
| A35 | Unknown residual/uncertainty or mixed joint units | Unknown remains explicit; no misleading zero or raw mixed-unit maximum |
| A36 | Reverse/lateral movement or IMU angular-rate watch | Public names accurately identify monitored signed component, magnitude or angular rate |

## 14. Definition of done

The platform is ready when all 36 required scenarios pass at their applicable platform layers under section 18, every effectful API has an explicit lifecycle/authority/completion contract, documentation agrees with implementation, and the ordinary examples remain short.

A programmer should learn the operation and session rules once and reuse that understanding for base, arm and tool. Failures must be explicit enough to recover without guessing whether motion happened or whether control was restored.

## 15. Deliberately deferred

Detached motion that survives its client; multiple independent motion owners; autonomous mission DSLs; built-in universal pick/place; fleet orchestration; universal dimensional-analysis types; automatic recovery that resumes motion; backward-compatibility aliases for unreleased APIs; enterprise identity management.

These can be added later without weakening the six core concepts or changing the meaning of success, cancellation, ownership and freshness.


## 16. Public-surface pruning requirements

This section incorporates the infrastructure-leak scan and deeper pruning audit. “Remove” means remove from the ordinary stable mission surface, not erase information needed by adapters or internal storage. “Defer” means do not advertise as stable until the stated contract is implemented. “Review” requires a documented disposition before release; it does not authorize mechanical deletion. These dispositions supersede any earlier suggestion to retain backend identity as ordinary mission metadata.

### 16.1 Remove or relocate implementation-only members

| Member or behavior | Required disposition |
|---|---|
| `slam.backend` | Remove from the mission API |
| `MapInfo.backend` | Remove from public map metadata; retain private format/backend provenance where needed for loading compatibility |
| Session `.controller`, `.loaded`, `.deactivated` | Relocate to explicit integration diagnostics |
| `GroupInfo.controllers` | Relocate role-to-backend-name mappings outside ordinary group metadata |
| `control(raw=...)` | Move to an explicitly advanced integration escape hatch if retained; no bypass of admission/ownership rules |
| Root `Transport` export | Remove from ordinary root vocabulary; preserve transport extension mechanisms in their own module |
| Vendor return `topic`/`sent` | Replace with domain result or explicitly limited dispatch receipt; publishing is not physical completion |
| `RobotDescription.hardware_joints` | Separate ros2_control-derived declarations from physical model data or expose them only as explicit integration provenance |

Controller choice requires a semantic decision, not merely a renamed enum. Trajectory/velocity/streaming behavior is valid robotics vocabulary. Actual controller names and load/configure/activate/restore sequences are adapter responsibilities.

### 16.2 Complete existing retirements

Remove `robot.joint_groups` in favor of `robot.groups`, and the deprecated `group.gripper` alias in favor of the existing `group.tool`. Retire the public dotted-string `robot.watch(path, ...)` grammar in favor of existing named/validated helpers, while retaining internal routing required by those helpers. Apply the migration consistently to sync, async, examples, course and tests.

`handles.py` is a retired placeholder module. Remove it from distribution unless an explicit packaging reason requires a tombstone; this is low-priority housekeeping, not an API redesign.

A future specialized gripper type is not prohibited. These removals target existing redundant aliases, not legitimate domain concepts.

### 16.3 Defer speculative public promises

| Candidate | Release requirement |
|---|---|
| Current root `Observation` synchronized-policy bundle | Defer or mark experimental until a real acquisition/synchronization API produces it; do not confuse it with measurement provenance in section 7 |
| `GraspFailed` | Remove from stable errors unless an implemented operation emits it; no built-in pick verb is implied |
| `PlanningFailed` | Defer until a supported planning operation needs a distinct failure; preserve existing NavigationFailed/Unreachable semantics |
| `GeoPose` | Review: retain only with a concrete supported use or clearly documented standalone value purpose; no geographic navigation is implied |
| `MapInfo.is_3d` | Defer unless the public map model actually supports both dimensionalities; current constant False is not a 3D contract |
| `TrajStatus.ee_pose` / `.effort` | Implement meaningful availability-aware feedback or remove from stable status; current producer omits pose and always emits empty effort |
| Phase values with no producers | Publish an operation-to-phase matrix; defer speculative phases rather than promising them. Cancellation/failure representation must follow the lifecycle decision, not search-based deletion |

An exported type is not justified solely by a schema-generation test or by a future roadmap. An intentionally standalone mathematical value can be useful without an agent producer, but its supported purpose must be stated.

### 16.4 Power: implement or withdraw the incomplete path

Power control is useful functionality, not inherently junk. The reviewed checkout nevertheless has public client calls to `power.battery`, `power.is_on_battery`, `power.lane_state` and `power.set_lane` without corresponding registration in the shipped agent. Sync PowerAPI also declares battery/source methods as async functions.

Before release, implement and conformance-test these endpoints or withdraw/defer the unsupported methods and advertised capabilities. Preserve working battery observation through the separate status path. Choose one canonical battery retrieval contract and one actual power-lane collection path; documentation must not advertise `robot.power.lanes` unless implemented.

Unknown source is not external power: represent unavailable state explicitly. Sync methods return values rather than coroutine objects. Renaming the internal `aio.ui` module to power is optional housekeeping and does not satisfy these requirements.

### 16.5 Consolidate only where behavior is actually redundant

- Review `robot.battery` versus `robot.power.battery()` after fixing their different execution paths. Convenience aliases are allowed only when they share the same contract.
- Review `group_info`, `group_infos` and per-group metadata for needless duplication. Keep useful DOF, limits, tool identity and aggregate introspection.
- Keep model identity/revision even if `description_sha` and `description.sha` are consolidated. Define behavior when model content is unavailable.
- Preserve task-returning navigation; rename `navigate_to_async` if needed to distinguish task submission from Python async execution.
- Do not merge `localize` and `relocalize` solely because the names resemble one another. Retain distinct behaviors when justified and documented.

### 16.6 Repair misleading measurements and names

- `localization_quality` and `is_lost` MUST describe evidence the integration actually provides. A correction-stability heuristic is not automatically localization accuracy or calibrated confidence. Expose validity and measurement meaning; no arbitrary cross-backend equivalence of scores.
- `worst_residual` MUST NOT turn absent evidence into zero error or compare raw metres with radians. Use per-joint residuals or an explicitly defined dimensionless normalized measure.
- `watches.speed` must either mean magnitude or be named as the signed forward component it monitors. Preserve separate lateral and angular quantities.
- `watches.imu_rotation` must be named/documented as angular rate if that is what it observes.
- Unknown variance MUST remain distinguishable from zero uncertainty.
- `distance_traveled` needs an explicit measurement epoch/reset boundary or mission-relative reference. Agent restart must not silently look like ordinary distance evolution.

These are contract repairs. Do not delete useful scientific observations to avoid defining them.

## 17. Infrastructure boundaries and source-backed migration

### 17.1 Frames and transforms

Retain coordinate transformations, lookup, pose inspection and frame graphs. `robot.tf` is a candidate naming change, not a reason to remove the functionality. Select a domain-oriented spelling only if it improves discoverability. Regardless of spelling, alias and canonical frame identities must work coherently in returned values and local mathematics. Normal examples should use discovered references or available semantic enums rather than depending on ROS-specific base/map spellings.

### 17.2 Robot models and recorded artifacts

Keep links, joints, limits, collision geometry, URDF interchange and model inspection. Receiving no description means model unavailable; it does not prove a particular publisher is absent. Avoid defining model state in terms of ROS topic lifecycle.

Advertised collision geometry and reach maps must be obtained through an explicit robot-resource/cache contract. The mission working directory or presence of an integrator repository must not silently determine feature availability. Local paths/environment overrides remain available for advanced tooling, and absence of optional artifacts is reported explicitly.

### 17.3 Errors and diagnostics

Primary messages explain the failed robotic operation, available evidence and effect on execution. Retain original ROS/service/controller errors in structured integration diagnostics. Do not require ordinary callers to diagnose latched topics or controller-manager service names, and do not infer a specific backend fault from missing data alone.

Vendor actions remain valid extensibility. Their payload/result contracts must not expose raw publication as if it proved completion. Camera calibration, capture identity and geometric frame metadata remain normal domain data.

### 17.4 Documentation migration checklist

Replace or remove all obsolete public examples, including `arm.pick`, the nonexistent current `arm.execute` spelling, deprecated dotted watches, future-phase descriptions of already shipped sessions, and contradictory power paths. Move topic/message YAML, node/service lifecycle, controller claimed-interface details, ros2 setup commands and onboarding commands into integrator/backend material linked from the mission guide.

Keep scientific vocabulary. A term does not become infrastructure junk because ROS also uses it. Preserve useful singular/collection accessors, calculation helpers, observation APIs, completion-versus-progress conveniences, and explicit identity/health/audit inspection.

Public examples should be executable or signature-checked against the actual API, and the canonical reference should not teach removed behavior. Test human and LLM task attempts against the same reference offline before any generated motion is attempted. Use recurring misunderstandings as evidence for ergonomic changes rather than redesigning on naming preference alone.

### 17.5 Migration evidence and tracking

The full findings and source locations are recorded in the adjacent read-only audit documents:

- `ringfour-infrastructure-leak-scan.md`
- `ringfour-public-surface-pruning-audit.md`

Implementation must recheck producer/consumer searches against the current checkout. Absence of usage in this review does not prove absence of private downstream consumers. For each public member, record one disposition: keep, remove, relocate, implement or defer; update examples/tests together. Do not leave removal candidates documented as supported while merely adding an upgrade note elsewhere.

The pruning work joins milestones A/B; power and artifact delivery must be resolved by integration/conformance milestones C/D. A release disposition is required for every review item in section 16, without demanding speculative features be implemented simply to preserve an existing name.

## 18. Test implementation and execution instructions

This section specifies the tests implementers MUST add and run. It does not claim these tests already exist or have passed. Existing tests are retained where useful and migrated with intentionally removed APIs. Test behavioral contracts rather than private function structure.

### 18.1 Test environments

| Tier | Environment | What it establishes |
|---|---|---|
| T0 — static/public surface | SDK source, generated reference and example files | Removed names, supported signatures, schema consistency, documentation accuracy |
| T1 — deterministic contract | In-process agent components with fake clock and fake robot adapter; no ROS, network or hardware | Ownership, lifecycle, guards, admission, validation and outcomes |
| T2 — protocol integration | Real client and agent dispatcher connected over isolated local transport; fake robot adapter | Serialization, identity, reply loss, reconnect, event delivery and sync/async parity |
| T3 — robot integration | Explicitly selected shared conformance suite against a robot stack and reviewed profile | Adapter behavior and portable contract; no hardcoded robot-specific expectations |
| T4 — deployment verification | Commissioned hardware and approved test envelope, independent stop mechanism and responsible operator | Measured timing, arrival/stop evidence and behavior under supported hardware faults |

T0–T2 MUST run in CI without a robot or ROS installation. T3 is a separate integration qualification job, not a D or platform-release gate. A controlled ROS 2 fixture checks platform adapter semantics independently of T3; run it in a dedicated ROS-enabled job. T4 is a recorded commissioning/release procedure for each deployment; never run automatically from ordinary unit tests. Passing simulation cannot substitute for measured hardware stopping behavior.

### 18.2 Required fixtures

Build reusable fixtures rather than mocking a different internal method in each test:

1. **Fake monotonic clock:** explicitly advance lease expiry, source age, cleanup deadlines and guard debounce; do not use real sleeps to prove timeout behavior.
2. **Recording adapter:** exposes capabilities and resource footprints; records each command with operation ID, goal handle and sequence; can accept, reject, stall or acknowledge calls without moving hardware.
3. **Feedback source:** supplies complete/partial/missing/non-finite/stale joint samples, poses and velocities; distinguishes source and receive timestamps and supports separate clock domains.
4. **Controller lifecycle simulator:** records load/configure/switch/restore steps and can fail each step independently, including during rollback.
5. **Transport fault proxy:** can drop a chosen request, drop only its response after dispatch, delay/reorder/duplicate responses, disconnect and create a half-open connection. Use actual protocol frames at T2.
6. **Two principals and multiple sessions:** an owner, another ordinary client, and separately granted stop/recovery permissions. Never test authorization with only one connection.
7. **Robot fixtures:** one mobile-base configuration and one articulated configuration with multiple groups, a tool and at least one shared controller footprint. Include synthetic non-production joint names and a mixed revolute/prismatic group.
8. **Event recorder:** collects authoritative operation transitions, admission rejections, audit outcomes and adapter invocations. Assertions compare these observations, not only the exception returned to the client.

The fake adapter MUST implement the same semantic interface used by production adapters. Tests MUST fail if the implementation bypasses admission through a special test path.

### 18.3 Test case format

Each test records its acceptance ID (A01–A36), initial state, action/fault, expected public result and expected adapter/audit effects. For rejected motion, assert **zero actuator dispatches**, not merely that an exception occurred. For cancellation, assert the exact affected goal/resource set and the verification outcome. For unknown outcomes, assert absence of fabricated success/cancellation.

Use parameterized cases for applicable operation families: navigation, trajectory, hold, tool, vendor motion, and motion-affecting power/configuration. Run public behavioral cases against both sync and async clients. An adapter that does not support a family must advertise that absence and pass the unsupported-capability case rather than silently skip an advertised feature.

### 18.4 Coverage assignment

| Acceptance IDs | Minimum tiers | Required coverage |
|---|---|---|
| A01–A04 | T1, T2 | Method classification; all effectful paths; competing/nested acquisition and partial entry failure |
| A05–A06 | T1, T2 | Retained iterator, normal/exceptional context exit, manual close, async caller cancellation |
| A07–A08 | T1, T3, T4 where advertised | Feedback completeness/freshness/numeric validity and measured tolerance |
| A09 | T1, T2, T3 | Fail every restore step; quarantine persists across reconnect and release |
| A10–A11 | T1, T2, T3 | Initial unsafe condition, trip, persistent inhibit, stale source, reset/removal authorization |
| A12 | T1, T2, T4 | Disconnect, half-open connection, expired lease, stale command with healthy heartbeat |
| A13–A16 | T1, T2 | Stale generation, duplicate/lost replies, restart epoch, owner versus stop authority |
| A17–A18 | T1, T2, T3 | Specific goal cancellation, shared footprint, event overflow and terminal query |
| A19–A23 | T1, T2 | Model changes, sync/async parity, double failure, identity mismatch and races |
| A24 | T1, T3, T4 | Stop acknowledgement without fresh evidence cannot produce verified stop |
| A25 | T1, T2 | Retry after expiry/eviction cannot execute old command as fresh |
| A26–A28 | T0, T2 | No public backend/controller bookkeeping; truthful vendor receipt |
| A29–A30 | T1, T2 | Domain errors plus retained diagnostics; coherent alias identity |
| A31 | T0, T1 | Every retained stable member has a current contract and valid availability semantics |
| A32 | T0, T2, T3 if power advertised | Client/registry agreement, sync returns values, unknown source remains unknown |
| A33 | T0, T2 | Current references/examples and retired API migration |
| A34 | T1, T2 | Same connected model from unrelated working directories and empty local checkout/cache |
| A35–A36 | T1 | Unknown values, mixed units, reverse/lateral motion and angular rate |

### 18.5 Mandatory fault-test recipes

**Lost submission reply (A14):** submit one motion command with ID K; let the adapter accept it; drop only the reply; observe SubmissionUnknown; reconcile K; resubmit identical K concurrently from two retries. Assert one actuator dispatch, one operation identity and consistent acceptance records. Change the payload with K and assert CommandConflict. Repeat with new agent epoch (A15) and after command validity expires (A25); neither may replay motion.

**Counted ownership (A03/A04):** client A acquires groups X and Y; close X; client B attempts motion and must be denied under the v1 whole-robot policy. Y remains usable. Repeat with failure opening X after Y was acquired. Closing/rolling back X must not release Y's acquisition. Finally close Y and verify the next eligible acquisition can succeed unless quarantined.

**Iterator lifetime (A05/A06):** store `iterator = iter(op.updates())`, consume one update, break and keep the iterator referenced. Assert iteration break alone did not cancel. Exit the operation scope and assert bounded cancellation/verification. Repeat with exceptions and the async equivalent. Do not call gc.collect to make this test pass.

**False arrival (A07/A08):** report controller success, then supply each of: no joints, one missing joint, stale samples, NaN, infinity, wrong revision, and excessive residual. Assert no SUCCEEDED result in every invalid case. Supply complete fresh in-tolerance data and assert success. Include revolute and prismatic joints so unit errors cannot hide.

**Cleanup failure (A09/A21):** fail each restore step in turn. Assert report identifies the failed step, resource is quarantined and reconnect cannot clear it. When the mission body also raised, assert both errors remain inspectable. Authorized recovery only clears quarantine after required state is reverified.

**Persistent guard (A10/A11):** enter guard on a fresh safe signal, start operation, cross threshold, observe stopping. Keep condition unsafe and attempt new motion: zero dispatches. Restore safe data; without explicit reset, motion remains inhibited. Check reset permission and missing/stale source behavior. Exiting/removing the guard must not race ahead of required stopping.

**Stop verification (A24):** adapter acknowledges cancellation but continues reporting motion or provides no fresh feedback. Assert StopUnverified and inhibition. In the passing case, provide fresh evidence satisfying the configured criterion and assert the report cites it. A zero command or acknowledgement alone is never sufficient evidence.

**Public-surface pruning (A26/A31/A33):** build a manifest of intentionally supported members from public exports and documented interfaces. Confirm removed aliases are absent, speculative members have been deferred or given implemented contracts, and trajectory status does not advertise permanently empty measurements. Validate all examples against supported signatures. Documentation checks target code examples and public reference entries, not historical audit files or changelog mentions.

**Power contract (A32):** enumerate client power RPC methods and compare with the registered methods under a power-capable fixture. Invoke every retained operation through sync and async clients. Assert sync calls return values rather than coroutine objects; unknown power source remains unavailable. Verify the documented lane accessor actually exists. If power is deferred, assert it is not advertised and unsupported access fails clearly.

**Model delivery (A34):** run the same request from a temporary unrelated directory, with no robot repository or profiles folder and then with an empty cache. Advertised artifacts must resolve through the declared resource contract or return explicit unavailability; they must not silently change because of cwd.

### 18.6 Commands and test entry points

Implementers MUST add these documented test entry points; they are target commands, not claims about existing packaging. Use the project's Python environment with SDK and agent installed. Add pytest markers and exclude integration/hardware work from the default suite.

```text
python -m pytest ringfour-sdk/tests -m "not integration and not hardware" -q
python -m pytest ringfour-sdk/tests -m contract -q
python -m pytest ringfour-sdk/tests -m protocol -q
python -m pytest ringfour-testing/integration -m integration -q
```

The first command includes T0/T1/T2 once implemented, requires no robot, and must not modify a live agent. Tests requiring transport use isolated temporary sockets/ports. The integration command requires an explicitly selected fixture/deployment using the repository's documented configuration mechanism; it must fail fast if no target is configured. Document exact dependency installation and fixture startup alongside these commands, replacing stale paths.

Hardware tests MUST require explicit hardware selection and operator opt-in, with preconditions printed before execution. Do not invent a universal hardware command that could discover and move an arbitrary connected robot. Publish the exact approved procedure per deployment.

### 18.7 Results, failure handling and release evidence

Each run records commit, SDK/agent/protocol versions, selected configuration and acceptance-ID coverage. Integration/hardware records additionally include model revision, effective tolerances, granted timeouts/watchdog periods, clock domains and measurement method.

A case passes only when both public outcome and underlying side effects match. Expected denial is a passing result only if no forbidden actuation occurred. A skipped required case is not a pass. T3/T4 qualify integrations and advertised deployment guarantees separately; their absence does not block platform D acceptance. Unsupported features are acceptable only when absent from advertised capabilities and documented accordingly.

For timing, record observed detection latency, dispatch latency and physical response separately, with workload and repetition count. Compare against the deployment's declared budgets; do not infer a hard real-time bound from a handful of successful samples. Any unverified stop/restore result fails that deployment's advertised verified behavior and requires investigation before release.

The release report MUST contain:

- A01–A36 mapped to test names and pass/fail/unsupported results at the required tiers.
- Public-member disposition manifest and documentation/example validation results.
- Controlled platform conformance results. Robot integration results are reported separately and are not a platform-release gate.
- Deployment evidence for every advertised physical stop/arrival guarantee.
- Explicit remaining limitations; no silent waivers or treating expected failures as completion.

Run the appropriate tiers after relevant changes. Do not repeatedly operate hardware for documentation-only changes; do rerun affected contract, protocol and integration cases whenever semantics, ownership, adapter behavior or schema changes.
