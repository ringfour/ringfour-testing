# Hardening implementation log

Started 2026-10-13 on `review_and_hardening` (base `efdc5cd`).
Specification: [HARDENING.md](HARDENING.md). This log records implemented
behavior, not proposed guarantees. No milestone is complete until its required
acceptance evidence exists.

## Execution boundaries

- User authorized offline implementation and headless Handy simulation tests.
- Target: `/home/cdr/Work/Robots/mr_handy`; containers `handy101-*` only.
- No hardware, Grove/XLeRobot, commits, pushes or automatic recovery.
- Later user authorization includes rebuilding/deploying the Handy agent and
  restarting its reviewed simulation fixture. One reusable base101 Nav2 launch
  extension supports that fixture; no other robot containers were changed.
- Existing checkout contains substantial earlier uncommitted implementation.
  Preserve it. Pre-existing tracked diff saved to
  `/tmp/ringfour-hardening-20261013/pre-existing.patch`; specification copied there.
- Work in this checkout because those earlier changes are required inputs;
  do not reset them or build an isolated checkout that silently omits them.
- Do not increase tolerances to hide environment collisions. Live motion requires
  reviewed settings and a valid environment; offline faults establish races.

## Milestone status

| Milestone | State | Evidence / next work |
|---|---|---|
| A: existing guarantees | Complete (software implementation/evidence) | A01–A09 core contracts and A24 measured-stop extension; 636 offline, 5 Handy head checks, stationary body stop/end |
| B: public contract | Complete (software and Handy simulator acceptance) | 650 offline; typed operations/results/commands, observations/guards, sync/async live evidence; see B completion below |
| C: distributed lifecycle | Complete (software and XLeRobot simulator acceptance) | Finite leases, durable command/result reconciliation, ownership and measured local recovery; 542 SDK + 172 dashboard checks, 10 shared live checks; see C completion below |
| D: portability/documentation | Partial | API/README/reference/course/examples migrated; Handy fixture accepted; second configuration and physical commissioning remain required |

## Decisions and verification

### Initial inspection

- Baseline offline suite: **302 passed**, 2 existing dotted-watch deprecation warnings,
  6.42 s. Command: `ringfour-sdk/.venv/bin/python -m pytest ringfour-sdk/tests -q`.
- Found real gaps: arm/hold/tool not in central motion gate; releasing one group
  releases another group's owner; cleanup catches failures but reports success;
  task cancellation ignores owner; terminal records are removed; trajectories
  validate width but not finite values, times or positional bounds.
- Current ROS adapter stores one trajectory goal globally. Until per-goal
  adapters exist, reject concurrent trajectories explicitly rather than let one
  overwrite/cancel another. Keep unsupported parallelism visible.
- Headless Handy agent runs an installed image, not this checkout. Any integration
  result must identify whether it exercised old installed code or the new source.

## Remaining acceptance gaps

All A01–A36 initially unclaimed. Record specific tests as work lands; passing
existing tests is not v1 release evidence. Physical stop verification, hardware
commissioning and second-robot integration are not established by this run.

### First implementation wave: admission, ownership and evidence

- Added mandatory effect inventory; an unknown registered RPC refuses startup.
  Central admission now covers group acquisition, trajectory, hold and tool.
  Full descriptor schemas/permissions/resource resolution are still pending.
- Added counted acquisition tokens and serialized controller lifecycle mutations.
  Group handles carry opaque session IDs; stale IDs fail. Closing/rolling back
  one group retains other acquisitions. Teleop and navigation use separate tokens.
- Controller cleanup attempts every step, verifies restored states by readback,
  preserves errors and quarantines all subsequent motion on failure. Reconnect
  and release do not clear quarantine. Explicit authorized recovery is pending.
- Trajectory numeric/time/bounds validation and complete finite arrival checks.
  Added per-joint tolerances and separate source/receive freshness checks.
  Effective group feedback policy appears in the handshake.
- One active trajectory adapter is enforced; group stop targets its own task
  resource. Cancellation acknowledgement timeout/rejection is now reported.
- Owner-checked task cancellation/status, immutable terminal outcomes, retained
  queryable records (300 s; 1024 records, fail closed at capacity).
- SDK wire errors retain `.details` and `.wire_type`; residual/cleanup evidence
  no longer disappears in reconstruction.
- Generated protocol/type references were regenerated in the same wave.
- Offline validation after the first wave: **474 passed** (SDK + dashboard).
  Includes five real Unix-socket false-arrival tests using production client,
  dispatcher and serializer, with recording physical adapter.

### Second implementation wave: explicit scope cleanup

- Sync/async trajectory and route iteration now only observes; break does not
  cancel based on iterator GC. Explicit scope exit/close defines cancellation.
- Lazy control openers create no unused coroutine or authority; `.open()` exists
  in both facades, and `control()` defaults to trajectory behavior.
- Bounded async cleanup shields caller cancellation; body and cleanup errors are
  both inspectable. Full v1 eager operation API is still pending.
- Removed `worst_residual`: it treated absence as zero and compared mixed units.
  Per-joint residuals remain available. Updated route mission/course sources and
  API documentation alongside behavior, not just a migration note.
- Added contract/protocol/integration/hardware markers. Live cases receive the
  integration marker; default offline commands cannot silently run them.

### Handy integration (new source, headless simulation)

- Original agent image was temporarily replaced by an **ephemeral** container
  running SDK + agent directly from the checkout through a read-only bind.
  Original ROS/router/simulator containers were left running. No image or robot
  profile was modified. Original `handy101-agent-1` restored afterward.
- First preflight used host networking incorrectly: controller services were
  unavailable. Corrected to the original ROS container's network namespace and
  host IPC. Recorded the failed collection as infrastructure failure, not robot
  incompatibility; waited for `agent ready` before the real run.
- Compatibility run: **57 robot passed, 4 failed, 85 skipped**; additionally
  139 dashboard offline cases passed. Skips: 29 N/A, 44 motion-disabled,
  12 missing configuration. XML: `/tmp/ringfour-hardening-20261013/handy-compatibility.xml`.
- Four failures are **pre-existing simulator/model limit violations**, observed
  before any motion: left elbow about -0.002583 rad vs lower -0.001;
  right elbow -0.002932 vs -0.001; left/right lifts -0.000321/-0.000332 m
  vs lower 0. Did not increase tolerances or move these groups to hide failures.
- Reviewed bounded head-only test configuration saved to
  `/tmp/ringfour-hardening-20261013/handy-head.conformance.yaml`.
- First head attempt found a newly introduced sync opener reference to removed
  `_coro`. Fixed the sync facade to use lazy `.open()`; no motion had dispatched
  on the failed attempt. Added opener and scope regression tests.
- Final head checks: **4 passed** (trajectory convergence, hold, scope cancellation,
  controller restoration). XML: `/tmp/ringfour-hardening-20261013/handy-head.xml`.
- New portable inhibit integration check: **1 passed** on head. Trajectory,
  hold, tool and acquisition requests all rejected under simulated inhibit;
  inhibit state and controller scope restored. XML: `handy-inhibit.xml` beside above.
- Captured ephemeral agent audit under
  `/tmp/ringfour-hardening-20261013/handy-agent-audit` before removing container.
- New deterministic/protocol hardening module currently **38 passed**.

## Acceptance evidence so far (partial coverage, no release claim)

| IDs | Existing evidence | Remaining requirement |
|---|---|---|
| A01 | Unknown registry refuses startup (T1) | Complete descriptors/dynamic registration and T2 |
| A02 | Every motion gate tested (T1); Handy group gate (T3) | Full family/permission coverage at T2 |
| A03/A04 | Counted children and failed additional entry (T1) | T2 and more partial-step faults |
| A05/A06 | Retained iterator and explicit cleanup; async cancellation (T1) | Eager operations and T2 |
| A07/A08 | Missing/stale/NaN/residual false arrival (T2); separate clocks (T1) | Revision checks, mixed-unit integration, deployment evidence |
| A09 | Cancel/restore/unload/readback faults quarantine (T1) | T2/T3 fault injection and authorized recovery |
| A13 | Stale session rejected, dispatch revalidation (T1) | Epoch/principal binding and T2 |
| A16/A17 | Owner-only cancel/query; group-specific cancellation (T1) | Principal permissions and per-goal adapter footprints |
| A18 | Dropped updates retained and queryable (T1/T2) | Sequence/gap/overflow protocol cases |
| A20/A21 | Scope parity and double-failure evidence (T1) | All operation families through T2 |
| A29 | Unknown wire errors and structured evidence retained (T1) | Full domain/diagnostic migration |
| A35 | Removed misleading mixed-unit residual maximum | Remaining unknown-value/uncertainty contracts |

A10–A12, A14–A15, A19, A22–A28, A30–A34 and A36 are not claimed complete.
Neither A, B, C nor D is complete. Next work: strengthen dispatch/disconnect races,
then operation identity, lease/permission/reconciliation contracts and pruning.

### Final reviewed batch and handoff

- Fixed a further release race: an old scope's close cannot close or cancel work
  in a replacement scope. Close validation and release are atomic; task resources
  are bound to session IDs rather than reusable group names. Closed-scope records
  retain their owner and outcome in a bounded 1024-entry cache.
- Repeated cancellation no longer interrupts in-flight cancellation cleanup.
  Disconnect and shutdown cancel/drain tasks before controller restoration;
  new motion is inhibited during shutdown. Cleanup/drain timeouts quarantine.
- Group hold occupies the shared trajectory adapter until its bounded controller
  completion, so a hold cannot silently replace or be replaced by another goal.
- Added readback restoration for controllers originally unconfigured; conflicting
  group/controller footprints reject acquisition. Hold also validates bounds.
- Arrival diagnostic selection uses normalized per-joint tolerance rather than
  comparing raw metres with radians. Invalid durations, overflowed horizons and
  invalid geometry/velocity/position values reject before actuator dispatch.
- Connection loss wakes waiting clients with `ConnectionLost` while retaining
  the last known execution state: it does not mark a running operation done.
  Late reports cannot rewrite terminal outcomes on agent or client.
- Vendor publication now returns typed `DispatchReceipt`, with unverified
  completion, in sync and async. Backend topic/payload remain in integration
  audit. Parameter/publication failures do not release other acquisitions.
- Updated the dashboard guidance check to derive the test inventory from source,
  replacing the fixed 88-test assertion. Every new selectable test has complete
  operator descriptions. Inhibit integration belongs to Diagnostics.
- **Final offline result: 493 passed** (354 SDK/agent cases + 139 dashboard cases),
  2 existing dotted-watch deprecation warnings. Includes **52 new hardening
  cases**, of which six use real isolated protocol clients.
- **Final source-agent integration: 5 head checks passed**. Compatibility remains
  **57 robot passed / 4 known limit failures / 85 skipped**. No tolerance changes.
  Source hashes were verified unchanged during the integration run.
- The final return-type annotation cleanup was additionally checked by the real
  sync/async vendor protocol regression; it changes no robot execution behavior.
- Protocol/type references and course notebooks regenerated. `git diff --check`
  passed. No commits, pushes, image rebuilds or robot profile edits.
- **Runtime handoff:** temporary source agent removed; original Handy agent
  restored and reachable, simulated e-stop false. It still runs its original
  installed image. New behavior was tested from source, not installed into it.
- Durable evidence copied to
  [test-results/hardening-20261013](ringfour-sdk/test-results/hardening-20261013):
  XML results, stdout, agent audit, tested/current source manifests, original
  tracked diff and the simulator validation script. Original `/tmp` copies remain.

### Explicit remaining work

This is a reviewable implementation batch, **not completed v1 hardening**.
Remaining release work includes complete admission schemas/resource/permission
descriptors, model-bound configurations/trajectories, eager typed operations,
persistent fresh-signal guards, verified stop criteria/reports and authorized
quarantine recovery. Leases, principal authentication/permissions, command-ID
deduplication/expiry/epochs, reply-loss reconciliation and reconnect inspection
are still absent. Power/public-surface pruning, frame/artifact delivery and
measurement naming/unknown-value review remain open. T3 still needs a second
materially different robot and a known movement environment; T4 remains a
separate commissioning procedure.

Next implementation should finish the shared semantic adapter and admission
contract before migrating every public operation family. Do not reinterpret
these passing tests or the four recorded simulator failures as release waivers.


### Milestone A continuation — admission and non-group cleanup

- Expanded every registered descriptor to declare permission, logical resources,
  ownership/inhibit rules, schema and actual completion model. Every RPC now has
  an explicit field schema, including empty schemas for no-parameter methods.
  Spatial models and integration-specific bounds remain validated by handlers;
  descriptors do not claim model binding or physical footprints.
- Central dispatch validates schemas before handler execution, resolves vendor
  effects from the profile and records admission policy in command audit.
  Invalid numeric types, non-finite JSON, missing/unknown fields and malformed
  params cannot reach motion handlers. Fixed falsey list/null params previously
  becoming an empty object.
- **Permission declaration is not permission enforcement.** Authenticated
  principals and authorization remain milestone C work. No new unauthenticated
  recovery/reset endpoint was introduced to pretend that requirement is solved.
- Fixed teleop silently swallowing zero-command publication failures. Stop,
  end, disconnect, watchdog and shutdown now quarantine on failure; failures
  retain evidence and cannot produce a successful cleanup audit. Disconnect
  still continues other cleanup; agent shutdown now ends teleop sessions.
- Fixed navigation cancellation failure during in-flight submission escaping
  quarantine. Preserves `CleanupFailed` rather than wrapping it as no-path.
  Mobility stop independently attempts cancellation and zero publication,
  drains affected tasks and reports every failure.
- Fixed partial controller-effect bookkeeping: load/configure/switch may apply
  before their replies fail. Record intended effects before dispatch, then
  restore/read back prior state while preserving unrelated child acquisitions.
- Added 37 acceptance cases: 34 admission/non-group cleanup cases and 3 lost
  controller-acknowledgement cases. 21 new cases use real isolated Unix protocol
  listeners; the rest use recording adapters and controlled events.
- Test development exposed fixture setup mistakes (missing listener argument,
  AuditEvent accessed as dict); fixed before recording passing evidence. A live
  startup preflight also caught an undefined safety variable introduced in the
  teleop bootstrap wiring; fixed by constructing and sharing one SafetyLayer.
- First live attempt could not start: `/controller_manager/list_controllers`
  unavailable. Inspection proved `handy101-ros-dev-1` contained only `sleep
  infinity`; the simulator process had exited. The original installed agent had
  the same failure. The test runner restored its original container in finally.
- Restored simulation using the existing launcher in the existing Handy ROS
  container, explicitly `headless:=true world:=empty`, with the existing domain
  184 / Zenoh router. Did not rebuild images, alter profiles or touch other
  robots. This is a fresh empty-world environment, not continuation of the old
  scene. Simulator output: `/tmp/ringfour-hardening-a2-sim.log` in that container.
- Live revalidation and final combined offline evidence recorded below once
  complete. Artifacts are separate from the previous batch, in
  `ringfour-sdk/test-results/hardening-20261013-a2`.

**A remains in progress.** This continuation fixes concrete admission/rollback/
cleanup defects. It does not establish verified physical stop criteria,
authorized quarantine recovery, complete resource footprints or authenticated
permission enforcement; those cannot be inferred from controller readback.

#### Final continuation evidence / handoff

- **530 offline cases passed**: 391 SDK/agent + 139 dashboard. Two existing
  dotted-watch deprecation warnings. Includes 37 new cases in this continuation,
  21 of them real isolated Unix protocol checks. JUnit evidence saved.
- **5/5 reviewed Handy head integration checks passed** against final source.
  Complete source manifest remained unchanged during the integration run.
- Compatibility: **56 robot passed / 5 failed / 85 skipped**; additionally
  **139 dashboard passed** in recursive collection. Skips are the same previously
  documented eligibility/configuration groups. Five failures are four existing
  joint-limit violations plus no nonempty SLAM map in the fresh empty world.
  Empty maps now report `CompletionUnverified` with dimensions, rather than an
  internal shared-memory serialization crash. No tolerances were widened.
- Fixed the live inhibit test to submit valid per-method schemas; its previous
  combined payload tested malformed input rather than inhibit eligibility.
- Fixed a dashboard offline test that depended on the removed base101_arm robot
  profile: it now provides its own synthetic selected YAML/catalog. This keeps
  tests independent of installed robot workspaces.
- Added a bootstrap regression for shared teleop SafetyLayer and an empty-map
  regression proving no payload allocation occurs before unavailable evidence.
- Artifacts preserved in `ringfour-sdk/test-results/hardening-20261013-a2`,
  including failed pre-harness-fix evidence, final XML/logs/audit, manifests and
  the runner. Prior batch artifacts are preserved separately.
- Temporary source agent removed; original Handy agent container restored.
  Fresh headless empty-world simulation remains running. Other robot containers
  untouched. Original agent runs the installed image, not these source changes.
- Remaining A work is explicitly the admission enforcement/physical footprint
  gaps, stop-verification contract and authorized quarantine recovery; full
  principal authentication remains C. Do not describe A or v1 as complete.


### Milestone A continuation — enforced access, footprints, measured stop and recovery

- Added connection principals and **enforced** descriptor permissions before
  handler dispatch. Unix identity is Linux SO_PEERCRED, with agent UID operator
  authority and other UIDs observation-only. TCP defaults to observation;
  explicit `--trusted-tcp` grants legacy simulator/private-deployment operator
  authority, without claiming remote authentication/encryption. TCP never gets
  recovery permission. Welcome and connection audit report identity/grants.
  Inhibit reset and recovery use separate permissions. No RPC parameters grant
  authority. Synthetic TCP fixtures now opt in to their explicit trusted boundary.
- Added atomic acquisition footprints for joints/controllers and shared mobility.
  Same-owner navigation/teleop cannot overlap. Opaque vendor motion and arbitrary
  raw controllers reserve exclusive effects; raw scopes require readback joint
  claims matching the resolved group. Unknown parallel effects reject rather
  than being inferred from morphology. SLAM lifecycle writes reserve mobility
  dependencies and cannot preempt accepted navigation. Reservations release only
  their own token after completion/failure/cancellation.
- Added explicit optional `groups.*.stop_verification` policy: separate reviewed
  rad/s and m/s thresholds, dwell and finite per-check timeout. Unit types come
  from the live model. Missing units/thresholds never fabricate success.
- Shared verifier requires fresh finite position/velocity evidence, advancing
  source and reception timestamps for every joint over dwell, and consistency
  with position-derived velocity. Missing/stale/future/nonfinite/frozen/paused
  samples and moving feedback never verify rest. Uses a fake-clock test adapter
  for deterministic fault evidence, and ROS cache provenance in integration.
- Configured cancellation verifies measured rest before terminal cancellation;
  failed criteria produce `StopUnverified`, retained failed task state and
  quarantine. Configured scope cleanup verifies before **and after** restoration,
  so restoring an old controller target cannot conceal renewed movement.
  Legacy unconfigured groups retain acknowledgement-only cleanup, explicitly
  unverified; no threshold was invented for every robot or pushed into profiles.
- Added privileged local `safety.recover`: authenticated local identity, explicit
  permission, engaged inhibit, no active tasks/acquisitions, saved controller
  checkpoint and measured-stop policy are mandatory. Reattempt cancel, verify
  states/rest/states; only then clear the requested group. Inhibit stays engaged,
  other quarantine and previous failed reports remain intact. Mobility/vendor
  recovery and persistent restart checkpoints remain unsupported.
- Releasing the simulated inhibit rejects while quarantine remains. This reset
  cannot substitute for verified recovery. Measured evidence is explicitly
  **uncommissioned**, not a physical/hardware safety guarantee.
- Tightened navigation/place/localization geometry schemas: numeric booleans/
  strings, nonunit quaternion, mixed place/pose payloads and unknown fields reject
  before motion dispatch. Acquisition footprints cannot change under an old ID.
- Harness motion/fault fixtures inspect connection grants and report missing
  authority as a setup requirement, preserving old-agent fallback behavior.
  Updated their offline fixture tests to supply the new Welcome argument; one
  fixture regression was caught before final results and corrected.
- Type/protocol references regenerated. Canonical Handy/Grove profiles untouched.
  Live validation uses a copied Handy YAML with **head only** measured-stop
  thresholds: 0.01 rad/s, 0.2 s dwell, 2 s per-check timeout. These are reviewed
  simulator test inputs, not commissioning defaults. Source agent explicitly opts
  into trusted TCP for this authorized simulator test. Original installed agent
  is always restored by the runner. No image builds, commits or pushes.
- Artifact folder: `ringfour-sdk/test-results/hardening-20261013-a3`. Live head
  evidence is reused for the final compatibility-only replay because agent/SDK
  source hashes are unchanged; the replay corrects only an offline fixture test.

Final results and runtime handoff are recorded below when verification finishes.
A is still not a v1 completion claim: verified stopping is opt-in for groups;
nav/teleop/tool/vendor operation-specific measured-stop contracts, full model/
guard admission and distributed lifecycle still need their specified evidence.


#### Final access/stop/recovery continuation evidence

- **576 offline cases passed**: 436 SDK/agent + 140 dashboard, 2 existing
  dotted-watch deprecation warnings. This continuation adds 46 cases: 41 stop/
  recovery/access/resource cases, 4 invalid spatial payload cases and 1 harness
  permission-eligibility case. Thirteen new isolated protocol cases cover access,
  invalid geometry and cancellation/recovery composition; exact test nodes
  and outcomes are in the XML. No ROS/hardware used for offline evidence.
- **5/5 reviewed Handy head checks passed with measured-stop policy enabled**.
  Audit confirms successful head cleanup includes passing measured-rest evidence
  before/after restoration. Agent/SDK source hashes remained unchanged and still
  match the checkout. No repeated head motion after the offline fixture fix.
- Final compatibility: **56 robot passed / 5 failed / 85 skipped** plus
  **140 dashboard passed**. Failed cases remain the four known joint-limit
  violations and unavailable occupancy grid in the empty world. No tolerances
  widened and no fake map/goal introduced. One intermediate offline fixture
  failure (new Welcome argument) was fixed; final suite is green.
- Original `handy101-agent-1` restored, reachable as `mr-handy`, simulated inhibit
  false. Fresh headless empty-world simulation remains running. Temporary source
  agent removed. Current installed image is not upgraded by this work.
- Current behavior documentation, test-environment deployment notes and generated
  type/protocol references updated; `git diff --check` passes. Durable artifacts
  include final JUnit, audit, source manifest, temporary profile/policy and both
  complete/compatibility-only validation runners. Prior batch artifacts preserved.
- **Remaining A boundary:** verified stop contracts for mobility/teleop/tool/
  vendor families and strict treatment of unconfigured groups remain open.
  Group stop/recovery is implemented and tested, not a universal physical stop
  guarantee. Remote identity, restart checkpointing, leases, model revisions and
  persistent guards retain their separate B/C work. No milestone is marked done.

### 2026-10-13 — Full A continuation: strict motion-family contracts

User requested completion of **full A**, rather than another optional-policy
batch. Implementation is in progress; final evidence/status will follow here.

Decisions and changes:

- Production/bootstrap and directly constructed managers default to strict
  measured stopping. A profile omission rejects before controller switching or
  motion publication. One reviewed `safety.stop_verification` policy supplies
  distinct rad/s and m/s limits, dwell and bounded timeout (maximum 15 seconds),
  inherited by groups with optional overrides. No robot-name/morphology branch.
- Existing synthetic transport/acknowledgement-only fixtures explicitly pass
  `allow_unverified_stops=True`; this is an internal compatibility/testing input,
  not a production CLI option. Welcome advertises strict mode. These fixtures
  do **not** establish measured-stop acceptance; separate strict cases do.
- Body stop evidence includes all six twist axes, full 3D position/orientation,
  advancing source and receive clocks, unit quaternions and independent
  pose-derived velocities. ROS caches atomic odometry receipt provenance.
- Teleop explicit stop/end, watchdog, disconnect and shutdown use shared verified
  stopping. Concurrent cleanup waits for the in-flight verification before
  releasing the acquisition. Failed verification retains quarantine.
- Navigation preflights/rechecks its contract, cleans up lost submission replies
  and wait errors, and checks measured rest plus framed 3D target residuals before
  reporting arrival. Cancellation waits for terminal controller result as well
  as acknowledgement. No generated navigation goal or invented map introduced.
- Tools retain unique acquisitions until completion/cleanup, recheck finite fresh
  feedback, verify residual/rest, and hold the current measured position on
  failure/revocation. Unverified holding retains tool quarantine. SDK tool call
  uses a finite 60-second wait rather than the generic five-second RPC budget.
- Motion vendor actions need a declared observable resource (mobility or a
  resolved group) and explicit `stop_arguments`. No guessed stop command.
  Disconnect/shutdown stop and verify all owned vendor actions; uncertain
  publication also takes the stop path.
- Group dispatch rechecks usable feedback. Velocity-role stopping publishes
  zero joint velocities rather than current positions. Strict unknown raw
  controllers reject before effects; declared raw aliases resolve their roles.
  Follow cannot use a nontrajectory role. Separate lifecycle mutex keeps long
  controller cleanup away from the shared admission mutex.
- Stop watches latch inhibit before scheduling asynchronous shared cleanup;
  operator inhibit uses the same cleanup path for all existing motion families.
  Failed cleanup cannot become a successful halt audit record.

Verification in progress: new strict deterministic family cases, complete offline
regression, source-agent Handy simulation checks, generated references and docs.
Canonical robot profiles and installed images remain untouched. B/C public API,
remote identity, leases, model revisions and persistent guards are separate work.


#### Full A final evidence and completion boundary

**Milestone A is complete for the existing software guarantees.** This does not
mean all acceptance IDs in the v1 specification or milestones B/C/D are complete.
The late-acceptance handoff is in-process adapter cleanup, not durable submission
reconciliation or a remote operation query API.

Additional final fixes/evidence:

- Shared `ActionSubmission` retains uncertain acceptance. Stop waits for
  acknowledgement **and terminal controller result**. If acceptance arrives
  after timeout/revocation, cancellation is attempted on that exact handle in
  a separate worker; unresolved stopping remains quarantined. No late goal can
  silently become a successful cleanup report.
- Closing an idle disjoint trajectory scope does not cancel another group's
  global adapter goal. The running child retains ownership. Strict regression
  tests cover this and inhibit arriving during controller preparation.
- Controller preparation rechecks eligibility before activation, and ROS action
  availability waits are followed by another eligibility check before send.
- Tool controllers must be active and claim exactly the declared tool joint.
  Background stop-watch cleanup is retained and joined during agent shutdown.
- **636 offline cases passed**: 496 SDK/agent + 140 dashboard, 2 existing watch
  deprecation warnings. This continuation adds 60 strict motion-family cases;
  those never opt into legacy acknowledgement-only fixtures.
- **5/5 reviewed Handy head checks passed** against final checkout source.
  Eight successful controller cleanup audit reports all include measured rest
  before/after restoration. **Stationary teleop stop and end both passed** full
  3D measured-body verification (six progressive samples each, 0.2-second dwell).
  No nonzero body command or navigation goal was dispatched for that check.
- Final read-only compatibility: **56 robot passed / 5 failed / 85 skipped**,
  plus 140 dashboard passed. The same four out-of-model joint limits and missing
  SLAM grid remain visible. No tolerance widening, fake goals/maps or changes to
  canonical Handy/Grove profiles. Empty-world limitation remains documented.
- Source manifest matches the final checkout and stayed unchanged during the
  final live run. An earlier live pass was retained; repeated motion was limited
  to validating subsequent action handoff/admission fixes, not chasing tolerances.
- Artifacts: `ringfour-sdk/test-results/hardening-20261013-a4/` contains final
  JUnit, exact runner, temporary shared/head policy, conformance settings, logs,
  audit, source manifest and `SUMMARY.json`. Prior batches are retained.
- Original `handy101-agent-1` restored and reachable as `mr-handy`, inhibit false;
  temporary source agent removed. Headless empty-world simulation remains on.
  Installed image was not upgraded. Other robot/hardware containers untouched.
- Implemented-contract docs, deployment notes and generated references updated.
  `git diff --check` passes. No commit, push, image build or deployment.

A acceptance trace (software scope):

| Gate | Evidence |
|---|---|
| A01 classification | `test_unclassified_rpc_rejected`, descriptor inventory validation, required vendor effect profile validation |
| A02 inhibit | Production protocol matrix, live `test_inhibit_rejects_all_group_motion`, strict preparation/dispatch rechecks |
| A03 children | Counted-acquisition tests and strict disjoint-scope cleanup preserving a running child |
| A04 failed entry | Partial effects/lost-reply rollback tests, strict inhibit-during-preparation rollback |
| A05 iterator lifetime | Retained-iterator tests for both sync/async facades |
| A06 bounded explicit cleanup | Scoped close/caller-cancellation/repeated-cancel tests, finite controller/task/verification budgets |
| A07 complete feedback | Protocol arrival faults plus strict joint/body/tool admission and verification faults |
| A08 false arrival | Residual/type/unit tests and strict navigation production-task false-arrival case |
| A09 restoration quarantine | Faulted controller readback/restoration, strict tool/vendor/teleop failures, retained quarantine and privileged group recovery |
| A24 verified stopping | Deterministic native-unit/dwell/provenance faults, all motion-family cleanup, live head and stationary-body evidence |

Still separate: eager public operations/results, typed model-bound command values,
observations and persistent guards (B); finite leases, remote authenticated
identity, durable reconciliation/checkpoints and complete recovery (C); shared
second-robot release evidence, course migration and physical commissioning (D).
Mobility/tool/vendor quarantine has no privileged recovery RPC yet; reconnect or
inhibit reset cannot clear it. Conservatively blocking it is intentional.

### 2026-10-13 — Authorized Handy fixture commissioning and milestone B

User explicitly requested: (1) fix Handy joint limits upstream, (2) select targets
on its existing world/map, (3) rebuild the agent, (4) start Docker, (5) implement
B with current documentation. Handy is the controlled integration fixture;
shared agent/SDK/test code must continue to work from arbitrary robot profiles.

- Direct MuJoCo reproduction finds exactly the four reported joint violations,
  independently of SDK/harness. Harder joint-stop constraints with native-unit
  activation margins remove the settled violations without widening URDF limits
  or clipping feedback. Constraints remain compliant physics, not a hardware
  guarantee. Regression and live evidence will follow.
- Existing `robocasa_home` includes a clear 12 × 12 m test room; use it instead
  of the empty floor so real lidar/SLAM can supply occupancy evidence. Candidate
  navigation targets must be in observed connected free space with clearance,
  then accepted by the planner. Fixture coordinates stay in fixture settings.
- Preserve earlier dirty changes in both repositories. New agent image/deployment
  and Handy simulator restart are now explicitly authorized; other robots stay
  untouched. No commits or pushes requested.
- B work will migrate sync/async together, update public examples/reference/course
  with implementation, and record remaining release gates explicitly.

#### Handy commissioning evidence before B

- Full upstream description/FK/mass/sensor/actuator/physics/home checks pass.
  Joint stops use positive time-constant settings for arms/lifts and a reviewed
  direct stiffness/damping setting for head tilt; a globally applied aggressive
  setting was rejected because it failed settling/stability checks. Endpoint
  checks preserve the original 1e-5 tolerance and raw qpos.
- Replaced empty floor with existing `robocasa_home`. Pure rotation did not
  produce adequate SLAM coverage; measured short translations within the known
  clear test room supplied genuine additional scans. The selector's one-metre
  clearance policy was retained. No unknown cells were marked free.
- Two target poses selected from connected known-free occupancy cells and accepted
  by live Nav2 ComputePathToPose (32 and 41 path points). Their coordinates and
  snapshot/recipe are in `hardening-20261013-b/handy-home-targets.json`.
- Saved `handy-home-fixture-v1` through the real SLAM service. Fixture settings
  live upstream at `mr_handy/profiles/mr_handy.testing.yaml`, independent of
  shared robot-agnostic SDK/harness logic. Goals require map/world revalidation
  when environment changes; they are not defaults for arbitrary robots.
- Built `ringfour-agent:handy-hardening-a` and `latest` from A source and recreated
  only Handy's agent service. Trusted TCP is explicit in Handy Compose. Canonical
  Handy profile now contains its reviewed shared measured-stop policy. Other
  robot containers were untouched. B image will be rebuilt after source migration.

#### B implementation started: operation and model-value producers

- Original Handy failures verified fixed on the installed hardened agent:
  **6/6** selected live joint-limit/map checks passed (`handy-repaired.xml`).
  Final upstream simulator scene also passed full endpoint/settling/home checks;
  relaunched the home scene and loaded/localized the saved fixture checkpoint.
- B now has immutable native-unit `JointConfiguration`/`JointTrajectory` values,
  identity/coverage/bounds/timing validation, named configuration builders and
  IK returning model-bound configurations. Group-specific model caches are
  distinct from the composed model.
- Added eagerly accepted generic sync/async `Operation`, immutable snapshots,
  typed result/progress producers, cancellation receipts, result-wait versus
  action timeout behavior, explicit bounded scope cleanup and observation-only
  update streams. Current source is under validation; this is **not B complete**.
- Added agent typed-trajectory admission/dispatch binding, session revision
  checks, terminal cancellation stopping evidence and update sequences. Generic
  code depends on discovered identities/units, not Handy coordinates or names.
- New canonical navigation/trajectory entry points are being migrated together.
  Legacy surfaces, observations/guards, route/tool operations and course/reference
  migration still need completion and tests before any B completion claim.
- Generated protocol/type references updated with source types immediately.
  Installed container currently runs the proven A image; a B image/redeployment
  comes after B source validation, not before.

#### B first contract regression and fixture revalidation

- Migrated IK regression fixtures to explicit robot identity and named
  configurations. Removed assumptions that IK returns an unbound positional
  list. Cancelled convenience execution now raises `OperationCancelled` with
  cleanup evidence, rather than returning an old trajectory status.
- Corrected a missing `CapabilityNotSupported` import. Updated no-ROS admission
  fixtures to use strict stopping so the new feature handshake cannot bypass
  their rejection/estop assertions through compatibility mode.
- Added focused offline tests for result-wait timeout leaving work running,
  iterator close ending observation only, idempotent verified close, convenience
  timeout retaining stopping evidence, rejection of missing cleanup evidence,
  immutable configuration values, mixed model/unit rejection and invalid times.
- Retained task status now includes agent-computed `report_age`; SDK snapshots
  add elapsed local receipt age. Fetching old progress cannot make it appear new.
  This is report provenance, not full sensor measurement provenance. Delayed
  lower-sequence updates cannot overwrite newer cached state.
- Async convenience cancellation preserves the original cancellation and attaches
  cleanup failure through the shared scope helper. Full measurement provenance,
  persistent guards, route/tool operations and public API/course migration remain
  pending. **B remains in progress; its source is not installed in Docker.**
- Revalidated both home-world targets after simulator restart, saved-map loading
  and spawn localization: live Nav2 planner succeeded with 79 and 60 path points.
  Evidence: `hardening-20261013-b/handy-home-targets-from-reset.json`.
  These are planning checks, not claims of completed navigation execution.
- Final first-slice B offline regression: **640 passed**, with two existing
  deprecated-watch warnings. Generated protocol and type reference are updated.
  Full output: `hardening-20261013-b/b-source-regression.log`.

### Completing B — active implementation

- Added immutable `Measurement` with separate agent-computed source/receive ages,
  declared ROS clock domain and explicit unavailable/stale/invalid outcomes.
  Header-less signals retain unknown source freshness; no receipt-as-source fiction.
- Discovered local signal references build typed conditions. Persistent guards
  verify fresh safe entry, latch independent inhibits, stop/drain work on trip,
  require fresh safe reset and drain before removal. Operator e-stop, other guards
  and quarantine remain independent. Disconnect removes guards only after cleanup.
- Finite routes use one accepted operation and retain mobility ownership across
  waypoints. Each waypoint requires measured arrival/rest before advancing.
- Tool positioning now uses accepted tasks, typed measured-position/residual/rest
  results and bounded cancellation cleanup. Holding/contact remains a separate
  inference, not a positioning completion claim.
- Model changes fence submissions and stop/drain accepted work/sessions before
  clients can use a new model. Inhibit remains if cleanup fails.
- Initial guard/provenance/strict motion regression: 100 passed. API pruning,
  references/course migration and live B commissioning are still active work.

- Public migration removes lazy navigation/trajectory iterators and raw-string
  navigation entry points, relocates backend/controller mappings and raw control
  to explicit integration diagnostics, exposes semantic control-mode inventory,
  and replaces remote properties with matching sync/async `get_*` methods.
  Deprecated group/gripper/watch aliases and placeholder handles are removed.
- Sync power methods now return values and preserve unknown power source as None.
  Named forward-speed/IMU-angular-rate helpers state the actual measured quantity.
- First B live probe found nested acquisition of a non-reentrant ROS cache lock
  during guard entry; no motion was submitted. Replaced that cache lock with an
  RLock so value/provenance can be read atomically through existing getters.
  Also corrected live group revision lookup to pass resolved group metadata.
- Repeated pending-status queries now preserve authoritative sequence zero rather
  than inventing a local counter that could suppress real later reports.
  Added a dedicated regression. Build/revalidation continues; no B completion yet.
- First live B guarded head motion/return-to-start passed. Tool hold at the
  closed stop correctly failed StopUnverified and retained `tool:right`
  quarantine. Readback showed position chatter ~0.0011–0.0016 rad but raw velocity
  spikes ~0.12 rad/s: this is a simulator rest problem, not false harness failure.
- Compared stop margin/direct-stiffness and actuator damping candidates offline.
  Increasing servo damping ratio to 20 passed isolated closed/open checks but
  failed Handy's existing mixed-posture settling regression; rejected/restored.
  Testing passive gripper damping next. Native limits/tolerances remain intact.
  Added an upstream full closed/partial-open/full-open native-rest check.
- Added explicit tool-resource recovery to the existing privileged recovery
  path; it must verify the tool joint and controller checkpoint while inhibited.
  This does not grant trusted TCP authenticated recovery authority or implement C.
- Passive gripper damping 1.0 brings raw tool velocity below 0.01 rad/s without
  altering the force-limited position servo. The existing mixed-posture test
  exposed independent left/right wrist-roll chatter in frame-only couplers.
  Reflected roll inertia candidates 0.0001/0.001 kg m² reduced but did not pass
  the unchanged 0.02 rad/s settling check; testing 0.003 next. Native mass/FK,
  geometry and limits are preserved. No failed full-physics run is a pass.

#### Handy fixture now passes full upstream physics

- Full checks passed after the selected correction: gripper passive damping 1.0;
  wrist-roll couplers passive damping 1.0 + reflected inertia 0.01 kg m² + stop
  impedance 0.99. Original URDF limits/effort/control bounds and SDK tolerances
  remain unchanged; wrist stop activation stays 0.001 rad inside those bounds.
  No feedback clipping. Source mass/FK/sensors, original 4-second mixed settling,
  gravity/both endpoint bounds, wheel-floor physics, full furnished home and the
  new closed/partial/full-open raw gripper-rest check all pass.
- Rejected stronger servo damping, inertia-only variants and direct-stiffness
  wrist stops are preserved in the log; they were not deployed or called passes.
- Handy Nav2 goal checker was inherited at 0.25 m/rad while its SDK arrival
  contract is 0.1 m/rad. Added a Handy-local 0.05 m/rad/speed overlay, using a
  reusable optional base101 `controller_overrides` launch argument. Base101
  defaults are unchanged. Local Handy development Compose mounts the reviewed
  launch file until its dependency image includes that extension.
- Enabled the agent's local Unix socket for explicit same-UID recovery authority;
  trusted TCP remains operator-only. Preparing fresh simulator commissioning
  after the physics change; prior failed-tool quarantine is not labelled a
  successful recovery. Restart-durable quarantine remains milestone C work.

#### B final deployment and live findings

- Recreated only the Handy services with final verified physics, shared Nav2
  launch override and privileged local Unix socket. This commissions a new
  simulator boot after an independently verified physics correction; it is not
  evidence of privileged recovery or restart-durable quarantine (milestone C).
- Both fresh measurements, guarded head trajectory, wait-timeout/observer
  independence, explicit cancellation/close, convenience timeout cleanup,
  threshold trip/drain and subsequent inhibition passed on the deployed agent.
  Evidence is persisted after every case in `live-acceptance.json`.
- Closed/partial tool motion passed. Full travel exposed an agent budget bug:
  reaching the target incorrectly reused the stop-verification timeout (2s).
  Introduced an independent positive finite `execution_timeout` (default 30s)
  in matching sync/async tool producers. Stop policy/feedback/residual tolerances
  remain unchanged. Rebuilding before repeating tool acceptance.
- Both existing furnished-home targets have real Nav2 plans after reset. Actual
  navigation exposed post-controller arrival residuals outside unchanged 0.1m /
  0.1rad thresholds on initial attempts. The SDK correctly refused verified
  success. Diagnostics show aggressive near-goal turning/translation; checking
  a slower Handy-only controller envelope, not weakening SDK arrival assertions.
- Full SDK/agent/dashboard offline check: 643 passed before the independent tool
  budget addition. Final regression and navigation acceptance still pending.

- Final independent tool-budget image passes all head/guard/lifecycle/tool cases,
  including native full open and close. Updated offline suite: **648 passed**.
- Migrated robot group metadata/watch checks: **22 passed**. Read-only robot
  compatibility: **61 passed / 85 skipped / 110 deselected**. Skips are explicit
  missing capabilities or disabled/review-required motion; no assertion weakening.
- Nine regenerated notebooks match source; all 47 code cells parse. Compile checks
  passed for examples/missions/harness. This is static validation, not notebook
  execution. Fixed one old integrator example importing the moved agent schema.
- Slower MPPI resolved one measured arrival but inherited progress checking
  aborted a route (0.5m/10s threshold). A follow-up attempt timed out cleanly.
  Adjusted Handy-only progress to 0.1m/0.2rad/20s and translation-before-final-yaw
  priorities (GoalCritic weight10, GoalAngleCritic distance0.1m). One more bounded
  acceptance is active. RPP is installed and its official Jazzy configuration was
  reviewed as an alternative, but has not been selected/deployed.

- Final MPPI attempt reached the first goal but continued orbiting at the second.
  Explicit operator halt was requested; its child task reported CleanupFailed
  rather than invented stopping success. Persisted the failed attempt log. This
  is not a clean cancellation acceptance. Stop/body thresholds were not changed.
- Replaced only Handy's MuJoCo Nav2 FollowPath plugin with installed Jazzy
  RegulatedPurePursuit (0.15m/s, 0.25rad/s in-place alignment, collision detection
  retained). Added explicit runtime package dependency and rebuilt the control
  config. New fixture boot is commissioned separately; it is not evidence of
  durable quarantine recovery across restart (C). Old failure/audit logs remain.
  SDK/profile thresholds remain 0.1m/0.1rad arrival and native stop policy.
- Acceptance driver now appends typed exceptions/details to live-failures.jsonl
  and persists successful results incrementally. Alternative-controller live
  acceptance is pending, not presumed by successful startup.

- RPP reaches the first endpoint and the next endpoint's position without the
  MPPI orbiting. The route correctly failed measured final yaw at 0.10264rad
  against unchanged 0.1rad SDK tolerance (position residual 0.03431m).
  Tightened the controller-only yaw target to 0.02rad and slowed in-place rotation
  to 0.15rad/s with 0.2rad/s² angular acceleration. No SDK/profile tolerance edits.
  The exact rejection/details are retained in live-failures.jsonl.
- Unauthorized typed trajectory with a valid payload/model and deliberately
  unacquired session ID rejects correctly (live-admission.xml, 1 passed), rather
  than testing schema rejection through an obsolete raw payload.

- Migrated head qualification/admission checks now pass **10/10**; two tool/contact
  checks are correctly not applicable to head. The first run caught a brittle
  legacy test matching the word "session" in an ownership error. It now checks
  the typed `SafetyViolation.reason == "motion lock held"` and still verifies
  acquisition succeeds after release. No ownership requirement was weakened.
- Tool qualification/restoration now pass the configured tool timeout to both
  the convenience wait and the independent target-execution budget, allowing
  slower profile-driven tools without using a hardcoded robot travel assumption.

- Final slower RPP acceptance passes: first goal residual **0.06052m / 0.01762rad**;
  finite route arrivals **0.01172m / 0.01243rad** and **0.03172m / 0.03098rad**.
  Every arrival retains independently verified body rest. Actual-motion cancel
  returns a receipt and verified cleanup (0.7537s observed verification, 0.2s dwell).
  Values are evidence from this simulation, not physical timing guarantees.
- Async acceptance driver's first invocation used a sync constructor spelling and
  failed before connecting/motion. Corrected it to the documented
  `async with await Robot.connect(...)`; original driver exception remains in
  the append-only failures file. Async execution acceptance is running now.

- Async head trajectory, Python cancellation with joined cleanup, and tool
  positioning pass. Async navigation's 120s convenience deadline expired while
  completing final heading at the second target (position was near the goal).
  It returned OperationTimeout with verified body stop and no cleanup error;
  exact details are retained. Slower RPP heading alignment needs a fixture timing
  budget consistent with its commissioned speed. Handy's reviewed navigation
  timeout is now 180s (route wait is per-goal budget × waypoint count), not a
  shared SDK default or arrival/stop tolerance change. The acceptance driver
  reads that YAML and selects the furthest reviewed endpoint for async motion.
- Rewrote SDK README to the actual repository layout/install/current operations,
  observations and guards. Removed claims that sensing/autonomy are upcoming,
  obsolete robot-specific harness startup, old spec-as-current links and paths.

#### B completion evidence and boundary

Milestone **B software and Handy simulator acceptance are complete**. C/D and
physical commissioning are not complete or implied. No commit/push was made.

- Final full SDK/agent/dashboard regression: **650 passed in 9.66s**. Added tool
  recovery cases proving it verifies the tool joint, rejects moving evidence,
  preserves group quarantine and leaves the inhibit engaged.
- Live group metadata/watch contract: **22 passed**. Reviewed head qualification
  and inhibit/admission/controller lifecycle: **10 passed / 2 inapplicable**.
  A separate unauthorized typed trajectory case passed. Native joint-limit/map
  checks during movement: **6 passed**. Read-only compatibility remains
  **61 passed / 85 skipped / 110 deselected**; no skipped check is counted as pass.
- Sync driver passes measured head/return, safe and unsafe guard entry, real guard
  trip/cleanup/inhibition, observer/result-wait independence, explicit close,
  convenience timeout cleanup, native closed/partial/full tool positions,
  reviewed navigation, two-waypoint route and actual-motion cancellation/rest.
- Async driver passes head/return, tool positions, Python cancellation with joined
  cleanup and actual-motion navigation to the furthest reviewed endpoint. Final
  async arrival residual **0.05235m / 0.03212rad**, verified 0.2s body-rest dwell.
  Initial constructor error and navigation deadline failure remain in the
  append-only failure evidence. Fixture navigation wait budget is explicitly 180s.
- Final agent image:
  `sha256:18eb7a18c3f72fc9801345b09d54d7ea726898a2422bd8627d528d38e5e81570`.
  Hash comparison proves **58 SDK + 36 agent Python files** exactly match deployed
  source. Model SHA:
  `1f9dc22c6dafd00265db3c8e8b8a2d85ea5dfab9db27909417eedd575cb7e219`.
  Fixture is existing `robocasa_home`, saved map `handy-home-fixture-v1`.
  `deployment.json` retains profile/config/world/map/source hashes and advertised
  policy; `deployed-python-hashes.json` records the exact runtime comparison.
- Full upstream physics checks pass without widening joint/actuator/effort bounds
  or native stop/settling assertions. Jaw damping/armature and wrist-roll damping/
  armature/stop impedance suppress simulation chatter. Handy-only RPP replaces
  inherited MPPI: 0.15m/s translation, 0.15rad/s rotation, 0.2rad/s² acceleration,
  0.05m/0.02rad controller goal thresholds, 0.1m/0.2rad/20s progress checker.
  SDK/profile arrival remains 0.1m/0.1rad; stop remains 0.001m/s, 0.01rad/s,
  0.2s dwell / 2s verification timeout. Failures were never turned into success.
- Authoritative API, generated protocol/types, README, progress/B pages, course
  and examples are updated. Nine notebooks match their generator and all 47 code
  cells parse. Examples/missions/live modules compile/parse. Notebooks were not
  executed end-to-end; experimental contact missions remain explicitly labelled.
- Evidence lives in `ringfour-sdk/test-results/hardening-20261013-b/`, including
  typed incremental results, failed attempts, JUnit reports, physics/build logs
  and final deployment hashes. No remaining B source/migration/live gate.

Remaining boundary: only Handy has B live simulation acceptance. C needs leases,
reply-loss reconciliation, restart/epoch/durable-result behavior and the remaining
recovery/identity work. D needs materially different robot fixtures and physical
commissioning where promised. Local Handy Compose depends on a reviewed optional
Nav2 launch extension mounted from the neighbouring base101 checkout; ship that
upstream dependency change before removing the mount. A simulator restart after
changing physics/controller is a fresh fixture boot, not verified recovery of old
quarantine. All successful StopReports still say `physical_stop: uncommissioned`.


### 2026-11-07 — XLeRobot profile, finite lease increment and native NET payloads

- Migrated the standard XLeRobot simulator profile to groups after read-only
  controller/topic inspection: left/right JTCs each claim six joints including
  their jaw; head claims two. No independent tool or Cartesian tip is claimed.
  RTAB-Map is not advertised as slam_toolbox. Missing stop-verification policy
  deliberately keeps strict motion unavailable; no thresholds were guessed.
- Built native ARM64 agent images, adding Cyclone DDS for domain 185. An initial
  temporary agent resolved 6+6+2 joints, received odometry and reached ready.
  It was removed when Compose became the launch path. Later user authorization
  to start the testing environment started only the Compose agent; the simulator
  was not restarted. Native Mac RGB and depth reads passed through NET. No
  motion was commanded.
- Added opt-in finite connection leases and random boot epochs. IDs bind the
  authenticated/deployment principal and connection. Expiry revokes dispatch
  and acquisition before cancellation/controller cleanup; repeated hello cannot
  replace a lease. SDK heartbeats renew authority only, never drive intent.
  No unconfigured profile claims finite-lease coverage. C remains partial.
- Implemented Transport.NET for raw binary arrays, negotiated per connection.
  New TCP clients select NET, Unix clients select SHM; explicit transport selects
  only advertised support. Sync/async sensing and map decoders share validation.
  NET does not replace JSON-RPC commands and adds no network motion permission.
- Native Mac tests: 20 focused NET/lease contracts passed, including real TCP
  RGB/depth/cloud/lidar/map/streams with SHM allocation forbidden. Linux SDK
  regression: 493 passed, 1 optional-kinematics module skipped. ROS pytest plugin
  autoload must be disabled for isolated SDK collection in the release image.
  Earlier Mac full-suite Linux-only failures and initial test-runner collection
  skips were not interpreted as passes. Dashboard changes made concurrently by
  the user are preserved and are outside this SDK regression claim.
- Simulator Compose/Pixi integration adds separate agent image/state, loopback
  observation endpoint, agent lifecycle tasks, native editable SDK environment
  and optional container SDK execution. User owns the stack restart.
- Remaining: lost-reply/idempotent submission reconciliation, durable restart
  records/quarantine and recovery/identity work (C); live second-configuration
  conformance and physical commissioning where promised (D). No C/D completion
  or physical stopping claim is made by the new transport/lease tests.

#### Final environment and performance evidence

- Final image `ringfour-agent:xlerobot-hardening` is
  `sha256:858fea9032e0bd9b13a4161673106abe5e33a405d8a188926a81066ed5650218`.
  All 95 installed SDK/agent Python files exactly match working-tree sources.
- Native Pixi SDK is installed editable and reads 640x480 uint8 RGB and float32
  metric depth from the separate Linux agent. Default TCP retains observe-only
  authority. Testing console is localhost:8766; existing 8765 was left untouched.
- Linux SDK regression after final source changes: 493 passed, one optional
  kinematics module skipped. New test operator descriptions were added after a
  real dashboard catalog check caught their absence; catalog and inspection now
  return HTTP 200. No dashboard-wide regression is claimed for concurrent edits.
- Profiles and raw reports are retained locally in
  `ringfour-sdk/test-results/hardening-20261107-xlerobot-net/`, with deployment
  source hashes and summary.json. This directory follows the existing ignored
  test-output convention. These November records do not recreate missing October
  evidence or establish C/D completion.
- Short read-only transport profiling found lossless WebSocket deflate expensive
  for float depth. A separate client with compression disabled improved median
  RGB/depth/synced reads from roughly 13.9/71.9/80.6ms to 1.46/1.86/2.83ms.
  Unique RGB streaming remained about 13.8-13.9fps. Fast polling repeats cached
  sensor frames and must not be presented as camera FPS. Compression defaults
  have not been changed by the profiling experiment. CPU/memory snapshots were
  taken at different phases, not a controlled idle-versus-load comparison.


### 2026-11-07 — Distributed lifecycle continuation and portable fault suite

- Finite lease renewal now has a priority dispatch lane independent of slow
  observation calls. A live seven-second missing-transform wait returned its
  normal timeout while the connection remained healthy; the earlier serialized
  dispatcher incorrectly let renewal starve and disconnect.
- Durable quarantine checkpoints preserve unresolved stopping failures across
  agent upgrades. Local Unix recovery verifies mobility rest and keeps inhibition;
  trusted TCP cannot recover. Feedback-loss testing stopped the joint-state
  broadcaster, retained failed head cleanup, restored feedback, and verified
  privileged recovery without resuming old work.
- SDK Linux regression before restart-recovery additions: 509 passed, one optional
  kinematics skip. Shared head acceptance: 10 passed, two not applicable. The
  corrected killed-base-client test failed the configured two-second stop criterion
  and quarantined mobility. The criterion was not increased to obtain a pass.
- Seven portable execution-hardening checks now live in the selectable diagnostics
  suite. Robot-specific fault/restore/recovery commands reside in reviewed scenario
  data, not test code. The catalog/conformance checks passed 58 offline cases.
  The first shared live batch was blocked by an existing restart inhibit: two
  non-motion checks passed, five motion setups failed with five teardown errors.
  That blocked attempt is not motion acceptance and was not imported into history.
- Agent recreation exposed ros2 run signal forwarding: the old Docker entrypoint
  left its child agent without graceful shutdown and retained the active boot
  marker. The entrypoint now execs the actual console script. Explicit restart
  recovery cancels orphan goals, verifies a reviewed controller baseline and
  measured rest, keeps inhibition and clears only restart. Its durability/recovery
  module passes 17 offline cases, including mismatch, cancellation, moving feedback
  and a baseline changed during verification. Live restart acceptance is pending.
- XLeRobot's simulation-only profile now declares all five loaded controllers as
  active for restart recovery, based on live ros2_control inspection. This is not
  physical commissioning or a recovery policy for unknown adapters/model changes.
- C remains partial: submission IDs/deduplication/conflicts, lost-reply queries,
  validity/retention/epoch retry semantics and durable terminal reconciliation
  remain required. No interrupted physical execution is automatically resumed.


#### Restart acceptance and cancellation-during-verification repair

- Image `ca82defb8de8b80675a676933299e686f92c46aa3b4dce605ac349ef6db45f85`
  preserved the existing restart quarantine. Authenticated local recovery cancelled
  the three trajectory channels and Nav2 channel, measured body/left/right/head
  at rest, and retained inhibition. Docker stop reached graceful shutdown, exited
  zero and committed `active_boot: false` with no quarantine. Starting only the
  agent produced no new restart quarantine; the simulator was not restarted.
- The shared seven-check live batch then passed six checks and exposed a real
  lifecycle race in feedback-loss cancellation. Cancellation during final
  stop verification discarded the await of a still-running thread, publishing
  cancelled with no cleanup evidence. Scope teardown correctly quarantined head,
  but the operation result did not retain the stop failure. A second diagnostic
  batch reproduced this result; both failed reports are retained.
- TaskHandle.verify_stop now shields and joins the worker, records successful
  evidence, and lets stopping failure take precedence over cancellation. Group
  submission/cancellation/final verification use this path. Event-controlled
  offline cases verify both successful and failed workers cannot be detached by
  cancellation. Full SDK regression: 518 passed, one optional kinematics skip.
  This is not yet a passing post-fix live batch or C completion.


#### Post-fix live acceptance

- Final image: `sha256:c6039cb04f20c62d036f5acda38babb97ab736fa1c765de015a41b2623feb04d`.
  All 97 installed SDK/agent Python files match the checkout byte-for-byte.
- One batch of the shared diagnostics suite passed all seven selected checks in
  46.76s: observer independence, guard entry/trip, healthy renewal during a slow
  read, live-socket lease expiry, killed head client, feedback-loss quarantine and
  local recovery, and network recovery denial. Motion eligibility was head only;
  these passes do not replace the earlier failed killed-base-client criterion.
- The feedback-loss operation now retains StopUnverified rather than a cancelled
  state with no stopping evidence. Recovery preserves that failed result, keeps
  inhibition until explicit reset, and permits restoration of initial head joints.
- Linux SDK: 518 passed, one optional kinematics skip. Portable catalog/conformance
  checks: 61 passed. Seven entries are visible under Diagnostics on port 8766;
  no old test runs were imported into console history. The original 8765 was left
  untouched. All controllers are restored; the default twist selector is restored.
- Reports, source hashes, image/build identity, failed diagnostic batches and the
  clean-shutdown checkpoint are retained in the ignored local evidence directory
  `ringfour-sdk/test-results/hardening-20261107-xlerobot-online/` with summary and
  SHA-256 manifest. Agent, simulator and test console remain running.
- The immediate restart blocker and cancellation-during-verification race are
  resolved for this simulator. C is not complete: submission deduplication,
  lost-reply reconciliation, durable results and bounded retry/epoch acceptance
  remain required. No physical stopping/commissioning guarantee is added.


### 2026-11-07 — Milestone C completion

**C software and XLeRobot simulator acceptance are complete.** The persistent
contract and limits are documented in `docs/milestone-c.md`.
This does not complete D, the full v1 release matrix, authenticated encrypted
remote deployment or physical commissioning.

- All admitted mutating SDK calls carry a client-generated command ID distinct
  from request/operation IDs. SQLite commits the principal/ID, immutable expiry
  and canonical request fingerprint before handler dispatch. Identical retries
  share the original reply; conflicting payloads cannot execute. Expired IDs
  remain expired after eviction; old epochs cannot execute after restart. Capacity
  rejects instead of evicting within promised retention. A durable logical clock
  floor prevents clock rollback from reviving old IDs.
- Lost mutating replies raise SubmissionUnknown with the original ID and
  reconcile-first guidance. Sync/async expose commands.new/get/retry and
  operations.get. No automatic replay or resume is implemented. A completed
  acceptance reply may refer to an operation that is still active.
- Operation IDs are random across boots. Initial records precede task dispatch;
  terminal results/errors/cleanup commit before delivery and remain queryable
  after dropped terminal notifications. Startup marks interrupted records unknown,
  retaining known terminal outcomes. Report age advances; historical stop evidence
  is not represented as a fresh physical observation.
- Same-principal authenticated local reconnection can inspect retained records.
  Unverified TCP remains bound to its original connection and advertises insecure
  development mode. IDs never confer query, cancellation or recovery authority.
- Journal failure inhibits and initiates bounded halt. Independent stopping remains
  available. Repaired storage requires a successful write plus explicit reviewed
  measured journal recovery after restart; unknown outcomes remain unknown. Shutdown
  initiates cancellation before draining command handlers, continues independent
  stop paths if draining fails, and cannot finish the boot marker with pending
  accepted handlers. Persistent tests cover these failure paths.
- Final Linux SDK regression: **542 passed, 1 optional kinematics skip**, 3.53s.
  Final dashboard/harness regression: **172 passed**. Every new contract test has
  operator guidance and is selectable in SDK; all ten shared execution-hardening
  diagnostics are visible on port 8766. No old runs were imported into history.
- Final shared live batch: **10 passed, 181 deselected**, 57.05s. It includes
  observer independence, guards, slow-read renewal, live-socket expiry, killed head
  client, feedback-loss quarantine/recovery, network recovery denial, lost trajectory
  reply with dropped terminal notifications, durable terminal/retry epoch checks
  across agent restart, and killed mobility client verified release.
- The final killed mobility case released in **0.628s** with measured 0.25s dwell
  and unchanged 0.001m/s / 0.01rad/s / 2s policy. The prior shared batch also passed
  (0.687s). The earlier failed base criterion remains in the evidence; no root-cause
  attribution to game/system load, universal timing bound or hardware safety claim
  is inferred from these later passes.
- Final image: `sha256:161e927b07bc8bf8cadc44141ef2bddf4880b92bfffa9a0c3296295ebf2c5c8b`.
  All **101** installed SDK/agent Python files match checkout sources. The agent
  state volume and earlier quarantine/recovery evidence were preserved.
- An initial live invocation could not connect because the simulator's processes
  had exited while its container stayed up. Mounted build/install caches contained
  native host paths. A preserved, isolated 15-package Linux rebuild restored the
  same standard model revision and reviewed fixture against a separate map DB.
  The robot entrypoint now skips broken native package hooks. This environment
  repair is documented in XLeRobot's guide and is not SDK motion acceptance.
- Raw XML/logs, deployment/source inventories, reviewed profiles, helpers and an
  indexed summary/manifest are local under the ignored artifact directory
  `ringfour-sdk/test-results/hardening-20261107-milestone-c/`. The tests themselves,
  fixtures and contract documentation live in the repositories for reruns.


### 2026-11-13 — Ringfour split and D scope

Split SDK, agent and testing into sibling repositories under Ringfour. Shared offline tests now belong to testing and remain discoverable in the dashboard. D is platform validation and documentation; two-robot qualification no longer gates it. See MIGRATION.md for provenance, packaging and deployment migration boundaries. D remains incomplete.


### 2026-11-13 — Learning material separation and final migration checks

Moved examples/course/recipes to sibling `missions`, archived phase demos/worklogs, retained only current SDK API/type references, and moved B/C contract records into testing/docs. Six exploration tests now belong to the separate mission suite, alongside three persistent material checks. Dashboard discovers all 478 guides with no historical-run import. See VERIFICATION.md for final counts and MIGRATION.md / ../missions/MATERIAL_REVIEW.md for scope. D remains incomplete.
