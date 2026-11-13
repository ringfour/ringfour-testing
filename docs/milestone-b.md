> Migrated contract/history: dated deployment evidence belongs to the original checkout. Local run archives were not copied. See [current migration verification](../VERIFICATION.md); this move does not establish new robot acceptance.

# Milestone B implementation and migration

Milestone B's software implementation and Handy simulator acceptance are
**complete** on `review_and_hardening` (13 October 2026). The running agent's 94
SDK/agent Python files exactly match this checkout. This does not complete C/D,
the full v1 release matrix, second-robot conformance or physical commissioning.
See [the implementation log](../HARDENING_LOG.md#b-completion-evidence-and-boundary)
for the deployed image and evidence.

## Accepted operations

`start_navigation(Pose)`, `start_route(poses)`, `tool.start_position(value)`
and `control.start_trajectory(JointTrajectory)` submit
immediately and return an `Operation`. They require an agent advertising verified
operations. Breaking or closing `op.updates()` stops observation only; work
continues. Leaving the operation scope or calling `op.close()` cancels unfinished
work and joins bounded cleanup. Keeping an iterator alive has no effect on work.
Sync and async facades have the same execution/lifetime contract.

`op.snapshot` is immutable cached state, with sequence and the agent-computed report age plus elapsed client receipt age.
Querying a retained report does not make its old progress fresh; no comparison
between client, agent and ROS clock epochs is used.
`op.get_status()` queries the originating agent. `op.result(timeout=...)` returns
a typed result, raises an execution error, or raises `OperationCancelled`.
`WaitTimeout` means the wait expired and work continues. `op.cancel()` returns
`CancelReceipt`, which acknowledges the request and does not prove stopping.
Terminal cancellation retains agent stopping evidence; unverified cleanup fails.

`navigate_to(goal, timeout=...)` and `control.run(trajectory, timeout=...)` are
blocking convenience methods returning typed verified results. Their timeout
requests cancellation and attempts finite cleanup before `OperationTimeout`.
Async cancellation of these convenience calls also joins bounded cleanup.
An observer's Python cancellation does not cancel independently owned work.

## Model-bound joint commands

`group.configuration({joint_name: value, ...})` creates a `JointConfiguration`
with exact joint coverage, robot/group identity, model revision and model-derived
radian/metre units and bounds. Values are immutable. `kin.ik` returns this type;
`kin.fk(group, q=configuration)` validates it before hypothetical calculations.
`JointTrajectory.from_waypoints(configurations, step_duration=...)` constructs
strictly increasing finite times. Explicit `times=` supports unequal intervals.
All configurations must refer to the same robot/group/model and joint model.
The agent repeats identity, coverage, unit, bounds and revision checks.

A stale value or stale session fails with `ModelChanged`. A named group model
must be refreshed explicitly when it differs from the composed description.
Endpoint validation does not prove interpolated velocity/acceleration limits.
Results preserve native-unit residuals and stopping evidence; controller
acknowledgement is not arrival.

## Measurements and guards

`get_pose_measurement`, `get_joint_measurement` and `get_signal` produce immutable
`Measurement` envelopes. Source and receive ages are separately computed by the
agent; unknown source age is unavailable for required freshness. `get_pose()`
returns a usable fresh framed Pose or raises StateUnavailable/StateStale. Other
latest-state readers are explicitly documented in the API reference.

Discovered signals build typed above/below conditions. `guard()` is a lazy scope:
entry confirms fresh safe evidence, a threshold/stale/unknown signal trips a
persistent robot-wide inhibit, and removal stops/drains before removing only
that guard. Reset requires fresh safe data and does not resume old work. E-stop,
other guards and quarantine remain independent. Failed cleanup retains inhibition.

## Migration and current acceptance

Remote properties now use matching sync/async `get_*()` names. Old route/follow
iterators, task-returning navigation spelling, group/gripper aliases and dotted
watch entry are removed. Named watches remain observational; guard() replaces
effectful stop=True watches. Backend/controller metadata and raw control are
explicit integration diagnostics. Sync power reads return values, not coroutines;
unknown source remains None. Generated reference/protocol and examples are
migrated; notebook generation parity and code-cell parsing pass.

Final-image head acceptance passes measured trajectory/return, observer and wait
independence, explicit cancellation, convenience-timeout cleanup, unsafe guard
entry, threshold trip/drain and subsequent inhibition. Closed/partial/full tool
positioning passes after verified upstream physics and independent target/stop
budgets; stop tolerances were not relaxed. Full regression: 650 passed. Live group
metadata: 22 passed. Read-only compatibility: 61 passed, 85 skipped, 110 deselected.
Nine course notebooks match their generator and all 47 code cells parse; they
have not been executed end-to-end on a robot.

Navigation plans, independently verified arrival, both finite-route endpoints
and cancellation after observed movement pass. The Handy-only RPP controller
uses 0.15m/s path following, 0.15rad/s heading rotation and tighter controller
arrival thresholds; SDK 0.1m/0.1rad arrival and native stop policies are unchanged.
Initial MPPI/RPP failures and typed timeout cleanup remain in the evidence.
Async head/tool/Python-cancellation and actual-motion navigation checks pass.
The reviewed Handy navigation wait budget is 180s for the slower controller;
it governs waiting, not measured acceptance tolerances.

| Acceptance | Evidence |
|---|---|
| SDK/agent/dashboard regression | 650 passed |
| Live group metadata and watches | 22 passed |
| Reviewed head qualification/admission | 10 passed, 2 inapplicable |
| Read-only compatibility | 61 passed, 85 skipped, 110 deselected |
| Native joint limits and map during motion | 6 passed |
| Sync execution/guard/observer/tool/navigation contracts | Passed; incremental typed results |
| Async head/tool/navigation and Python cancellation | Passed; incremental typed results |
| Simulator physics | Full upstream checks passed; unchanged joint/native-rest assertions |
| Documentation | Generated protocol/types, nine notebooks and 47 parsed code cells; examples/missions parse |

Artifacts are local under `test-results/hardening-20261013-b/`: `deployment.json`,
`deployed-python-hashes.json`, `live-acceptance.json`, JUnit reports and logs.
Failures are retained separately in `live-failures.jsonl` and the MPPI attempt
log; their presence is not erased by later passes. Source/fixture changes and
rejected physics/controller trials are recorded in the implementation log.

The local development Compose still mounts the optional Nav2 launch extension
from the neighbouring base101 checkout. Publish that dependency update before
removing the mount. Restart-durable quarantine and distributed reconciliation
remain C work; rebooting the fixture did not demonstrate privileged recovery.

Current API: [api-reference.md](api-reference.md). Detailed test, deployment and
physics evidence: [implementation log](../HARDENING_LOG.md). Milestones C and D
remain separate: distributed reconciliation/leases and second-robot/physical
commissioning are not implied by B simulator acceptance.
