> Migrated contract/history: dated deployment evidence belongs to the original checkout. Local run archives were not copied. See [current migration verification](../VERIFICATION.md); this move does not establish new robot acceptance.

# Milestone C: distributed execution lifecycle

C combines finite authority leases, reconciliation of uncertain submissions,
retained terminal evidence, principal checks and explicit measured recovery.
This page describes the implementation on `review_and_hardening`; the complete
v1 release specification also contains D and physical commissioning gates.

Acceptance status (November 7, 2026): **C software and XLeRobot simulator
acceptance complete**. Linux SDK: 542 passed, one optional kinematics skip.
Dashboard/harness: 172 passed. Shared live diagnostics: 10 passed in 57.05s.
All 101 installed SDK/agent Python files match the checkout. This is not D,
authenticated encrypted remote release acceptance or physical commissioning.

## Submission identity and retry rules

A hardened agent advertises `features.commands`: boot epoch, server time,
300-second maximum validity, at least 300-second reply retention, 10,000-record
capacity, durable-storage availability and the mutating method inventory.
The SDK attaches a client-generated `command_id` to every admitted motion,
configuration, guard and stop request. The ID is distinct from the JSON-RPC
request ID and operation ID; its format is `epoch.expiry_milliseconds.uuid`.
Expiry is immutable inside the ID. Regenerate neither its expiry nor its UUID
when reconciling uncertain work.

The agent validates the schema and permission, then commits the principal/ID,
canonical request fingerprint and pending acceptance before handler dispatch.
Only the fingerprint is stored, not the command payload. Concurrent identical
submissions share one dispatch and original reply; changed payloads receive
`CommandConflict`. Retained errors are returned consistently too. The dispatch
rechecks current lease, ownership, inhibit, model and adapter eligibility.

Expired IDs receive `CommandExpired`, even after their deduplication record was
evicted. Earlier boot epochs receive `AgentEpochChanged`, even when their result
can still be queried. A persistent clock floor prevents backwards clock changes
from reviving an evicted ID. Capacity exhaustion rejects new effects rather than
evicting records that are still within their retention promise.

A connection loss or timeout while awaiting a mutating reply raises
`SubmissionUnknown` with `details['command_id']` and `retry: reconcile-first`.
It does not assert that dispatch failed. The SDK never automatically replays it.
Read timeouts retain the ordinary read-error contract.

```python
from ringfour import SubmissionUnknown

# Keep the original control scope: exiting it still cancels/drains its work.
try:
    operation = control.start_trajectory(trajectory)
except SubmissionUnknown as uncertain:
    accepted = robot.commands.get(uncertain.details['command_id'])
    if accepted.operation_id is not None:
        observed = robot.operations.get(accepted.operation_id)
        # Inspect recorded evidence; explicitly stop/close the original scope
        # if the mission cannot continue observing it.
```

For explicit protocol integrations, sync and async both provide
`commands.new(valid_for=...)`, `commands.retry(original_id, method, params)` and
`commands.get(original_id)`. Retry the identical request only after reconciliation;
never invent a replacement ID to bypass an unknown result. `CommandRecord.state`
reports the request/acknowledgement lifecycle: completed may contain an operation
ID whose motion is still active. Query `operations.get(id)` for execution evidence.

## Results across disconnect and restart

The agent's state directory contains a per-robot SQLite ledger with full-sync
transactions and WAL. Production CLI/Compose use the persistent state volume.
In-process tests without a state directory advertise `durable: false`.
Operation IDs are random across boots. Initial records precede task scheduling;
terminal results, errors and cleanup evidence commit before terminal delivery.
Terminal retention starts at completion, independently of acceptance retention.
The bounded notification queue is never the sole copy of terminal evidence.

`robot.operations.get(id)` returns an `OperationRecord` with the original epoch,
state and snapshot. A terminal snapshot retains its result/error/cleanup and
sequence; report age grows rather than looking like a new sensor observation.
An active record found at a new boot becomes unknown. A pending acceptance found
after a crash also becomes unknown. Neither is replayed, relabelled succeeded or
presented as an exactly-once physical execution guarantee. Observation of retained
stopping evidence is not a fresh assertion that the robot remains stopped.

Local Unix identity comes from kernel peer credentials. Authenticated same-principal
reconnection can inspect retained records; another principal cannot. Cancellation
and control-session mutation still require their current owner/generation.
An unverified TCP connection can reconcile only its own live connection's records.
The opt-in trusted development listener advertises `insecure-development`; its
shared operator permission is not authenticated identity and cannot recover or
inherit another connection's records. Remote authenticated encrypted deployment
remains a separate v1 release/deployment requirement, not a claim of this local
and simulator C acceptance.

## Authority and recovery

Configured finite leases revoke authority before cancellation/cleanup; renewal
never refreshes teleop intent. Priority renewal/stop dispatch prevents slow reads
from starving heartbeats. Disconnect drains accepted command handlers before
restoring controllers. Cancellation during a stop-verification worker joins that
worker and retains its failure instead of publishing unsupported cancellation.

Durable quarantine and active-boot markers survive agent replacement. Explicit
local recovery requires inhibition, idle ownership, reviewed adapter/controller
state and progressive measured-rest evidence. Recovery clears only the named
verified resource, keeps inhibition, retains failed results and never resumes work.
Restart recovery supports reviewed trajectory-only groups and declared mobility;
model changes and opaque vendor/tool restart recovery fail closed. Storage faults
inhibit new effects and initiate the bounded shared halt of accepted work. Independently authorized stopping remains available if the
ledger fails; unavailable storage cannot provide a durable acceptance guarantee
for that degraded stop path. After repairing storage and restarting the agent,
a local operator may recover the journal quarantine against the same reviewed
controller/measurement baseline. Recovery verifies a new ledger write first;
unknown operation records remain unknown and are never resumed.

## Persistent acceptance checks

Every test below is a repository test with dashboard operator guidance. Offline
checks are selectable in SDK; robot checks are in Diagnostics. Robot names,
controller/topic names and lifecycle/fault argv belong in reviewed conformance
fixtures, not test implementation.

| Acceptance | Persistent evidence |
|---|---|
| A12/A13 finite authority and stale generation | `test_leases_contract.py`; shared head lease-expiry/client-death checks; reviewed mobility client-death check |
| A14 lost reply/concurrent duplicate/conflict | `test_submission_reconciliation_contract.py`; shared lost-motion-reply check |
| A15/A25 restart epoch and expiry/eviction | offline crash/expiry/clock-floor checks; shared agent-restart probe |
| A16 principal ownership | offline authenticated/unverified/cross-principal queries; existing owner cancellation tests; live network recovery denial |
| A18 terminal delivery loss | real-protocol offline query; shared trajectory test drops terminal notifications and queries authoritative evidence |
| A09/A24 recovery and truthful stopping | durable safety contracts; shared feedback-loss/local recovery; restart baseline recovery |
| A23 cancellation/completion race | event-controlled stop-worker and duplicate-submission tests |

`lifecycle` conformance requires reviewed local probe/restart argv and explicit
debug eligibility. Its helper uses a synthetic non-motion task and restarts only
the selected agent. `mobility_fault` separately requires reviewed drive axes,
watchdog, input isolation/restoration argv and timeout. Missing fixtures are
unconfigured, never counted as passing fault coverage.

Final image: `sha256:161e927b07bc8bf8cadc44141ef2bddf4880b92bfffa9a0c3296295ebf2c5c8b`.
The final killed mobility case reached verified release in 0.628s with the original
stop thresholds unchanged. Earlier failed attempts remain retained and do not
establish an attributed root cause or a universal physical stop bound.

See `HARDENING_LOG.md` for completion evidence and the local raw artifact index.
Run outputs are stored under the ignored SDK test-results convention; test
implementations, reviewed robot fixtures and documentation persist in the repos.
