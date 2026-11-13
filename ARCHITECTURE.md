# Architecture — what belongs where

Written 2026-08-26, when four things that had grown into one directory were
separated: the SDK, the agent, the robots, and the project's own journal.

This file exists because the mixing was not cosmetic. Robocore's central
claim is that it is **robot-agnostic**, and the old layout shipped
base101's profile inside the library, asserted base101's joint names in
the library's test suite, and kept a ROS/Nav2 porting guide in the SDK's
docs. The claim was contradicted by the directory tree, and — worse — it
was *untestable*, because "port to a new robot" meant editing the SDK.

---

## The six layers

| layer | lives in | one-line rule |
|---|---|---|
| **SDK** | `ringfour-sdk/` | Knows the *protocol*. Never a robot |
| **Agent** | `ringfour-agent/` | Knows *ROS*. Never a mission |
| **Robots** | `robots/<name>/` | Knows *itself*. Never the protocol |
| **End-to-end** | `ringfour-testing/integration/` | Knows the *contract*. Never one robot |
| **Mission UI** | `ringfour-ui/` | Knows *pixels*. Never the protocol |
| **Project** | `docs/` | Knows *why*. Never runs |

Six, not four — `ringfour-testing/integration/` earned its own line the moment it
stopped being filed under a robot, and `ringfour-ui/` earned one because a
web framework must not sit behind `import ringfour`. These are neither the SDK's (they need
a robot to run) nor any one robot's (they are shared by all of them).

It is named `ringfour-testing/integration/` rather than `tests/` so that it can never be
confused with `ringfour-sdk/tests/`. The two answer different questions:
`ringfour-sdk/tests/` is what the SDK can prove about itself with nothing
attached; `ringfour-testing/integration/` is what only a real robot, running, can answer.

### 1. The SDK — `ringfour-sdk/`

```
ringfour-sdk/
  ringfour/        the library
  docs/            api-reference.md, types-reference.md
  examples/        robot-agnostic snippets
  missions/        composable user-land helpers (explore, …)
  tests/           OFFLINE only — no robot, no ROS, no Gazebo
  pyproject.toml
```

**Rule: nothing here may require a specific robot to run.**

A base101 mention in a *docstring* is fine and often better than an
abstract one — `watch_effort("arm_joint_base", above=1.5)` teaches more
than `watch_effort("<joint>", ...)`. What is not fine is *importing* a
profile, resolving a path into `robots/`, or asserting a joint name in a
test.

The offline suite is the enforcement: **no robot, runs in seconds.**
If a change makes that suite need a robot, the change is in the
wrong layer.

**The profile schema is the agent's, not the SDK's.** Verified rather than
assumed: no module under `ringfour-sdk/ringfour/` imports `RobotProfile`,
`GroupConfig`, `ToolConfig` or anything else from the 564-line schema. Only
the agent parses YAML. It now lives at
`ringfour_agent.models.profile`.

Two things both called "profile" have to be kept apart:

| | where | who reads it |
|---|---|---|
| `ProfileInfo` | `ringfour.models.protocol` | **clients** — name, model, version, capabilities, handed over at handshake |
| `RobotProfile` | `ringfour_agent.models.profile` | **the agent** — the full YAML vocabulary for describing a robot |

A mission author never writes a profile; a robot integrator does. Shipping
the schema in the client library meant every mission author downloaded the
grammar for configuring machines they do not configure. The *enums* stay in
the SDK — `Controller`, `GroupKind`, `Locomotion` cross the socket, so both
sides must agree on those strings.

> **Known exception, deliberately kept.** Nine of the thirteen offline
> modules import `ringfour_agent`. That looks like a layering violation
> and is not: they test the **wire contract**, and a contract has two
> sides. The SDK's encoder and the agent's decoder must agree about the
> same bytes, and asserting that needs both in the room. They still need
> no robot, which is what keeps them here. A test that needs a *running*
> agent is in the wrong directory.

### 2. The agent — `ringfour-agent/`

The ROS-side bridge. Owns `rclpy`, controllers, TF, action clients, the
safety layer and the audit log.

**Rule: the agent may know about ROS and about the protocol. It may not
know what a mission is trying to achieve.**

#### Shared source, per-robot build

The agent sits next to the SDK rather than inside any robot, and it is
built separately by every robot. Both halves of that matter.

**One source, because there is one wire contract.** The SDK's encoder and
the agent's decoder have to agree about the same bytes. Vendoring a copy of
the agent into each robot would let two robots drift onto two dialects of
the protocol, and the failure would show up as a client that works on one
robot and returns nonsense on another. It is the same argument that keeps
the contract tests in `ringfour-sdk/tests/`.

**Many builds, because a workspace is a robot's own business.** Each robot
owns its container, its colcon workspace and its dependency list, and mounts
the shared `agent/src` and `ringfour-sdk/` into it:

```
robots/base101/docker/    compose + image for the mobile robot
robots/mod101/docker/     compose + image for the bench
```

The difference between those two images is the point. base101's carries
Nav2, slam_toolbox, laser_filters, twist_mux and robot_localization.
mod101's carries **none** of them — the bench is bolted to a table, cannot
drive, has no lidar and has nothing to localise against. Sixteen
`ros-jazzy-*` lines become nine. A shared image would force the bench to
ship a navigation stack that exists only to be unused, and an unused Nav2
in an image is an invitation to wire it up by accident.

They also run on **different ROS domains** — base101 on 0, the bench on 1 —
because two robots on one graph means two sets of `/joint_states` racing.

The cost is honest and worth stating: a compose file in a robot repo
reaches up to `../../../agent/src` and `../../../ringfour`, so it assumes
the standard checkout with robots under `bpe/robots/`. A robot cloned on
its own builds nothing until the SDK and agent are beside it. That is the
price of one protocol implementation, and it is the right trade.

`ros2 run ringfour_agent onboard` writes its snapshots beside the profile
it was given — which now means `robots/<robot>/profiles/<name>/<sha>/`. That fell
out of the move for free, and it is right: a snapshot is measured data
about one robot.

### 3. The robots — `robots/`

```
robots/
  base101/         robot repo (own git) — SELF-CONTAINED
    src/             description, control, nav, bringup
    docker/          its stack: compose + image, Nav2 + SLAM + filters
    profiles/        base101.yaml, base101_arm.yaml, base101_arm_hw.yaml
      <name>/<sha>/  its onboarding snapshots
  mod101/          robot repo (own git) — SELF-CONTAINED
    src/             description, control, tools, harness
    docker/          its stack: compose + image, no navigation at all
    profiles/        mod101_harness.yaml
    docs/            its own measurements (dof-and-masks.md)
```

**There is no shared profile pool.** An earlier draft of this layout put
every profile in `robots/<robot>/profiles/`, which was better than shipping them
inside the SDK and still wrong: adding a robot meant editing a directory
every other robot shared. A robot's profile is part of that robot's
solution, so it lives in that robot's repo, next to the description it
describes and the container that runs it.

The test is unchanged and now actually holds: **adding a robot creates
`robots/<name>/` and touches nothing else.**

**Rule: everything one robot needs, and nothing another robot shares.**
Description, controllers, container, profile, snapshots, measurements.
`docker/` used to sit at the repo root as if it were shared; it was
base101's sim stack throughout, down to launching
`base101_bringup_gazebo`.

The `ringfour-testing/integration/` used to live here as `robots/conformance/`, and
before that inside the SDK. See §4 — they belong to neither.

### 4. End-to-end tests — `ringfour-testing/integration/`

The executable definition of what a robot must do to be drivable by
ringfour. One suite, run against any loaded profile through
`RINGFOUR_AGENT_TCP` or `RINGFOUR_AGENT_UNIX`. The handshake selects its
capabilities and named instances; optional `RINGFOUR_PROFILE` supplies an
independent YAML to check those claims.

**Rule: they may assume a robot exists. They may not assume which one.**
Checks expand across live cameras, groups, and sensors. Physical scenarios
belong to the robot, supplied through `RINGFOUR_CONFORMANCE`; joint names,
controller roles, tool endpoints, and dimensions come from live metadata.
Missing capabilities are not applicable. Missing scenarios or unsupported
backend contracts are untested, keeping the dashboard's run incomplete.

Formerly `engine/e2e_testing/test_phase*_exit.py`, filed with the SDK. They were
never SDK tests: they require a live agent. Their former base101-specific
assertions have been replaced by shared capability contracts. Startup and
profile migration regressions use synthetic offline SDK fixtures.

### 5. Mission UIs — `ringfour-ui/`

```
ringfour-ui/
  ringfour_ui/     map_image(), MapView, theme(), launch()
  tests/           OFFLINE — pixel assertions, no browser
  pyproject.toml
```

**Rule: it may import the SDK. The SDK may never import it.**

A mission UI is a **Gradio app**. This package adds two things and nothing
else:

| | why it exists |
|---|---|
| `map_image()` / `MapView` | Gradio has an Image, a Dataframe and a Chatbot; it has no "occupancy grid with robot poses and a field of measured values drawn over it in world coordinates", and never will. That is a robotics concern |
| `theme()` / `launch()` | the Robocore design system, shared with `robots/mod101/configurator`, so two operator surfaces in one project do not look like two projects |

It is a separate distribution rather than a `ringfour[ui]` extra because the
SDK's rule is that it knows the protocol and never a robot. A web framework
behind `import ringfour` breaks that, and would make every mission author
download Gradio to talk to a robot they never intend to watch.

**This replaced a much larger thing, deliberately (2026-08-28).** There was a
two-part system built to `docs/archive/ringfour-ui-spec-v0.1.md`: a
declaration library speaking a wire format to a separate NiceGUI renderer
("Studio"), ~3,300 lines, 42 tests, working. It was deleted in favour of
Gradio because AI components — chat, image annotation, galleries, file drops
— are going to be most of what these UIs show, and a closed set of six
primitives could never keep pace. The archived spec's header records what was
given up: out-of-process chrome (a hung mission is now a dead UI, stop button
included), dial-out transport, and the fleet view.

**The rule that survives from it:** nothing in `ringfour-ui/` may construct a
motion command the author did not write. A UI reaches mission code; mission
code decides what to call. That is why attaching a screen adds no safety
surface, and it is one convenience helper away from not being true.

### 6. The project — `docs/`

```
ARCHITECTURE.md          this file — at the root, because it is the map
docs/
  actual_docs/           written to be read by someone else
    porting-a-robot.md     for integrators: ROS, Nav2, URDF, profiles
  plans_and_architecture/  specs and drafts, incl. the API reference
  journal/                 written to be read by us, later
    log.md                   what happened
    found_along_the_way.md   what broke
    state-of-play.md         where we stand
  archive/                 superseded, kept for provenance
```

`docs/actual_docs/porting-a-robot.md` moved out of the SDK because it never mentions an SDK
call. Its audience owns a robot and wants it to speak ringfour.

---

## The test that this layout has to pass

**Adding a robot must touch zero files in `ringfour-sdk/`.**

Concretely, for the Tiago step (`state-of-play.md` §7 item 5):

1. write `robots/<robot>/profiles/tiago.yaml`;
2. run onboarding → `robots/<robot>/profiles/tiago/<sha>/`;
3. point `RINGFOUR_AGENT_TCP` at its running agent, optionally supply its
   profile YAML and reviewed conformance scenarios, and run
   `pytest ringfour-testing/integration/test_*.py` or the local test console.

---

## Still open

**Backend coverage still limits what can be proved.** The suite no longer
assumes a robot morphology, but the current SDK's planar teleop/Nav2 APIs
do not implement flight control or balance recovery. Its IK backend also
cannot solve independently articulated mounts or separate group descriptions.
These gaps are explicitly untested, not disguised as green conformance.
Native FK checks compare in each group's mounting frame. Robot-specific
physical scenarios remain necessary; the suite cannot infer safe targets
or independently known collision/contact cases from a profile.

**The Nav2 / SDK boundary** (`state-of-play.md` §6). Two fixes in one
session landed on opposite sides of a line nobody had drawn: the laser
filter went robot-side, correctly; costmap clearing sits in the SDK
because the behaviour server that used to do it was deleted. Proposed
rule, still to be agreed:

> The robot owns anything that would be wrong for **any** consumer —
> sensor filtering, TF, controllers, the URDF. The SDK owns anything only
> wrong for a **mission** — goal lifecycle, recovery policy, costmap state
> between goals. If a second, non-ringfour client would need it too, it
> belongs to the robot.

By that rule costmap clearing is misplaced.

---

## Practical notes

- The venv lives at `ringfour/.venv`. It moved with the directory rename,
  so `.venv/bin/pip` has a stale shebang — use `.venv/bin/python -m pip`.
  The editable install was reinstalled at the new path; if you rebuild the
  venv, `python -m pip install -e . --no-deps` from `ringfour-sdk/`.
- Offline suite: `cd ringfour-sdk && .venv/bin/python -m pytest tests/ -q`
- System tests (need a live robot):
  `cd ringfour-testing/integration && RINGFOUR_EXTERNAL_STACK=1 ... pytest . -q`
- Each robot brings its own stack up from its own directory:
  `cd robots/base101/docker && docker compose -f docker-compose.sim.yaml up`
  `cd robots/mod101/docker  && docker compose -f docker-compose.bench.yaml up`
  Build the workspace first with `run --rm build` after changing sources.
- base101 is domain 0, the mod101 bench is domain 1. A client must export
  the matching `ROS_DOMAIN_ID`.
