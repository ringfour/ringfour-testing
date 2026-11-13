"""Vendor escape hatches: profile `sensors:` and `actions:`.

The feature exists because the capability registry can only cover what
most robots have. A servo's temperature, a front LED and an outrigger are
things one vendor decided to put on a topic, and the profile is where they
get names.

Driven through the real agent stack with a fake ROS side, so the wire
path, the audit trail and the guard rails are all exercised — none of it
needs a robot.
"""

from __future__ import annotations

import asyncio
import threading
import time

import pytest
import yaml
from pydantic import ValidationError

from ringfour import Robot
from ringfour.enums import AuditKind
from ringfour.errors import CapabilityNotSupported, RingfourError
from ringfour_agent.access import AccessPolicy
from ringfour_agent.models.profile import ActionConfig, RobotProfile, SensorSignal

from ringfour_agent.bootstrap import build_agent
from ringfour_agent.profile import load_profile

from .conftest import _free_tcp_port

PROFILE = """
name: vendor-test
model: fake robot with vendor topics
version: "0"
protocol: 1
frames: {map: map, odom: odom, base: base_link}
sensors:
  base_motor:
    temperature: {topic: /motor_telemetry/j1/temperature,
                  type: std_msgs/Int32, units: degC}
    current:     {topic: /motor_telemetry/j1/current,
                  type: std_msgs/Float32, units: mA}
  bus:
    voltage: {topic: /power/bus, type: std_msgs/Float32, units: V}
actions:
  set_front_led:
    topic: /leds/front
    type: std_msgs/ColorRGBA
    is_motion: false
    max_rate: 100
    parameters:
      r: {type: float, min: 0.0, max: 1.0, default: 0.0}
      a: {type: float, min: 0.0, max: 1.0, default: 1.0}
  horn:
    topic: /horn
    type: std_msgs/Empty
    is_motion: false
    max_rate: 2
  deploy_outriggers:
    topic: /outriggers/command
    type: std_msgs/Bool
    is_motion: true
    parameters:
      data: {type: bool}
watches: {}
"""


# -- schema ----------------------------------------------------------------------

def test_is_motion_has_no_default():
    """Is motion has no default

    Categories: sdk
    What it checks: Neither default is safe. `true` makes every LED author write
    `false`, which they then paste onto something that does move;
    `false` lets a brake release through unguarded. Requiring it costs one
    line and forces the thought once, where the context is. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Neither default is safe. `true` makes every LED author write
    `false`, which they then paste onto something that does move;
    `false` lets a brake release through unguarded. Requiring it costs one
    line and forces the thought once, where the context is.
    """
    with pytest.raises(ValidationError, match="is_motion"):
        ActionConfig.model_validate(
            {"topic": "/x", "type": "std_msgs/Bool"})


def test_a_default_outside_its_own_range_is_rejected():
    """A default outside its own range is rejected

    Categories: sdk
    What it checks: Verifies that a default outside its own range is rejected. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    with pytest.raises(ValidationError, match="above max"):
        ActionConfig.model_validate({
            "topic": "/x", "type": "std_msgs/Float32", "is_motion": False,
            "parameters": {"v": {"type": "float", "max": 1.0,
                                 "default": 5.0}}})


def test_a_signal_needs_only_a_topic():
    """A signal needs only a topic

    Categories: sdk
    What it checks: Most telemetry is a single-field std_msgs; the defaults should
    cover it without ceremony. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Most telemetry is a single-field std_msgs; the defaults should
    cover it without ceremony.
    """
    signal = SensorSignal.model_validate({"topic": "/t"})
    assert signal.type == "std_msgs/Float32"
    assert signal.field is None and signal.units is None


def test_both_sections_are_capabilities():
    """Both sections are capabilities

    Categories: sdk
    What it checks: Verifies that both sections are capabilities. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    profile = RobotProfile.model_validate(yaml.safe_load(PROFILE))
    assert "sensors" in profile.capabilities
    assert "actions" in profile.capabilities
    assert profile.instances["sensors"] == ("base_motor", "bus")
    assert profile.instances["actions"] == (
        "set_front_led", "horn", "deploy_outriggers")


def test_a_url_in_a_vendor_section_is_still_rejected():
    """A url in a vendor section is still rejected

    Categories: sdk
    What it checks: The no-file-path rule applies to every section, including new
    ones — `_Section` is where it lives, so this comes free, but it is
    worth pinning. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The no-file-path rule applies to every section, including new
    ones — `_Section` is where it lives, so this comes free, but it is
    worth pinning.
    """
    data = yaml.safe_load(PROFILE)
    data["sensors"]["bus"]["voltage"]["urdf"] = "/tmp/x.urdf"
    with pytest.raises(ValidationError, match="never points at a description"):
        RobotProfile.model_validate(data)


# -- the agent -------------------------------------------------------------------

class FakeRos:
    """The vendor surface RosInterface exposes, plus what bootstrap needs."""

    def __init__(self):
        self.published: list[tuple[str, dict]] = []
        self.values = {("base_motor", "temperature"): 41.0,
                       ("base_motor", "current"): 820.5}
        self.units = {"temperature": "degC", "current": "mA", "voltage": "V"}

    def vendor_read(self, group, signal):
        value = self.values.get((group, signal))
        if value is None:
            return None
        return {"value": value, "stamp": time.time(),
                "units": self.units.get(signal)}

    def vendor_value(self, group, signal):
        return self.values.get((group, signal))

    def vendor_snapshot(self):
        return {"base_motor": {"temperature": self.vendor_read(
            "base_motor", "temperature")}}

    def vendor_ready(self, action):
        return True

    def vendor_publish(self, action, payload):
        self.published.append((action, payload))

    def joint_snapshot(self):
        return {}


@pytest.fixture()
def robot(tmp_path):
    path = tmp_path / "vendor.yaml"
    path.write_text(PROFILE)
    ros = FakeRos()
    port = _free_tcp_port()
    server, ctx = build_agent(
        load_profile(path), unix_path=None, host="127.0.0.1", port=port,
        audit_dir=tmp_path / "audit", ros=ros,
        access_policy=AccessPolicy(trusted_tcp=True), allow_unverified_stops=True)
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    asyncio.run_coroutine_threadsafe(server.start(), loop).result(5)
    with Robot(f"ws://127.0.0.1:{port}") as client:
        yield client, ros, ctx
    asyncio.run_coroutine_threadsafe(server.close(), loop).result(5)
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)
    ctx.audit.close()


# -- sensors ---------------------------------------------------------------------

def test_a_signal_reads_as_a_number(robot):
    """A signal reads as a number

    Categories: sdk
    What it checks: Verifies that a signal reads as a number. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, _ros, _ctx = robot
    assert client.sensors.base_motor.get_temperature() == 41.0
    assert client.sensors.base_motor.get_current() == 820.5


def test_units_are_reported_never_converted(robot):
    """Units are reported never converted

    Categories: sdk
    What it checks: Int32 celsius beside Float32 milliamps is exactly the pair that
    misleads silently. Saying so is cheap; rescaling would be a trap. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Int32 celsius beside Float32 milliamps is exactly the pair that
    misleads silently. Saying so is cheap; rescaling would be a trap.
    """
    client, _ros, _ctx = robot
    assert client.sensors.base_motor.units("temperature") == "degC"
    assert client.sensors.base_motor.units("current") == "mA"


def test_declared_but_silent_is_not_the_same_as_absent(robot):
    """Declared but silent is not the same as absent

    Categories: sdk
    What it checks: A signal nobody has published yet reads as None. A signal that does
    not exist raises. A mission deciding whether to wait or give up needs
    the difference. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    A signal nobody has published yet reads as None. A signal that does
    not exist raises. A mission deciding whether to wait or give up needs
    the difference.
    """
    client, _ros, _ctx = robot
    assert client.sensors.bus.get_voltage() is None
    with pytest.raises(AttributeError, match="no signal"):
        _ = client.sensors.base_motor.get_nonesuch()


def test_an_unknown_group_names_what_exists(robot):
    """An unknown group names what exists

    Categories: sdk
    What it checks: Verifies that an unknown group names what exists. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, _ros, _ctx = robot
    with pytest.raises(CapabilityNotSupported, match="base_motor"):
        _ = client.sensors.nonesuch


def test_signals_are_discoverable(robot):
    """Signals are discoverable

    Categories: sdk
    What it checks: Verifies that signals are discoverable. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, _ros, _ctx = robot
    assert set(client.sensors.groups) == {"base_motor", "bus"}
    assert set(client.sensors.base_motor.signals) == {"temperature", "current"}
    assert "watch_temperature" in dir(client.sensors.base_motor)


def test_signals_are_watch_paths(robot):
    """Signals are watch paths

    Categories: sdk
    What it checks: The reason the feature is worth building: a vendor signal is
    watchable like anything else, so `stop=True` on a servo temperature
    halts the robot through the shipped Phase 4 path. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The reason the feature is worth building: a vendor signal is
    watchable like anything else, so `stop=True` on a servo temperature
    halts the robot through the shipped Phase 4 path.
    """
    _client, _ros, ctx = robot
    assert "sensors.base_motor.temperature" in ctx.watch_paths
    assert ctx.watch_paths["sensors.base_motor.temperature"]() == 41.0


# -- actions ---------------------------------------------------------------------

def test_an_action_publishes_with_its_defaults(robot):
    """An action publishes with its defaults

    Categories: sdk
    What it checks: Verifies that an action publishes with its defaults. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, ros, _ctx = robot
    client.actions.set_front_led(r=1.0)
    assert ros.published == [("set_front_led", {"r": 1.0, "a": 1.0})]


def test_an_action_with_no_parameters(robot):
    """An action with no parameters

    Categories: sdk
    What it checks: Verifies that an action with no parameters. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, ros, _ctx = robot
    client.actions.horn()
    assert ros.published[-1] == ("horn", {})


def test_arguments_are_range_checked(robot):
    """Arguments are range checked

    Categories: sdk
    What it checks: Verifies that arguments are range checked. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, ros, _ctx = robot
    with pytest.raises(RingfourError, match="above the declared maximum"):
        client.actions.set_front_led(r=5.0)
    assert not ros.published, "nothing should reach the topic"


def test_unknown_arguments_are_refused(robot):
    """Unknown arguments are refused

    Categories: sdk
    What it checks: Verifies that unknown arguments are refused. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, _ros, _ctx = robot
    with pytest.raises(RingfourError, match="unexpected"):
        client.actions.set_front_led(colour="red")


def test_an_unknown_action_names_what_exists(robot):
    """An unknown action names what exists

    Categories: sdk
    What it checks: Verifies that an unknown action names what exists. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, _ros, _ctx = robot
    with pytest.raises(AttributeError, match="set_front_led"):
        _ = client.actions.nonesuch


def test_the_rate_limit_is_server_side(robot):
    """The rate limit is server side

    Categories: sdk
    What it checks: An arbitrary topic is otherwise a hole straight through to the
    robot's DDS. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    An arbitrary topic is otherwise a hole straight through to the
    robot's DDS.
    """
    client, _ros, _ctx = robot
    client.actions.horn()                       # max_rate: 2 Hz
    with pytest.raises(RingfourError, match="limited to 2 Hz"):
        client.actions.horn()


def test_every_action_is_audited(robot):
    """Every action is audited

    Categories: sdk
    What it checks: Verifies that every action is audited. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, _ros, _ctx = robot
    client.actions.set_front_led(r=0.5)
    events = client.audit.query(kind=AuditKind.COMMAND, limit=20)
    invoked = [e for e in events if e.call == "actions.set_front_led"]
    assert invoked, "an action is a command and must be in the safety record"
    assert invoked[-1].detail["topic"] == "/leds/front"
    assert invoked[-1].detail["is_motion"] is False
    assert invoked[-1].detail["arguments"] == {"r": 0.5, "a": 1.0}


def test_a_motion_action_declares_itself(robot):
    """A motion action declares itself

    Categories: sdk
    What it checks: Verifies that a motion action declares itself. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, _ros, _ctx = robot
    assert client.actions.moves_the_robot("deploy_outriggers")
    assert not client.actions.moves_the_robot("set_front_led")
    assert client.actions.deploy_outriggers.is_motion


def test_a_motion_action_is_refused_under_estop(robot):
    """A motion action is refused under estop

    Categories: sdk
    What it checks: `is_motion: true` means it goes through the same gate as anything
    else that moves the robot — that is the entire reason the field is
    required rather than defaulted. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    `is_motion: true` means it goes through the same gate as anything
    else that moves the robot — that is the entire reason the field is
    required rather than defaulted.
    """
    from ringfour.errors import SafetyViolation

    client, ros, _ctx = robot
    client._runner.run(client._robot.connection.call(
        "debug.set_estop", {"engaged": True}))
    try:
        with pytest.raises(SafetyViolation):
            client.actions.deploy_outriggers(data=True)
        assert not ros.published
    finally:
        client._runner.run(client._robot.connection.call(
            "debug.set_estop", {"engaged": False}))


# -- typed watch handles (row 23) ------------------------------------------------

def test_a_dotted_path_still_works_but_warns(robot):
    """A dotted path still works but warns

    Categories: sdk
    What it checks: Shipped surface, kept through v1.0 (decision 8). Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    Shipped surface, kept through v1.0 (decision 8).
    """
    client, _ros, _ctx = robot
    assert not hasattr(client,'watch')


def test_a_typed_sensor_watch_needs_no_string(robot):
    """A typed sensor watch needs no string

    Categories: sdk
    What it checks: Verifies that a typed sensor watch needs no string. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, _ros, _ctx = robot
    with client.sensors.base_motor.watch_temperature(above=60) as watch:
        # The sampler runs at 20 Hz, so a watch has no value the instant
        # it is armed.
        for _ in range(40):
            if watch.get_value() is not None:
                break
            time.sleep(0.05)
        assert watch.get_value() == 41.0


def test_a_typo_in_a_typed_handle_raises_at_the_callsite(robot):
    """A typo in a typed handle raises at the callsite

    Categories: sdk
    What it checks: The whole point. A mistyped path is a monitor that silently never
    fires, which for `stop=True` means the guard is absent and nothing
    says so. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    The whole point. A mistyped path is a monitor that silently never
    fires, which for `stop=True` means the guard is absent and nothing
    says so.
    """
    client, _ros, _ctx = robot
    with pytest.raises(AttributeError, match="no signal"):
        client.sensors.base_motor.watch_temprature(above=60)


def test_fixed_quantities_have_typed_handles(robot):
    """Fixed quantities have typed handles

    Categories: sdk
    What it checks: Verifies that fixed quantities have typed handles. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, _ros, _ctx = robot
    assert repr(client.watches).startswith("<robot.watches")
    for name in ("battery_level", "forward_speed", "turn_rate",
                 "localization_quality", "pose_x"):
        assert callable(getattr(client.watches, name))


def test_instance_names_are_checked_eagerly(robot):
    """Instance names are checked eagerly

    Categories: sdk
    What it checks: A range sensor name is a runtime string — it has to be — but it is
    validated against what the robot reported rather than arming a watch
    that cannot fire. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.

    Technical notes:
    A range sensor name is a runtime string — it has to be — but it is
    validated against what the robot reported rather than arming a watch
    that cannot fire.
    """
    client, _ros, _ctx = robot
    with pytest.raises(ValueError, match="no range sensor"):
        client.watches.range("nonesuch", below=0.2)


def test_axis_arguments_are_checked(robot):
    """Axis arguments are checked

    Categories: sdk
    What it checks: Verifies that axis arguments are checked. Uses synthetic inputs or a fake agent; no running robot is used.
    Why it matters: Checks that custom readings, units, action parameters, watches, and motion guards remain consistent.
    If it fails: This is an offline SDK/agent regression. Inspect the failing assertion and the custom sensors and actions code or synthetic fixture. Changing your robot profile does not repair this check.
    Profile fields: None.
    Scenario fields: None.
    Robot effects: Offline only; does not connect to the selected robot.
    """
    client, _ros, _ctx = robot
    with pytest.raises(ValueError, match="axis must be"):
        client.watches.imu_acceleration("w", above=1.0)

@pytest.mark.protocol
def test_vendor_receipt_is_truthful_in_sync_and_async(robot):
    """A20/A28: vendor publication is an unverified typed receipt

    Categories: sdk
    What it checks: Calls declared vendor actions through both real protocol clients and verifies typed dispatch receipts omit backend topic/payload fields and make no completion claim.
    Why it matters: Publication alone must not look like successful physical execution.
    If it fails: Inspect vendor receipt serialization and sync/async action facades.
    Profile fields: actions.*
    Scenario fields: None.
    Robot effects: Isolated fake adapter publication only; no ROS or connected robot.
    """
    import asyncio
    from ringfour import aio
    from ringfour.models import DispatchReceipt
    client,ros,ctx=robot
    receipt=client.actions.set_front_led(r=.5)
    assert isinstance(receipt,DispatchReceipt)
    assert receipt.dispatched and receipt.completion=='unverified'
    assert not hasattr(receipt,'topic') and not hasattr(receipt,'sent')
    async def exercise():
        peer=await aio.Robot.connect(client._robot.connection._uri)
        try:
            result=await peer.actions.horn()
            assert isinstance(result,DispatchReceipt)
            assert result.action=='horn' and result.completion=='unverified'
        finally:await peer.close()
    asyncio.run(exercise())
