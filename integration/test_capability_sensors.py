"""Read contracts for the sensors a profile advertises, on any morphology."""
import math
import pytest

pytestmark = pytest.mark.sim


def finite_vector(value):
    assert all(math.isfinite(v) for v in (value.x,value.y,value.z))


@pytest.mark.requires("imu")
def test_imu_reading(robot):
    """IMU readings are usable

    Categories: sensors
    What it checks: Reads angular velocity, acceleration, timestamp, and frame; checks finite values and a unit orientation quaternion when orientation is available.
    Why it matters: Orientation and motion estimates need well-formed measurements.
    If it fails: Check the IMU topic, publisher, frame, and quaternion normalization. An IMU without orientation should declare that through its covariance.
    Profile fields: imu.topic
    Scenario fields: None.
    Robot effects: Reads IMU data.
    """
    reading = robot.imu()
    assert reading.frame and reading.stamp >= 0
    finite_vector(reading.angular_velocity)
    finite_vector(reading.linear_acceleration)
    if reading.orientation is not None:
        q = reading.orientation
        assert sum(v*v for v in (q.x,q.y,q.z,q.w)) == pytest.approx(1, abs=1e-3)


@pytest.mark.requires("range_sensors")
def test_every_range_sensor(robot, range_name):
    """Range sensors report consistent measurements

    Categories: sensors
    What it checks: Reads each named range sensor and checks its frame, timestamp, distance bounds, beam width, and non-NaN reading. Infinity is a valid no-return reading.
    Why it matters: Distance sensors must expose consistent units and bounds before obstacle logic can use them.
    If it fails: Check the configured sensor topic and its range limits. Diagnose missing or malformed messages at the driver.
    Profile fields: range_sensors.*.topic
    Scenario fields: None.
    Robot effects: Reads range data.
    """
    reading = robot.range(range_name)
    assert reading.frame and reading.stamp >= 0
    assert reading.max_range >= reading.min_range >= 0
    assert not math.isnan(reading.range)
    assert reading.field_of_view >= 0


@pytest.mark.requires("force_torque", "groups")
def test_every_wrist_force_torque(robot, wrench_name):
    """Wrist force and torque readings are finite

    Categories: sensors,manipulation
    What it checks: Reads every declared wrist sensor and checks all force/torque components, frame, and timestamp.
    Why it matters: Contact control needs meaningful force and torque data.
    If it fails: Check the force/torque topic, matching group name, sensor driver, and frame.
    Profile fields: force_torque.*.topic;groups
    Scenario fields: None.
    Robot effects: Reads force/torque data.
    """
    reading = robot.groups[wrench_name].wrench()
    finite_vector(reading.force)
    finite_vector(reading.torque)
    assert reading.frame and reading.stamp >= 0


@pytest.mark.requires("sensors")
def test_vendor_sensor_readings(robot, sensor_name, welcome):
    """Custom sensor signals have data and units

    Categories: sensors
    What it checks: Reads every signal in each custom sensor group. Checks that advertised signal names match the available signals, readings exist, timestamps are valid, and units match the handshake.
    Why it matters: A declared custom signal with no publisher or wrong units is misleading to callers.
    If it fails: Check the signal topic, ROS message type, driver output, and units. A declared signal with no data is a failure.
    Profile fields: sensors.*.*.topic;sensors.*.*.type;sensors.*.*.units
    Scenario fields: None.
    Robot effects: Reads custom signals.
    """
    sensors = getattr(robot.sensors, sensor_name)
    assert set(sensors.signals) == set(welcome.sensors[sensor_name])
    for signal in sensors.signals:
        reading = sensors.reading(signal)
        assert reading["value"] is not None, f"advertised signal {sensor_name}.{signal} has no data"
        assert reading["stamp"] >= 0
        assert reading.get("units") == welcome.sensors[sensor_name][signal].get("units")


@pytest.mark.requires("status")
def test_status_interfaces(robot, profile_data, request):
    """Battery and diagnostics sources respond

    Categories: sensors
    What it checks: Uses the selected YAML to read its configured battery and diagnostics sources, and checks that emergency-stop state is a Boolean. Unknown battery percentage is allowed.
    Why it matters: Status should reflect the sources configured for this robot.
    If it fails: Select the agent's original YAML, verify status topics and publishers, and inspect missing battery or diagnostics data.
    Profile fields: status.battery;status.diagnostics
    Scenario fields: None.
    Robot effects: Reads status.
    """
    from conformance import skip_coverage
    if profile_data is None:
        skip_coverage(request, "not_configured", "status sources are not in the handshake; supply profile YAML")
    config = profile_data["status"]
    if config.get("battery"):
        reading = robot.get_battery()
        assert reading is not None, "configured battery source has no data"
        assert reading.level is None or math.isfinite(reading.level)
    if config.get("diagnostics"):
        assert robot.diagnostics() is not None
    assert isinstance(robot.get_estop_engaged(),bool)


@pytest.mark.requires("environment")
def test_environment_sources(robot, profile_data, request):
    """Environment sensors publish finite values

    Categories: sensors
    What it checks: Reads temperature, pressure, and illuminance channels configured in the selected YAML; checks values and timestamps.
    Why it matters: Environment measurements should be usable numbers rather than absent or corrupt data.
    If it fails: Select the correct YAML and check each configured environment topic and driver.
    Profile fields: environment.temperature;environment.pressure;environment.illuminance
    Scenario fields: None.
    Robot effects: Reads environment data.
    """
    from conformance import skip_coverage
    if profile_data is None:
        skip_coverage(request, "not_configured", "environment channels need independent profile YAML")
    fields = {"temperature":"temperature", "pressure":"pressure", "illuminance":"illuminance"}
    for method,field in fields.items():
        if profile_data["environment"].get(method):
            reading = getattr(robot,method)()
            assert math.isfinite(getattr(reading,field))
            assert reading.stamp >= 0


@pytest.mark.requires("power")
def test_power_lane_metadata(robot, power_name):
    """Power lanes expose their state

    Categories: sensors
    What it checks: Reads every named power lane and checks its identity and Boolean on/off state.
    Why it matters: Operators need to inspect a rail without switching it.
    If it fails: Check lane names, the power-state source, and the power controller's reported state.
    Profile fields: power.lanes
    Scenario fields: None.
    Robot effects: Reads power state; does not switch rails.
    """
    lane = robot.power_lanes[power_name]
    assert lane.name == power_name
    # Reading the state does not switch a power rail.
    assert isinstance(lane.state().on,bool)


@pytest.mark.requires("actions")
@pytest.mark.motion
def test_vendor_action_changes_feedback(robot, action_name, welcome, request, scenarios, record_property):
    """Custom action produces its configured feedback

    Categories: profile
    What it checks: Resolves the advertised action, invokes its configured arguments, waits for the expected measured feedback change and restores using configured arguments when supplied. Missing execution fixtures remain untested.
    Why it matters: Missing action bindings should be found before a mission tries to use them.
    If it fails: Check the action binding, configured arguments, measured feedback source and expected change, and restoration arguments.
    Profile fields: actions
    Scenario fields: actions.*.arguments;actions.*.feedback;actions.*.restore_arguments
    Robot effects: Executes the configured action and optional restoration; requires commands enabled.
    """
    assert callable(getattr(robot.actions,action_name))
    assert action_name in welcome.actions
    from conformance import skip_coverage
    fixture=scenarios.get('actions',{}).get(action_name)
    if fixture is None:
        skip_coverage(request, "not_configured", f"Action {action_name} needs arguments and measured feedback in Test settings")
    request.getfixturevalue('motion_enabled')
    import time
    import math
    feedback=fixture['feedback']; tolerance=feedback.get('tolerance',.01)
    from conformance import read_sdk_feedback
    def wait_value(expected=None):
        deadline=time.monotonic()+fixture.get('timeout',10)
        while True:
            value=read_sdk_feedback(robot,feedback['path'])
            if value is not None and math.isfinite(value) and (expected is None or abs(value-expected)<=tolerance):
                return value
            assert time.monotonic()<deadline, f"Action {action_name}: expected feedback {expected}, received {value}"
            time.sleep(.05)
    before=wait_value()
    if abs(before-feedback['expected'])<=tolerance:
        skip_coverage(request,'not_configured','Action feedback already matches target; configure an observable state change')
    started=time.monotonic()
    try:
        result=getattr(robot.actions,action_name)(**fixture.get('arguments',{}))
        assert isinstance(result,dict)
        actual=wait_value(feedback['expected'])
        record_property('action.feedback_path',feedback['path'])
        record_property('action.observed_value',actual)
    finally:
        if 'restore_arguments' in fixture:
            rate=welcome.actions[action_name].get('max_rate')
            if rate and rate>0:
                time.sleep(max(0,1/rate-(time.monotonic()-started)+.05))
            getattr(robot.actions,action_name)(**fixture['restore_arguments'])
            wait_value(before)



@pytest.mark.requires("imu")
def test_optional_magnetometer(robot, profile_data, request):
    """Configured magnetometer readings are finite

    Categories: sensors
    What it checks: If the selected YAML declares a magnetometer, reads its magnetic field and timestamp and checks finite components.
    Why it matters: A configured heading source needs valid measurements.
    If it fails: Check imu.mag_topic and its publisher. An IMU without a magnetometer is not applicable.
    Profile fields: imu.mag_topic
    Scenario fields: None.
    Robot effects: Reads magnetometer data.
    """
    from conformance import skip_coverage
    if profile_data is None:
        skip_coverage(request, "not_configured", "magnetometer presence is not exposed by the handshake")
    if not profile_data["imu"].get("mag_topic"):
        skip_coverage(request, "not_applicable", "profile has no magnetometer source")
    reading = robot.magnetometer()
    finite_vector(reading.magnetic_field)
    assert reading.stamp >= 0
