"""Teleop scenarios and frame aliases, independent of robot geometry."""
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest
from ringfour import Robot
from conformance import SDK_DIR

pytestmark = pytest.mark.sim


def speed(robot):
    v = robot.get_velocity()
    return math.sqrt(sum(x*x for x in (v.linear.x, v.linear.y, v.linear.z)))


def drive(robot, session, recipe):
    from movement import MotionTrace, drive_segment, motion_pose
    return drive_segment(robot, session, recipe["command"], recipe["duration"],
                         recipe["watchdog"], MotionTrace(motion_pose(robot)))


@pytest.mark.requires("teleop", "mobility")
@pytest.mark.motion
def test_teleop_drives_robot_and_pose_changes(robot, teleop_recipe):
    """Teleop produces measurable movement

    Categories: movement
    What it checks: Sends the scenario's velocity command for its duration, verifies movement, stops, and checks displacement and final speed against the scenario's limits.
    Why it matters: A command that reports success without moving the robot is not enough.
    If it fails: Check the drive command topic, controller, odometry, limits, and independently reviewed scenario commands. Use measured acceptance limits, not guessed tolerances.
    Profile fields: mobility.cmd_vel;mobility.max_linear;mobility.max_angular;teleop.max_watchdog
    Scenario fields: teleop.command;teleop.duration;teleop.min_distance;teleop.stop_speed;teleop.halt_timeout
    Robot effects: Moves the robot and stops it afterwards.
    """
    r = teleop_recipe
    start = robot.get_pose()
    try:
        with robot.teleop(watchdog=r["watchdog"]) as session:
            drive(robot, session, r)
            assert robot.get_is_moving()
            session.stop()
        deadline = time.monotonic() + r["halt_timeout"]
        while robot.get_is_moving() and time.monotonic() < deadline:
            time.sleep(.05)
        assert start.distance_to(robot.get_pose()) >= r["min_distance"]
        assert speed(robot) <= r["stop_speed"]
    finally:
        robot.stop()


@pytest.mark.requires("teleop", "mobility")
@pytest.mark.motion
def test_watchdog_halts_on_command_silence(robot, teleop_recipe):
    """Robot stops when teleop commands go silent

    Categories: movement,safety
    What it checks: Drives using the configured scenario, confirms speed above the stop threshold, stops sending commands, and checks speed falls below the threshold before the halt deadline.
    Why it matters: A stalled client should not leave the robot driving indefinitely.
    If it fails: Check the configured watchdog limit, command cadence, controller braking, and measured halt deadline. Verify actual velocity feedback.
    Profile fields: teleop.max_watchdog;mobility.cmd_vel
    Scenario fields: teleop.watchdog;teleop.duration;teleop.stop_speed;teleop.halt_timeout
    Robot effects: Moves the robot, deliberately stops commands, then stops it.
    """
    r = teleop_recipe
    try:
        with robot.teleop(watchdog=r["watchdog"]) as session:
            drive(robot, session, r)
            assert speed(robot) > r["stop_speed"], "scenario never produced observable motion"
            deadline = time.monotonic() + r["halt_timeout"]
            while speed(robot) > r["stop_speed"] and time.monotonic() < deadline:
                time.sleep(.05)
            assert speed(robot) <= r["stop_speed"], "watchdog did not halt within scenario deadline"
    finally:
        robot.stop()


@pytest.mark.requires("teleop", "mobility")
@pytest.mark.motion
def test_killed_client_halts_robot_within_watchdog(robot, agent_full, teleop_recipe):
    """Robot stops after its drive client is killed

    Categories: movement,safety
    What it checks: Starts a separate configured teleop driver, observes movement, kills it without a goodbye, and verifies the robot halts within the scenario deadline.
    Why it matters: A disconnected or crashed mission must not keep commanding motion.
    If it fails: Check watchdog/disconnect handling, drive feedback, and independently measured stop thresholds.
    Profile fields: teleop.max_watchdog;safety.stop_on_disconnect
    Scenario fields: teleop.command;teleop.watchdog;teleop.stop_speed;teleop.halt_timeout
    Robot effects: Moves the robot and kills its test driver; issues stop in cleanup.
    """
    import json
    r = teleop_recipe
    env = dict(os.environ, PYTHONPATH=str(SDK_DIR))
    helper = Path(__file__).parent / "helpers" / "drive_forever.py"
    driver = subprocess.Popen([sys.executable, str(helper), agent_full.unix_uri,
                               str(r["watchdog"]), json.dumps(r["command"])], env=env)
    try:
        deadline = time.monotonic() + r["halt_timeout"]
        while speed(robot) <= r["stop_speed"]:
            assert driver.poll() is None, "driver exited before motion"
            assert time.monotonic() < deadline, "scenario never produced observable motion"
            time.sleep(.05)
        driver.kill()
        driver.wait(timeout=5)
        deadline = time.monotonic() + r["halt_timeout"]
        while speed(robot) > r["stop_speed"] and time.monotonic() < deadline:
            time.sleep(.05)
        assert speed(robot) <= r["stop_speed"]
    finally:
        if driver.poll() is None:
            driver.kill()
        driver.wait(timeout=5)
        robot.stop()


@pytest.mark.requires("mobility")
def test_tf_wrapper_against_live_tree(robot):
    """Position agrees with the transform tree

    Categories: movement,description
    What it checks: Waits for the configured base frame, resolves odometry-to-base aliases, and compares the base pose through TF with the reported pose within 0.05 metres.
    Why it matters: Incorrect frame aliases or stale transforms make movement targets and feedback disagree.
    If it fails: Check frames.base, frames.odom, odometry, TF publishers, and consistent time sources.
    Profile fields: frames.base;frames.odom;mobility.odom_topic
    Scenario fields: None.
    Robot effects: Reads poses and transforms.
    """
    robot.tf.wait_for("@base", timeout=10)
    transform = robot.tf.lookup("@odom", "@base")
    assert transform.parent and transform.child
    assert transform.child in robot.tf.get_frames()
    pose = robot.get_pose()
    actual = robot.tf.pose_of("@base", in_frame=pose.frame)
    assert actual.frame == pose.frame
    assert actual.distance_to(pose) < .05
