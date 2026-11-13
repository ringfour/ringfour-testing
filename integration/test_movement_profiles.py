"""Motion profiles using explicit velocity commands and live feedback."""
import math
import time

import pytest
from movement import MotionTrace, drive_segment, linear_speed, wait_stopped, motion_pose, motion_feedback, drive_turn

pytestmark = [pytest.mark.sim, pytest.mark.requires('mobility', 'teleop'), pytest.mark.motion]


@pytest.mark.movement_profile('figure_eight')
def test_figure_eight_closes_both_loops(robot, movement_profile_recipe, record_property):
    """Figure eight closes both loops

    Categories: movement
    What it checks: Drives each measured circle with a bounded response deadline, checks measured yaw for each turn, verifies each loop returns near its starting pose, and checks total odometry path length and stopping.
    Why it matters: Exercises forward motion and both turning directions together, exposing inconsistent steering or odometry.
    If it fails: Check velocity limits, odometry scale, frame continuity, steering response, and configured loop tolerances. Review the space needed for two circles of radius speed divided by turn rate.
    Profile fields: mobility.cmd_vel;mobility.odom_topic;mobility.max_linear;mobility.max_angular
    Scenario fields: movement_profiles.figure_eight
    Robot effects: Drives two circles in opposite directions and stops. Does not return by navigation if a loop misses its start.
    """
    r = movement_profile_recipe
    origin = motion_pose(robot)
    trace = MotionTrace(origin)
    try:
        with robot.teleop(watchdog=r['watchdog']) as session:
            for direction in (1, -1):
                command = {"linear":r["linear"], "angular":direction*r["angular"]}
                beginning = motion_pose(robot)
                initial_yaw = trace.yaw
                rates = drive_turn(robot, session, command, direction*2*math.pi, r['watchdog'], trace)
                record_property("measured.loop_"+str(direction)+"_robot_seconds", rates["robot_seconds"])
                record_property("measured.loop_"+str(direction)+"_wall_seconds", rates["wall_seconds"])
                record_property("measured.loop_"+str(direction)+"_angular_velocity", rates["angular_velocity"])
                angle = trace.yaw - initial_yaw
                closure = beginning.distance_to(motion_pose(robot))
                record_property('measured.loop_' + str(direction) + '_yaw_rad', angle)
                record_property('measured.loop_' + str(direction) + '_closure_m', closure)
                assert abs(angle - direction*2*math.pi) <= r['angle_tolerance'], 'Measured turn differs from one full circle'
                assert closure <= r['position_tolerance'], 'Circle did not close within tolerance'
            session.stop()
            wait_stopped(robot, r)
        record_property('measured.path_m', trace.path)
        assert trace.path >= r['min_path'], 'Insufficient measured travel'
    finally:
        robot.stop()


@pytest.mark.movement_profile('braking')
def test_braking_time_and_distance(robot, movement_profile_recipe, record_property):
    """Braking stays within time and distance limits

    Categories: movement,safety
    What it checks: Drives straight, confirms measurable travel and velocity, sends stop, then measures time and odometry path until speed stays below the configured threshold for 0.2 seconds.
    Why it matters: A successful stop response alone does not show how long or how far the robot continues moving.
    If it fails: Check controller deceleration, speed feedback, command routing, and stopping limits measured for the selected speed and surface.
    Profile fields: mobility.cmd_vel;mobility.odom_topic
    Scenario fields: movement_profiles.braking
    Robot effects: Drives straight and sends stop, with unconditional stop cleanup.
    """
    r = movement_profile_recipe
    trace = MotionTrace(motion_pose(robot))
    try:
        with robot.teleop(watchdog=r['watchdog']) as session:
            drive_segment(robot, session, {'linear':r['linear'], 'angular':0.}, r['duration'], r['watchdog'], trace)
            assert trace.path >= r['min_distance'], 'Drive produced insufficient travel before braking'
            assert linear_speed(robot) > r['stop_speed'], 'Robot was not moving before stop'
            start = time.monotonic()
            start_stamp = motion_feedback(robot).stamp
            braking = MotionTrace(motion_pose(robot))
            session.stop()
            stopped_since = None
            while True:
                now = time.monotonic()
                braking.sample(motion_pose(robot))
                if linear_speed(robot) <= r['stop_speed']:
                    if stopped_since is None:
                        stopped_since = now
                        stopped_stamp = motion_feedback(robot).stamp
                    if now - stopped_since >= .2:
                        break
                else:
                    stopped_since = None
                assert now-start <= r['halt_timeout'], 'Robot did not settle before the stop deadline'
                time.sleep(.02)
            measured = stopped_stamp - start_stamp
            record_property("measured.braking_wall_time_s", stopped_since-start)
            record_property('measured.braking_time_s', measured)
            record_property('measured.braking_distance_m', braking.path)
            assert measured <= r['max_braking_time'], 'Braking exceeded the configured time'
            assert braking.path <= r['max_braking_distance'], 'Braking exceeded the configured distance'
            wait_stopped(robot, r)
    finally:
        robot.stop()


@pytest.mark.movement_profile('odometry_spin')
def test_odometry_spin_rotation_and_drift(robot, movement_profile_recipe, record_property):
    """Odometry spin matches rotation and drift limits

    Categories: movement
    What it checks: Commands an in-place turn until the measured number of revolutions is reached, with a bounded response deadline, unwraps odometry yaw through full rotations, checks angle error and maximum position drift, and verifies stopping.
    Why it matters: A final heading alone cannot distinguish standing still from a full rotation. Accumulated yaw and position drift provide more useful feedback.
    If it fails: Check angular command scaling, odometry yaw, frame stability, and turn response. This tests odometry consistency; external measurements are needed to calibrate true rotation.
    Profile fields: mobility.cmd_vel;mobility.odom_topic;mobility.max_angular
    Scenario fields: movement_profiles.odometry_spin
    Robot effects: Turns in place through the configured revolutions and stops.
    """
    r = movement_profile_recipe
    expected = math.copysign(2*math.pi*r['turns'], r['angular'])
    trace = MotionTrace(motion_pose(robot))
    try:
        with robot.teleop(watchdog=r['watchdog']) as session:
            command = {"linear":0., "angular":r["angular"]}
            rates = drive_turn(robot, session, command, expected, r['watchdog'], trace)
            measured_rotation = trace.yaw
            session.stop()
            wait_stopped(robot, r)
            trace.sample(motion_pose(robot))
        record_property('measured.robot_seconds', rates['robot_seconds'])
        record_property('measured.wall_seconds', rates['wall_seconds'])
        record_property('measured.angular_velocity', rates['angular_velocity'])
        record_property('measured.rotation_rad', measured_rotation)
        record_property('measured.max_drift_m', trace.drift)
        assert abs(measured_rotation-expected) <= r['angle_tolerance'], 'Accumulated rotation missed the expected angle'
        assert trace.drift <= r['max_drift'], 'Position drift exceeded the configured tolerance'
    finally:
        robot.stop()
