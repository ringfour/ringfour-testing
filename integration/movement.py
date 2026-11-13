"""Selectable movement recipes and feedback measurements; no commands on import."""
import math
import copy
import time

DEFAULT_TELEOP = {'command':{'linear':.3, 'angular':0.}, 'duration':2., 'watchdog':.5,
                  'min_distance':.2, 'stop_speed':.05, 'halt_timeout':2.5}
DRIVE_FIELDS = [('command.linear','Forward speed · m/s',.3),('command.angular','Turn speed · rad/s',0.),
                ('duration','Drive duration · seconds',2.),('min_distance','Minimum travel · metres',.2),
                ('watchdog','Command watchdog · seconds',.5),('stop_speed','Stopped speed · m/s',.05),
                ('halt_timeout','Stop deadline · seconds',2.5)]
COMMON = [
    ('watchdog', 'Command watchdog · seconds', .3),
    ('stop_speed', 'Stopped speed tolerance · m/s', .05),
    ('stop_angular_speed', 'Stopped turn tolerance · rad/s', .05),
    ('halt_timeout', 'Stop deadline · seconds', 3.),
]
MOVEMENT_PROFILES = [
    {'id':'figure_eight', 'label':'Figure eight', 'description':'Drive two complete circles in opposite directions. Check each loop closes, the turns follow their commanded direction, and the path covers the expected distance.',
     'fields':[('linear','Forward speed · m/s',.3),('angular','Turn speed magnitude · rad/s',.6),('position_tolerance','Loop closure tolerance · metres',.2),('angle_tolerance','Turn angle tolerance · radians',.3),('min_path','Minimum total path · metres',4.)] + COMMON},
    {'id':'braking', 'label':'Braking time', 'description':'Drive straight, send stop, and measure the time and distance until velocity stays below the stopped threshold.',
     'fields':[('linear','Forward speed · m/s',.3),('duration','Drive duration · seconds',2.),('min_distance','Minimum expected travel · metres',.2),('max_braking_time','Maximum braking time · seconds',2.5),('max_braking_distance','Maximum braking distance · metres',.4)] + COMMON},
    {'id':'odometry_spin', 'label':'Odometry spin', 'description':'Turn in place through a configured number of revolutions. Check accumulated odometry yaw and maximum position drift. This checks odometry consistency, not external calibration.',
     'fields':[('angular','Turn speed · rad/s',.6),('turns','Number of revolutions',1.),('angle_tolerance','Rotation tolerance · radians',.3),('max_drift','Maximum position drift · metres',.1)] + COMMON},
]



# Precision limits only. Command speeds, minimum movement and stop deadlines
# remain independent so this setting cannot turn missing motion into a pass.
PRECISION_FIELDS = {"figure_eight": ("position_tolerance", "angle_tolerance"),
                    "odometry_spin": ("max_drift", "angle_tolerance")}


def validate_movement_tolerance(value):
    from conformance import number
    factor = number(value, "movement_tolerance", minimum=1)
    if factor > 5:
        raise ValueError("movement_tolerance must be between 1 and 5")
    return factor


def effective_movement_recipe(name, recipe, factor=1):
    factor = validate_movement_tolerance(factor)
    result = copy.deepcopy(recipe)
    for field in PRECISION_FIELDS.get(name, ()):
        result[field] *= factor
    return result


def default_scenarios(welcome, profile=None):
    """Built-in velocity tests; applicable interfaces are identified by capability."""
    data = {'version':1, 'movement_tolerance':1, 'debug':True, 'profile':{'name':welcome.profile.name, 'model':welcome.profile.model,
                                  'version':welcome.profile.version}}
    if {'mobility','teleop'} <= set(welcome.capabilities):
        data['teleop'] = copy.deepcopy(DEFAULT_TELEOP)
        data['movement_profiles'] = {p['id']:{key:value for key,_,value in p['fields']} for p in MOVEMENT_PROFILES}
    if profile and 'teleop' in data:
        # Known command limits are configuration, not morphology. Keep the
        # preset geometry while deriving achievable speeds and travel checks.
        limits = profile.get('mobility', {}) or {}
        def bounded(value, key):
            bound = limits.get(key)
            return min(value, bound) if isinstance(bound,(int,float)) and math.isfinite(bound) and bound > 0 else value
        drive = data['teleop']
        drive['command']['linear'] = bounded(drive['command']['linear'],'max_linear')
        drive['min_distance'] = min(drive['min_distance'], drive['command']['linear']*drive['duration']*.5)
        drive['stop_speed'] = min(drive['stop_speed'], drive['command']['linear']*.25)
        for name, recipe in data['movement_profiles'].items():
            for axis, key in (('linear','max_linear'),('angular','max_angular')):
                if axis in recipe: recipe[axis] = bounded(recipe[axis],key)
            if 'linear' in recipe:
                recipe['stop_speed'] = min(recipe['stop_speed'],recipe['linear']*.25)
            if 'angular' in recipe:
                recipe['stop_angular_speed'] = min(recipe['stop_angular_speed'],recipe['angular']*.25)
            if name == 'figure_eight':
                recipe['min_path'] = min(recipe['min_path'], 4*math.pi*recipe['linear']/recipe['angular']*.6)
            elif name == 'braking':
                recipe['min_distance'] = min(recipe['min_distance'],recipe['linear']*recipe['duration']*.5)
    if profile and (profile.get('mobility') or {}).get('nav2_navigate_to_pose'):
        from navigation import NAVIGATION_FIELDS
        data['navigation'] = {'auto':True, **{key:value for key,_,value in NAVIGATION_FIELDS}}
    return data

def validate_movement_profiles(recipes, capabilities):
    from conformance import number
    if not isinstance(recipes, dict):
        raise ValueError('movement_profiles must be a mapping')
    if recipes and not {'mobility','teleop'} <= set(capabilities):
        raise ValueError('Movement profiles require mobility and teleop capabilities')
    catalog = {p['id']:p for p in MOVEMENT_PROFILES}
    for name, recipe in recipes.items():
        if name not in catalog:
            raise ValueError(f'Unknown movement profile {name!r}')
        fields = {f[0] for f in catalog[name]['fields']}
        if not isinstance(recipe, dict) or set(recipe) != fields:
            raise ValueError(f'{name}: configure exactly these fields: {", ".join(sorted(fields))}')
        for field in fields:
            number(recipe[field], f'movement_profiles.{name}.{field}', positive=field not in ('linear','angular'))
        for field in ('linear','angular'):
            if field in recipe and recipe[field] == 0:
                raise ValueError(f'{name}.{field} must exercise motion')
        if name == 'figure_eight' and recipe['angular'] <= 0:
            raise ValueError('figure_eight.angular must be a positive magnitude')
        if recipe['watchdog'] > 1:
            raise ValueError(f'{name}.watchdog must be <= 1 second')
        if recipe['stop_speed'] >= abs(recipe.get('linear', float('inf'))):
            raise ValueError(f'{name}.stop_speed must be below its drive speed')
        if name == 'braking' and recipe['max_braking_time'] > recipe['halt_timeout']:
            raise ValueError('Braking acceptance time must fit within the stop deadline')
    return recipes


def linear_speed(robot):
    v = robot.get_velocity().linear
    return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)


class MotionTrace:
    def __init__(self, pose):
        self.origin = self.previous = pose
        self.path = self.yaw = self.drift = 0.

    def sample(self, pose):
        if pose.frame != self.origin.frame:
            raise AssertionError('Odometry frame changed during the movement test')
        self.path += self.previous.distance_to(pose)
        delta = pose.yaw - self.previous.yaw
        self.yaw += math.atan2(math.sin(delta), math.cos(delta))
        self.drift = max(self.drift, self.origin.distance_to(pose))
        self.previous = pose


def motion_feedback(robot):
    state = robot.get_mobility_state()
    if state.stamp is None or state.odometry_pose is None:
        raise AssertionError('Agent needs timestamped odometry support; update the agent before motion tests')
    if not math.isfinite(state.stamp):
        raise AssertionError('Odometry timestamp is not finite')
    return state


def motion_pose(robot):
    return motion_feedback(robot).odometry_pose


def drive_segment(robot, session, command, duration, watchdog, trace):
    first = motion_feedback(robot)
    start = previous = first.stamp
    wall_start = last_update = time.monotonic()
    while True:
        session.drive(**command)
        feedback = motion_feedback(robot)
        stamp = feedback.stamp
        if stamp < previous:
            raise AssertionError('Odometry clock moved backwards during motion')
        if stamp > previous:
            last_update = time.monotonic()
        elif time.monotonic() - last_update > 3:
            raise AssertionError('Odometry clock stopped advancing during motion')
        previous = stamp
        trace.sample(feedback.odometry_pose)
        if stamp-start >= duration:
            break
        time.sleep(min(.05, watchdog / 4))
    return {'robot_seconds':stamp-start, 'wall_seconds':time.monotonic()-wall_start,
            'linear_velocity':feedback.velocity.linear.x, 'angular_velocity':feedback.velocity.angular.z}


def wait_stopped(robot, recipe):
    deadline = time.monotonic() + recipe['halt_timeout']
    while time.monotonic() < deadline:
        v = robot.get_velocity()
        linear = math.sqrt(v.linear.x**2 + v.linear.y**2 + v.linear.z**2)
        angular = math.sqrt(v.angular.x**2 + v.angular.y**2 + v.angular.z**2)
        if linear <= recipe['stop_speed'] and angular <= recipe['stop_angular_speed']:
            return
        time.sleep(.05)
    raise AssertionError('Robot did not stop before the configured deadline')



def drive_turn(robot, session, command, angle, watchdog, trace):
    """Measure a complete turn from odometry, with a bounded response budget.

    Acceleration and wheel slip mean speed times elapsed time is not a
    guaranteed angle. Stop at the measured angle; still reject a stalled,
    reversed, or excessively slow response. External calibration is separate.
    """
    first = motion_feedback(robot)
    start = previous = first.stamp
    initial_yaw = trace.yaw
    wall_start = last_update = time.monotonic()
    budget = abs(angle/command['angular'])*1.5 + 2.
    direction = math.copysign(1., angle)
    while direction*(trace.yaw-initial_yaw) < abs(angle):
        session.drive(**command)
        feedback = motion_feedback(robot)
        stamp = feedback.stamp
        assert stamp >= previous, 'Odometry clock moved backwards during motion'
        if stamp > previous:
            last_update = time.monotonic()
        assert time.monotonic()-last_update <= 3., 'Odometry clock stopped advancing during motion'
        previous = stamp
        trace.sample(feedback.odometry_pose)
        assert stamp-start <= budget, 'Accumulated rotation did not reach its target before the response deadline'
        time.sleep(min(.02, watchdog/4))
    return {'robot_seconds':stamp-start, 'wall_seconds':time.monotonic()-wall_start,
            'linear_velocity':feedback.velocity.linear.x, 'angular_velocity':feedback.velocity.angular.z}
