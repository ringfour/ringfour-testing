"""Morphology-independent discovery and explicit physical test scenarios.

The handshake is authoritative. A YAML can independently verify its claims;
a scenario file supplies targets a live profile cannot describe (safe space,
known collision configurations, and tolerances). Discovery never commands motion.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SDK_DIR = ROOT / "ringfour-sdk"
STRUCTURAL_KEYS = {"name", "model", "version", "protocol", "frames", "safety", "joint_states"}


@dataclass(frozen=True)
class AgentEndpoints:
    unix_socket: str = ""
    tcp_port: int = 0
    external_tcp_uri: str = ""

    @property
    def unix_uri(self):
        return f"unix://{self.unix_socket}" if self.unix_socket else self.tcp_uri

    @property
    def tcp_uri(self):
        return self.external_tcp_uri or (f"ws://127.0.0.1:{self.tcp_port}" if self.tcp_port else self.unix_uri if self.unix_socket else "")


def external_endpoints(unix_var="RINGFOUR_AGENT_UNIX", tcp_var="RINGFOUR_AGENT_TCP"):
    from urllib.parse import urlparse
    unix = os.environ.get(unix_var, "")
    tcp = os.environ.get(tcp_var, "")
    if unix and not tcp and os.environ.get('RINGFOUR_CONFORMANCE'):
        tcp=read_yaml(os.environ['RINGFOUR_CONFORMANCE']).get('connection',{}).get('tcp_uri','')
    if not unix and not tcp:
        return None
    return AgentEndpoints(unix, urlparse(tcp).port or 0, tcp)


def read_yaml(path):
    data = yaml.safe_load(Path(path).expanduser().read_text())
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def discover(uri, timeout=5):
    from ringfour.models.protocol import Welcome
    from ringfour.transport import Connection
    connection = Connection(uri, timeout=timeout)
    try:
        return Welcome.model_validate(connection.call("hello", {"protocol": 1, "client_version": "conformance"}, timeout=timeout))
    finally:
        connection.close()


def expected_capabilities(profile):
    return {key for key, value in profile.items() if key not in STRUCTURAL_KEYS and value is not None}


def instance_names(welcome, kind):
    if kind == "capabilities":
        return list(welcome.capabilities)
    if kind == "groups":
        return list(welcome.groups)
    return list(welcome.instances.get(kind, ()))


def number(value, name, *, positive=False, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{name}: expected a finite number")
    if (positive and value <= 0) or (minimum is not None and value < minimum):
        raise ValueError(f"{name}: value is out of range")
    return float(value)


def validate_scenarios(data, welcome):
    """Reject incompatible recipes BEFORE any test acquires a motion session."""
    if not data:
        return {}
    if data.get("version") != 1:
        raise ValueError("conformance.version must be 1")
    identity = data.get("profile", {})
    if identity.get("name") != welcome.profile.name or identity.get("model") != welcome.profile.model:
        raise ValueError("conformance.profile name/model must match the loaded agent")
    if "description_sha" in identity and identity["description_sha"] != welcome.description_sha:
        raise ValueError("conformance.profile.description_sha does not match the live description")
    if "version" in identity and str(identity["version"]) != welcome.profile.version:
        raise ValueError("conformance.profile.version does not match the loaded profile")
    allowed = {"version", "profile", "teleop", "navigation", "groups", "debug", "mapping", "tolerances", "description_change", "movement_profiles", "movement_tolerance", "artifacts", "actions", "connection", "runtime", "lifecycle", "mobility_fault"}
    if set(data) - allowed:
        raise ValueError(f"Unknown conformance sections: {sorted(set(data) - allowed)}")
    connection=data.get('connection',{})
    if not isinstance(connection,dict) or set(connection)-{'tcp_uri'}:
        raise ValueError('Connection only accepts tcp_uri')
    if 'tcp_uri' in connection:
        from urllib.parse import urlparse
        uri=urlparse(connection['tcp_uri'])
        if uri.scheme not in ('ws','wss') or not uri.hostname or not uri.port:
            raise ValueError('Additional TCP address must be ws://host:port or wss://host:port')
    runtime=data.get('runtime',{})
    if not isinstance(runtime,dict) or set(runtime)-{'test_timeout'}:
        raise ValueError('Runtime only accepts test_timeout')
    if 'test_timeout' in runtime:
        number(runtime['test_timeout'],'runtime.test_timeout',minimum=1)
    artifacts = data.get("artifacts", {})
    if not isinstance(artifacts, dict) or set(artifacts) - {"snapshot_dir"}:
        raise ValueError("Artifacts only accepts snapshot_dir")
    if 'snapshot_dir' in artifacts and (not isinstance(artifacts['snapshot_dir'],str) or not artifacts['snapshot_dir'].strip()):
        raise ValueError("Snapshot directory must be a nonempty path")
    actions = data.get('actions',{})
    if not isinstance(actions,dict):
        raise ValueError('Actions must be a mapping')
    for name, fixture in actions.items():
        if name not in welcome.actions:
            raise ValueError(f'Unknown action fixture {name}')
        if not isinstance(fixture,dict) or set(fixture)-{'arguments','restore_arguments','feedback','timeout'}:
            raise ValueError(f'Action {name}: unknown fixture fields')
        for key in ('arguments','restore_arguments'):
            if key in fixture:
                if not isinstance(fixture[key],dict):
                    raise ValueError(f'Action {name}.{key} needs named arguments')
                parameters=welcome.actions[name].get('parameters',{})
                if set(fixture[key])-set(parameters):
                    raise ValueError(f'Action {name}: unknown arguments')
                for argument,spec in parameters.items():
                    if argument not in fixture[key] and spec.get('default') is None:
                        raise ValueError(f'Action {name}: configure argument {argument}')
                for argument,value in fixture[key].items():
                    spec=parameters[argument];kind=spec.get('type','float')
                    if kind in ('float','int'):
                        number(value,f'actions.{name}.{argument}')
                        if kind=='int' and not isinstance(value,int):
                            raise ValueError(f'Action {name}.{argument} must be an integer')
                        if spec.get('min') is not None and value<spec['min'] or spec.get('max') is not None and value>spec['max']:
                            raise ValueError(f'Action {name}.{argument} outside declared bounds')
                    elif kind=='bool' and not isinstance(value,bool) or kind=='string' and not isinstance(value,str):
                        raise ValueError(f'Action {name}.{argument} has the wrong type')

        feedback=fixture.get('feedback')
        if not isinstance(feedback,dict) or set(feedback)-{'path','expected','tolerance'} or not isinstance(feedback.get('path'),str) or not feedback['path'].strip():
            raise ValueError(f'Action {name}: configure feedback path and expected value')
        number(feedback.get('expected'),f'actions.{name}.feedback.expected')
        number(feedback.get('tolerance',.01),f'actions.{name}.feedback.tolerance',minimum=0)
        number(fixture.get('timeout',10),f'actions.{name}.timeout',positive=True)
    if "debug" in data and not isinstance(data["debug"], bool):
        raise ValueError("conformance.debug must be true or false")
    for section in ("teleop", "navigation", "groups", "mapping", "tolerances"):
        if section in data and not isinstance(data[section], dict):
            raise ValueError(f"conformance.{section}: expected a mapping")
    from movement import validate_movement_profiles, validate_movement_tolerance
    validate_movement_tolerance(data.get("movement_tolerance", 1))
    validate_movement_profiles(data.get("movement_profiles", {}), welcome.capabilities)
    tolerances = data.get("tolerances", {})
    if set(tolerances) - {"joint_limits", "fk_position", "fk_quaternion"}:
        raise ValueError("Unknown conformance tolerance")
    joints = {j for g in welcome.groups.values() for j in g.joints}
    for joint, value in tolerances.get("joint_limits", {}).items():
        if joint not in joints:
            raise ValueError(f"Unknown joint tolerance {joint!r}")
        number(value, f"tolerances.joint_limits.{joint}", minimum=0)
    for key in ("fk_position", "fk_quaternion"):
        if key in tolerances:
            number(tolerances[key], f"tolerances.{key}", positive=True)
    for name, recipe in data.get("groups", {}).items():
        if name not in welcome.groups:
            raise ValueError(f"Unknown scenario group {name!r}")
        info = welcome.groups[name]
        allowed_group = {"waypoints", "clear_configurations", "collision_configurations", "dt",
                         "settle", "tolerance", "max_check_seconds", "tool", "allow_sessions", "clear_state", "collision_state", "restore_timeout", "probe", "fixture_provenance", "hardening"}
        if not isinstance(recipe, dict) or set(recipe) - allowed_group:
            raise ValueError(f"groups.{name}: unknown fields or invalid mapping")
        for field in ("clear_state", "collision_state"):
            if field in recipe:
                if not isinstance(recipe[field], dict):
                    raise ValueError(f"groups.{name}.{field}: expected joint positions")
                for joint,value in recipe[field].items():
                    number(value, f"groups.{name}.{field}.{joint}")
        if "hardening" in recipe:
            hardening = recipe["hardening"]
            if not isinstance(hardening, dict) or set(hardening) - {"lease_expiry", "client_death", "feedback_loss"}:
                raise ValueError(f"groups.{name}.hardening: unknown execution fault")
            for flag in ("lease_expiry", "client_death"):
                if flag in hardening and type(hardening[flag]) is not bool:
                    raise ValueError(f"groups.{name}.hardening.{flag}: expected a boolean")
            if "feedback_loss" in hardening:
                fault = hardening["feedback_loss"]
                allowed = {"reviewed", "command", "restore_command", "recovery_command", "timeout"}
                if not isinstance(fault, dict) or set(fault) - allowed:
                    raise ValueError(f"groups.{name}.hardening.feedback_loss: invalid fixture")
                if type(fault.get("reviewed")) is not bool:
                    raise ValueError("feedback loss needs an explicit reviewed flag")
                if data.get("debug") is not True:
                    raise ValueError("feedback loss requires explicit debug eligibility")
                for field in ("command", "restore_command", "recovery_command"):
                    command = fault.get(field)
                    if not isinstance(command, list) or not command or any(not isinstance(v,str) or not v for v in command):
                        raise ValueError(f"feedback_loss.{field}: expected an explicit nonempty argv list")
                if "timeout" in fault:
                    number(fault["timeout"], "feedback_loss.timeout", positive=True)
        if "fixture_provenance" in recipe:
            provenance=recipe["fixture_provenance"]
            keys={"description_sha","geometry_sha256","oracle","oracle_version","model_sha256","method","adapter_sha256","reviewed","reviewed_at","candidate_ids"}
            if not isinstance(provenance,dict) or set(provenance)-keys:
                raise ValueError(f"groups.{name}.fixture_provenance: invalid evidence metadata")
            if provenance.get('description_sha')!=welcome.description_sha or provenance.get('reviewed') is not True:
                raise ValueError(f"groups.{name}: fixture evidence must be reviewed for the live model")
            if any(not isinstance(value,str) for key,value in provenance.items() if key not in ('reviewed','candidate_ids')):
                raise ValueError('Fixture evidence fields must be strings')
            if not isinstance(provenance.get('candidate_ids'),list) or any(not isinstance(value,str) for value in provenance['candidate_ids']):
                raise ValueError('Fixture evidence needs candidate identifiers')
        if "probe" in recipe:
            probe = recipe["probe"]
            if not isinstance(probe, dict) or set(probe) - {"mode", "reviewed", "angular_step", "linear_step"}:
                raise ValueError(f"groups.{name}.probe: invalid testing policy")
            if probe.get("mode") not in ("read", "hold", "small", "manual"):
                raise ValueError(f"groups.{name}.probe: unknown mode")
            if "reviewed" in probe and not isinstance(probe["reviewed"], bool):
                raise ValueError(f"groups.{name}.probe.reviewed must be boolean")
            for key in ("angular_step", "linear_step"):
                if key in probe:
                    number(probe[key], f"groups.{name}.probe.{key}", positive=True)
            if probe.get("mode") in ("hold", "small") and "trajectory" not in info.control_modes:
                raise ValueError(f"groups.{name}: selected mode needs a trajectory controller")
        if "allow_sessions" in recipe and not isinstance(recipe["allow_sessions"], bool):
            raise ValueError(f"groups.{name}.allow_sessions must be boolean")
        for field in ("waypoints", "clear_configurations", "collision_configurations"):
            values = recipe.get(field, [])
            if not isinstance(values, list):
                raise ValueError(f"groups.{name}.{field}: expected a list")
            for vector in values:
                validate_vector(vector, info.joints, f"groups.{name}.{field}")
        for key in ("dt", "settle", "tolerance", "max_check_seconds", "restore_timeout"):
            if key in recipe:
                number(recipe[key], f"groups.{name}.{key}", positive=True)
        if "waypoints" in recipe and (len(recipe["waypoints"]) < 2 or "trajectory" not in info.control_modes):
            raise ValueError(f"groups.{name}: motion needs >=2 waypoints and a trajectory controller")
        if "tool" in recipe:
            if info.tool is None or info.tool.joint is None or info.tool.open is None or info.tool.closed is None:
                raise ValueError(f"groups.{name}.tool: no position tool declared")
            tool = recipe["tool"]
            if not isinstance(tool, dict) or set(tool) - {"settle", "tolerance", "blocked_target", "stall_command", "stall_restore_command", "timeout"}:
                raise ValueError(f"groups.{name}.tool: unknown fields or invalid mapping")
            if "stall_command" in tool:
                for key in ("stall_command", "stall_restore_command"):
                    command = tool.get(key)
                    if not isinstance(command, list) or not command or any(not isinstance(v,str) for v in command):
                        raise ValueError(f"tool.{key} must be an explicit argv list")
                if data.get("debug") is not True:
                    raise ValueError("Simulated tool stall requires explicit debug configuration")
            if "timeout" in tool:
                number(tool["timeout"], f"groups.{name}.tool.timeout", positive=True)
            if "blocked_target" in tool:
                number(tool["blocked_target"], f"groups.{name}.tool.blocked_target")
            for key in ("settle", "tolerance"):
                if key in recipe["tool"]:
                    number(recipe["tool"][key], f"groups.{name}.tool.{key}", positive=True)
    if "teleop" in data:
        recipe = data["teleop"]
        if set(recipe) - {"command", "duration", "watchdog", "min_distance", "stop_speed", "halt_timeout", "watch_stop"}:
            raise ValueError("Unknown teleop scenario field")
        if not {"teleop", "mobility"} <= set(welcome.capabilities):
            raise ValueError("Teleop scenario requires mobility and teleop capabilities")
        command = recipe.get("command", {})
        if not command or set(command) - {"linear", "angular", "lateral"}:
            raise ValueError("teleop.command must use supported linear/angular/lateral axes")
        for key, value in command.items():
            number(value, f"teleop.command.{key}")
        if not any(command.values()):
            raise ValueError("teleop.command must exercise motion")
        for key in ("duration", "watchdog", "min_distance", "stop_speed", "halt_timeout"):
            number(recipe.get(key), f"teleop.{key}", positive=True)
        if recipe["watchdog"] > 1:
            raise ValueError("teleop.watchdog must be <= 1 second")
        if "watch_stop" in recipe:
            condition = recipe["watch_stop"]
            if not isinstance(condition, dict) or not isinstance(condition.get("path"), str) or not condition["path"]:
                raise ValueError("teleop.watch_stop needs a watch path")
            if set(condition) - {"path", "above", "below"} or not ({"above", "below"} & set(condition)):
                raise ValueError("teleop.watch_stop needs above/below thresholds")
            for key in ("above", "below"):
                if key in condition:
                    number(condition[key], f"teleop.watch_stop.{key}")
    if "navigation" in data:
        if "mobility" not in welcome.capabilities:
            raise ValueError("Navigation scenario requires mobility")
        recipe = data["navigation"]
        if set(recipe) - {"goals", "unreachable_goals", "timeout", "tolerance", "min_distance", "halt_timeout", "auto", "clearance", "max_distance"}:
            raise ValueError("Unknown navigation scenario field")
        if 'auto' in recipe and not isinstance(recipe['auto'],bool):
            raise ValueError('navigation.auto must be boolean')
        if recipe.get('auto') and recipe.get('min_distance',.75) >= recipe.get('max_distance',2.):
            raise ValueError('Navigation minimum distance must be below maximum distance')
        for field in ("goals", "unreachable_goals"):
            for pose in recipe.get(field, []):
                validate_pose(pose, f"navigation.{field}")
        for key in ("timeout", "tolerance", "min_distance", "halt_timeout", "clearance", "max_distance"):
            if key in recipe:
                number(recipe[key], f"navigation.{key}", positive=True)
    mapping = data.get("mapping", {})
    if set(mapping) - {"save_localize"} or ("save_localize" in mapping and not isinstance(mapping["save_localize"], bool)):
        raise ValueError("mapping only accepts save_localize: true/false")
    if "description_change" in data:
        recipe = data["description_change"]
        if not isinstance(recipe, dict) or set(recipe) - {"command", "restore_command", "timeout"}:
            raise ValueError("description_change: unknown fields or invalid mapping")
        for key in ("command", "restore_command"):
            args = recipe.get(key)
            if not isinstance(args, list) or not args or not all(isinstance(a, str) and a for a in args):
                raise ValueError(f"description_change.{key}: expected explicit argv strings")
        if "timeout" in recipe:
            number(recipe["timeout"], "description_change.timeout", positive=True)
    for section, commands in [('lifecycle', ('probe_command','restart_command')),
                              ('mobility_fault', ('setup_command','restore_command'))]:
        if section not in data:
            continue
        config = data[section]
        allowed = {'reviewed','timeout',*commands}
        if section == 'mobility_fault':
            allowed |= {'command','watchdog'}
        if not isinstance(config,dict) or set(config)-allowed:
            raise ValueError(f'{section}: unknown fields or invalid mapping')
        if config.get('reviewed') is not True or data.get('debug') is not True:
            raise ValueError(f'{section}: requires explicit review and debug eligibility')
        for field in commands:
            argv = config.get(field)
            if not isinstance(argv,list) or not argv or any(not isinstance(v,str) or not v.strip() for v in argv):
                raise ValueError(f'{section}.{field}: explicit argv strings required')
        if 'timeout' in config:
            number(config['timeout'],f'{section}.timeout',positive=True)
        if section == 'mobility_fault':
            if not {'teleop','mobility'} <= set(welcome.capabilities):
                raise ValueError('mobility_fault requires teleop and mobility capabilities')
            command = config.get('command')
            if not isinstance(command,dict) or not command or set(command)-{'linear','lateral','angular'}:
                raise ValueError('mobility_fault.command: supported drive axes required')
            for axis,value in command.items():number(value,f'mobility_fault.command.{axis}')
            if not any(command.values()):raise ValueError('mobility fault must exercise motion')
            if number(config.get('watchdog'),'mobility_fault.watchdog',positive=True)>1:
                raise ValueError('mobility_fault.watchdog must be <= 1 second')
    return data


def validate_vector(vector, joints, label):
    if not isinstance(vector, dict) or set(vector) != set(joints):
        raise ValueError(f"{label}: use an exact joint-name -> value mapping; expected {list(joints)}")
    return [number(vector[joint], f"{label}.{joint}") for joint in joints]


def validate_pose(data, label):
    from ringfour import Pose
    if not isinstance(data, dict) or not isinstance(data.get("frame"), str) or not data["frame"]:
        raise ValueError(f"{label}: pose needs an explicit frame")
    for key in ("x", "y"):
        number(data.get(key), f"{label}.{key}")
    for key in ("z", "yaw"):
        if key in data:
            number(data[key], f"{label}.{key}")
    return Pose(**data)


def skip_coverage(request, category, reason):
    import pytest
    request.node.user_properties.append(("coverage", category))
    pytest.skip(f"{category.upper().replace('_', ' ')}: {reason}")


def read_sdk_feedback(robot, path):
    """Read a declared numeric signal without requiring watch registration."""
    parts=path.split('.')
    if len(parts)==3 and parts[0]=='sensors':
        value=getattr(robot.sensors,parts[1]).reading(parts[2])['value']
    elif len(parts)==4 and parts[0] in ('arms','groups'):
        group=robot.groups[parts[1]]
        if parts[2]=='joint_positions':
            value=group.get_joint_positions()[parts[3]]
        elif parts[2] in ('velocity','effort'):
            value=getattr(group.get_joints()[parts[3]],parts[2])
        else:
            raise ValueError('Unsupported group feedback quantity')
    elif parts[0] in ('velocity','pose','battery'):
        value=getattr(robot,parts[0])
        for field in parts[1:]:
            value=getattr(value,field)
    elif path in ('is_moving','estop_engaged','distance_traveled'):
        value=getattr(robot,path)
    else:
        raise ValueError('Use an SDK sensor, joint, pose, velocity or battery feedback path')
    if value is None:
        return None
    return number(value,'action feedback') if not isinstance(value,bool) else float(value)
