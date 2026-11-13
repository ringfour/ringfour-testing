"""Generate reproducible joint-space probes from measured state, never morphology."""
import math


def generate_probe(joints, positions, specs, policy, tolerance=.01, limit_allowances=None):
    if policy.get('mode') != 'small' or policy.get('reviewed') is not True:
        raise ValueError('Small movement requires reviewed group eligibility')
    budgets = {'revolute': policy.get('angular_step', .05),
               'continuous': policy.get('angular_step', .05),
               'prismatic': policy.get('linear_step', .005)}
    baseline = {}
    measured_baseline = {}
    adjustments = {}
    for name in joints:
        value = positions.get(name)
        if value is None or not math.isfinite(value):
            raise ValueError(f'{name}: missing finite measured baseline')
        spec = specs.get(name)
        if spec is None or spec.type not in budgets:
            raise ValueError(f'{name}: unsupported or missing joint type')
        measured_baseline[name] = value
        if spec.limits and not spec.limits[0] <= value <= spec.limits[1]:
            lo, hi = spec.limits
            allowance = (limit_allowances or {}).get(name, 1e-5)
            if value < lo-allowance or value > hi+allowance:
                raise ValueError(f'{name}: measured baseline outside command limits')
            value = max(lo, min(hi, value))
            adjustments[name] = {'measured':measured_baseline[name], 'command':value, 'allowance':allowance}
        baseline[name] = value
    targets = []
    for name in joints:
        spec = specs[name]
        step = budgets[spec.type]
        if not math.isfinite(step) or step <= 0:
            raise ValueError('Movement budgets must be finite and positive')
        delta = step
        if spec.limits:
            lo, hi = spec.limits
            positive, negative = hi-baseline[name], baseline[name]-lo
            delta = min(step, positive) if positive >= negative else -min(step, negative)
        if abs(delta) <= 2*tolerance:
            raise ValueError(f'{name}: movement budget/available range cannot exceed measurement tolerance; adjust the budget or tolerance')
        target = dict(baseline)
        target[name] += delta
        targets.extend([target, dict(baseline)])
    if not targets:
        raise ValueError('Group has no joints')
    return {'baseline': baseline, 'measured_baseline': measured_baseline,
            'limit_adjustments': adjustments, 'waypoints': targets,
            'policy': dict(policy), 'tested_joints': list(joints),
            'collision_checked': False}
