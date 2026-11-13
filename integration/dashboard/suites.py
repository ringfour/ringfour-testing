"""Robot acceptance defaults and independently selectable extended checks."""
SUITES = [
    {"id": "missions", "label": "Mission recipes · offline", "description": "Test optional mission helpers with synthetic inputs; no robot required."},
    {"id": 'compatibility', "label": 'Robot compatibility', "description": 'Verify profile bindings, live feedback, measured commands, stopping and control ownership.'},
    {"id": 'qualification', "label": 'Robot qualification', "description": 'Measure movement precision, braking, tool stalls and collision-query performance against configured limits.'},
    {"id": 'diagnostics', "label": 'Diagnostics & fault injection', "description": 'Inspect recorded geometry, intermediate transforms, model changes and debug fault injection.'},
    {"id": 'regression', "label": 'Live software regressions', "description": 'Check client aliases, protocol behavior, types and detailed software contracts on a live agent.'},
    {"id": 'sdk', "label": 'Platform · offline', "description": 'Run software regressions with synthetic fixtures. Does not connect to or command the robot.'},
    {"id": 'all', "label": 'All suites', "description": 'Run robot compatibility and every optional suite, including offline SDK regressions.'},
]
REGRESSION = {
    'test_advertised_capability_has_a_known_contract',
    'test_arms_sugar_resolves_to_the_same_handles',
    'test_audit_outcomes_arrive_as_enum_members',
    'test_capability_report_lists_groups',
    'test_connect_tcp',
    'test_description_is_stable_across_connections',
    'test_group_without_tip_refuses_ik',
    'test_handshake_is_internally_consistent',
    'test_ik_is_deterministic',
    'test_limits_come_from_live_description',
    'test_limits_match_joint_state_call',
    'test_manipulability_is_finite',
    'test_nav_status_phase_is_typed',
    'test_payload_transport_is_typed',
    'test_progress_climbs_monotonically',
    'test_protocol_mismatch_rejected',
    'test_reach_is_finite',
    'test_rigid_bodies_are_excluded_from_collision_pairs',
    'test_slam_mode_is_typed',
    'test_switch_is_audited',
    'test_tool_metadata_matches_declared_group',
}
DIAGNOSTICS = {
    'test_lost_motion_reply_reconciles_without_duplicate_dispatch',
    'test_agent_restart_retains_results_and_rejects_old_epoch',
    'test_killed_mobility_client_verifies_stop_and_releases',
    'test_operation_wait_and_observer_exit_do_not_cancel',
    'test_joint_guard_rejects_unsafe_entry_and_latches',
    'test_healthy_lease_survives_slow_observation',
    'test_group_lease_expiry_stops_a_live_nonrenewing_connection',
    'test_killed_group_client_stops_and_releases',
    'test_feedback_loss_requires_quarantine_and_local_recovery',
    'test_unverified_network_identity_cannot_recover',

    'test_inhibit_rejects_all_group_motion',
    'test_binary_payload_channel',
    'test_fk_intermediate_links_agree',
    'test_reach_map_is_internally_consistent',
    'test_rejected_call_lands_in_audit_log_with_reason',
    'test_republishing_triggers_model_changed',
    'test_snapshot_matches_live_robot',
    'test_task_lifecycle_over_wire',
}
QUALIFICATION = {
    'test_braking_time_and_distance',
    'test_collision_checking_meets_configured_budget',
    'test_figure_eight_closes_both_loops',
    'test_is_holding_distinguishes_known_obstruction',
    'test_odometry_spin_rotation_and_drift',
}

def suite_for(function, scope):
    if scope == "missions":
        return "missions"
    if scope == "sdk":
        return "sdk"
    for name, functions in (("qualification", QUALIFICATION), ("diagnostics", DIAGNOSTICS), ("regression", REGRESSION)):
        if function in functions:
            return name
    return "compatibility"
