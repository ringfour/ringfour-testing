# SDK compatibility suite audit

Purpose: verify that a deployed robot can be operated through ringfour-sdk and that advertised operations have observable, correct results.

Inspected 87 live test functions. Parameterization expands these per capability, camera, and group; instance counts are not independent operation counts. In run `20261008-184331-5f5ab804`, 375 passes comprise 300 offline SDK cases and 75 live robot cases. Its two failures are the already diagnosed base/resolved-profile mismatch; the golden-profile targeted rerun passed those two checks and TF.

## Recommended default acceptance run

- Connection, effective profile/handshake agreement, model identity and actual bindings.
- Every advertised read interface: usable sensor/camera/map/joint feedback, appropriate timestamps and frames.
- Every advertised command interface: reviewed targets, observed motion, arrival, progress/errors, explicit stop/cancel and restoration.
- Distinct safety paths: watchdog silence, disconnected client, command ownership and session enforcement; watches when advertised and reviewed.
- Cartesian/collision APIs only where supported; validate against the actual model/TF and independently reviewed geometry.

Keep SDK offline regressions separate from robot compatibility totals. Keep movement qualification, numerical diagnostics, benchmarks, model-change and debug fault injection available as optional suites. Do not delete regression coverage to shorten robot acceptance.

## Reporting and coverage changes

- Report operations supported/tested/failed/unverified; show per-instance detail below them.
- Do not count capability names or callable metadata as proof of execution.
- Unsupported features should appear once per instance, rather than expanding into many N/A checks.
- Missing fixtures for supported motion remain visible as unverified. Optional qualification fixtures should not reduce basic operation coverage.
- Observed motion/feedback proves command execution relative to reported sensors, not independent physical accuracy. Figure eight closure does not independently validate odometry; use an external reference for that claim.
- Profile mappings declare commands; environmental targets, budgets and physical assumptions stay in test settings.

## Decisions for each live test

| Test | Decision | Reason |
|---|---|---|
| `test_capability_sensors.py::test_imu_reading` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_capability_sensors.py::test_every_range_sensor` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_capability_sensors.py::test_every_wrist_force_torque` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_capability_sensors.py::test_vendor_sensor_readings` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_capability_sensors.py::test_status_interfaces` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_capability_sensors.py::test_environment_sources` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_capability_sensors.py::test_power_lane_metadata` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_capability_sensors.py::test_vendor_action_changes_feedback` | Core | Reviewed arguments must produce the configured measured feedback change; optional restoration is checked against the baseline. Missing fixtures remain untested. |
| `test_capability_sensors.py::test_optional_magnetometer` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_movement_profiles.py::test_figure_eight_closes_both_loops` | Optional / regression | Optional mobility qualification; path closure is not a universal SDK contract. |
| `test_movement_profiles.py::test_braking_time_and_distance` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_movement_profiles.py::test_odometry_spin_rotation_and_drift` | Optional / regression | Optional mobility qualification; drift budget and spin ability depend on platform. |
| `test_phase1_exit.py::test_connect_to_configured_agent` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase1_exit.py::test_connect_tcp` | Merge | Merge with connection check when both resolve to the same endpoint; retain distinct configured transports. |
| `test_phase1_exit.py::test_protocol_mismatch_rejected` | Optional / regression | Protocol regression; run in agent integration CI, outside default robot acceptance. |
| `test_phase1_exit.py::test_handshake_is_internally_consistent` | Merge | Merge into connection/profile contract report. |
| `test_phase1_exit.py::test_advertised_capability_has_a_known_contract` | Rework | Known-name membership is a coverage inventory, not proof the advertised capability works. Report coverage, not a pass. |
| `test_phase1_exit.py::test_binary_payload_channel` | Optional / regression | Requires debug random-payload injection; actual camera payload retrieval covers deployed transport. |
| `test_phase2_exit.py::test_capabilities_reflect_profile_yaml` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase2_exit.py::test_pose_and_velocity_are_finite` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase2_exit.py::test_rejected_call_lands_in_audit_log_with_reason` | Optional / regression | Keep as optional fault-injection safety qualification; requires debug estop mutation. |
| `test_phase2_exit.py::test_multi_client_reads` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase2_exit.py::test_task_lifecycle_over_wire` | Optional / regression | Synthetic debug task; real navigation/trajectory tasks should prove deployed task lifecycle. |
| `test_phase3_exit.py::test_teleop_drives_robot_and_pose_changes` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase3_exit.py::test_watchdog_halts_on_command_silence` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase3_exit.py::test_killed_client_halts_robot_within_watchdog` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase3_exit.py::test_tf_wrapper_against_live_tree` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase4_exit.py::test_camera_frame_matches_its_intrinsics` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase4_exit.py::test_camera_stream_yields_fresh_frames` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase4_exit.py::test_synced_depth_deproject_and_cloud` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase4_exit.py::test_lidar_scan_and_stream` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase4_exit.py::test_group_joint_states_with_urdf_limits` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase4_exit.py::test_stop_watch_halts_motion_without_client_polling` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase5_exit.py::test_map_grid_is_live` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase5_exit.py::test_navigate_to_configured_goal` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase5_exit.py::test_navigation_cancellation_halts` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase5_exit.py::test_navigation_failure_carries_reason` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase5_exit.py::test_map_save_localize_and_go_journey` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase5_exit.py::test_existing_places_are_readable` | Rework | An empty list is a valid read, but does not verify save/resolve/delete. Report operation coverage separately. |
| `test_phase6_description_exit.py::test_description_arrives_in_the_handshake` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_description_exit.py::test_description_slices_into_each_group_subchain` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_description_exit.py::test_camera_frame_exists_in_live_description` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_description_exit.py::test_limits_match_joint_state_call` | Merge | Merge into one group feedback/model/limits check. |
| `test_phase6_description_exit.py::test_description_is_stable_across_connections` | Merge | Merge into multi-client/model identity check. |
| `test_phase6_description_exit.py::test_refresh_matches_handshake` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_description_exit.py::test_republishing_triggers_model_changed` | Optional / regression | Optional model-change qualification requiring explicit change/restore commands. |
| `test_phase6_exit.py::test_audit_outcomes_arrive_as_enum_members` | Merge | Assert typed outcomes with real control audit checks. |
| `test_phase6_exit.py::test_slam_mode_is_typed` | Merge | Assert mode/backend alongside live mapping check. |
| `test_phase6_exit.py::test_payload_transport_is_typed` | Merge | Assert alongside actual camera retrieval; type alone proves no accessibility. |
| `test_phase6_exit.py::test_profile_locomotion_is_reported_independently_of_robot_name` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_exit.py::test_nav_status_phase_is_typed` | Merge | Inspect progress/status in successful navigation check; avoid repeating navigation. |
| `test_phase6_groups_exit.py::test_capability_report_lists_groups` | Merge | Merge into profile/group inventory check. |
| `test_phase6_groups_exit.py::test_arms_sugar_resolves_to_the_same_handles` | Optional / regression | Client convenience/ambiguity behavior; SDK regression with diverse synthetic profiles. |
| `test_phase6_groups_exit.py::test_group_identity_and_controller_roles` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_groups_exit.py::test_joint_names_match_live_description` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_groups_exit.py::test_joint_watch_paths_follow_resolved_joints` | Rework | Unreachable threshold plus nullable value only proves watch registration. Validate measured updates and separately test a reviewed trigger. |
| `test_phase6_groups_exit.py::test_deprecated_alias_warns_and_returns_groups` | Optional / regression | Legacy client alias; SDK regression. |
| `test_phase6_groups_exit.py::test_tool_metadata_matches_declared_group` | Merge | Merge into group metadata validation, retaining read-only coverage without motion. |
| `test_phase6_kinematics_exit.py::test_fk_agrees_with_tf_tree` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_kinematics_exit.py::test_fk_intermediate_links_agree` | Optional / regression | Detailed geometry diagnostic; retain tip FK/TF check for supported Cartesian operation. |
| `test_phase6_kinematics_exit.py::test_ik_round_trips_real_chain` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_kinematics_exit.py::test_ik_is_deterministic` | Optional / regression | Solver regression; exact solutions rounded to ten decimals are not an operation criterion. |
| `test_phase6_kinematics_exit.py::test_profile_dof_mask_is_consistent` | Rework | Requiring six constraints by default can reject valid underactuated Cartesian groups. Respect supported solver constraints; do not infer from joint count alone. |
| `test_phase6_kinematics_exit.py::test_unreachable_names_group_and_tip` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_kinematics_exit.py::test_limits_come_from_live_description` | Merge | Merge into same group feedback/model/limits check, including local kinematics bounds. |
| `test_phase6_kinematics_exit.py::test_reach_is_finite` | Merge | Merge into supported Cartesian kinematics smoke check. |
| `test_phase6_kinematics_exit.py::test_manipulability_is_finite` | Merge | Merge into supported Cartesian kinematics smoke check. |
| `test_phase6_kinematics_exit.py::test_group_without_tip_refuses_ik` | Merge | Report Cartesian unsupported once per group; regression covers refusal semantics. |
| `test_phase6_kinematics_exit.py::test_collision_model_covers_reported_geometry` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_kinematics_exit.py::test_snapshot_matches_live_robot` | Optional / regression | Optional geometry onboarding; absence of a snapshot does not prevent basic SDK operation. |
| `test_phase6_kinematics_exit.py::test_reach_map_is_internally_consistent` | Optional / regression | Optional precomputed planning feature, not a universal prerequisite. |
| `test_phase6_kinematics_exit.py::test_rigid_bodies_are_excluded_from_collision_pairs` | Optional / regression | Collision algorithm invariant; test in SDK CI. |
| `test_phase6_kinematics_exit.py::test_known_clear_configurations` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_kinematics_exit.py::test_known_collision_configurations` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_kinematics_exit.py::test_ik_refuses_colliding_configuration` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_kinematics_exit.py::test_collision_checking_meets_configured_budget` | Optional / regression | Optional benchmark; morphology and host specific budget. |
| `test_phase6_motion_exit.py::test_follow_waypoints_and_converge` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_motion_exit.py::test_progress_climbs_monotonically` | Merge | Assert monotonic progress in existing convergence trajectory; avoid repeating motion. |
| `test_phase6_motion_exit.py::test_break_cancels_trajectory` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_motion_exit.py::test_second_client_is_refused_while_lock_is_held` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_motion_exit.py::test_nothing_moves_outside_session` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_motion_exit.py::test_session_restores_controllers_on_exit` | Rework | Repeated transition lists are indirect evidence. Compare actual controller states before and after when available. |
| `test_phase6_motion_exit.py::test_switch_is_audited` | Merge | Merge with session switching/restoration check; correlate actual session. |
| `test_phase6_motion_exit.py::test_hold_commands_current_position` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_motion_exit.py::test_tool_opens_and_closes` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |
| `test_phase6_motion_exit.py::test_is_holding_distinguishes_known_obstruction` | Optional / regression | Optional tool qualification with a reviewed obstruction/stall fixture; not proof of a physical grasp. |
| `test_phase6_motion_exit.py::test_missing_controller_role_says_what_is_available` | Keep | Direct deployed interface, behavior, feedback, binding or distinct safety coverage; run only when applicable. |

Function-level decisions: 49 keep, 6 rework, 16 optional / regression, 16 merge. These are recommendations, not a predicted final instance count; merging retains the useful assertions.

Implemented: default compatibility contains 54 live definitions; qualification 5,
diagnostics 7, and live regression 22 remain selectable. One new places CRUD
check was added, giving 88 live definitions in total. Sixteen duplicate checks
remain selectable in regression while their useful assertions were folded into
acceptance checks. The capability-name inventory also moved out of acceptance;
the profile check reports advertised capabilities with no shared contract.
Offline counts are separated, and repeated unsupported reasons are grouped only
for display. All original artifacts and checks remain available.


## Implementation verification

- SDK regressions: 300 passed; dashboard/harness self-tests: 90 passed.
- G1 dashboard run `20261008-191116-dbfb2b42` with commands disabled:
  36 passed, zero failures/errors, 35 untested, 29 raw N/A instances.
  Those unsupported instances display as 14 feature groups; raw cases are retained.
- Browser verification: default 54 checks, optional suite selection, grouped
  unsupported details, separated counters, and responsive layouts at 1500,
  1000, 800 and 390 pixels; no JavaScript errors.
- No ROS restart or physical motion was performed. Places write coverage used
  a unique temporary label and verified cleanup.
