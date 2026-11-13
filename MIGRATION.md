# Ringfour migration

## Order and scope

1. Preserve the A–C SDK/agent contract while splitting and renaming the repositories.
2. Build installable previews and verify the renamed SDK and agent together.
3. Complete D: controlled platform conformance plus reference, examples and course validation.

Robot integration qualification is a separate suite and does not gate D or platform releases. Physical claims remain deployment-specific. D is not marked complete by this migration.

## Repository ownership

| Repository | Owns |
|---|---|
| `ringfour-sdk` | Pure Python `ringfour` client, shared protocol models, schema generators, reference, examples and course |
| `ringfour-agent` | ROS 2 `ringfour_agent` bridge, ROS package and release Docker build |
| `ringfour-testing` | `platform_tests`, robot `integration` checks, fixtures, dashboard, hardening plan and acceptance tooling |

The agent depends on the SDK; the SDK has no agent or ROS dependency. Testing depends on both. The local dashboard and source-reference checks currently expect sibling checkout names. CI checks out those three repositories explicitly into a shared workspace. Registry URLs/remotes are not invented or published by this migration.

## Renames

| Before | After |
|---|---|
| Distribution `robocore` | `ringfour-sdk` |
| Python import `robocore` | `ringfour` |
| `RobocoreError` | `RingfourError` |
| Agent import/ROS package `robocore_agent` | `ringfour_agent` |
| Agent image `robocore-agent` | `ringfour-agent` |
| `ROBOCORE_*` | `RINGFOUR_*` |
| `/opt/robocore`, `/etc/robocore`, `/var/lib/robocore` | `/opt/ringfour`, `/etc/ringfour`, `/var/lib/ringfour` |
| Default agent `/tmp/robocore.sock` | `/tmp/ringfour.sock` |

JSON-RPC method names, protocol version, wire-schema structure and domain behavior are preserved. Three generated schema descriptions change only the brand name. Python names, environment variables, default storage/socket paths and SHM naming change with the brand. Use matching renamed clients/agent for SHM. Existing remote clients using NET require explicit endpoints; no old-import compatibility aliases are installed.

Do not silently start with a fresh state directory when replacing an existing deployment: stop the old agent cleanly, back up its durable state and explicitly migrate or mount that state into the new location. Preserve journal and quarantine evidence; normal recovery rules still apply. No robot Compose files, profiles, deployments or old state stores were changed in this migration.

## Provenance and limits

The source was copied from the dirty `review_and_hardening` working tree, not merely HEAD. `migration-source.json` records the base commit and SHA256 of every copied source file before renaming. Existing dashboard UI changes were included. Ignored build products, local configurations and historical run outputs were excluded. The original checkout and C Preview artifacts remain available and unchanged apart from updating the original hardening plan's D scope.

The SaaS `platform` directory and its containers are out of scope and untouched. No remote repositories, registry pushes or deployments are performed here. Git histories remain in the original checkout; the new repositories start locally, and the provenance manifest identifies their source.

## Verified migration baseline

Before reorganizing learning material: Linux platform and harness checks **714 passed, 1 optional kinematics skip**; native harness **172 passed**. Clean SDK/agent wheel installation passed. The image `ringfour-agent:c-preview` matched all 101 installed Python source files. HTTP smoke found 475 selectable dashboard guides, served assets successfully and confirmed empty dashboard history. Wire-schema changes were limited to three brand descriptions. No robot deployment or SaaS stack was changed.

Teaching material now lives in sibling `missions`; six exploration recipe tests moved with their helper into the optional Missions suite. Platform acceptance no longer imports that helper. See `../missions/MATERIAL_REVIEW.md`.
