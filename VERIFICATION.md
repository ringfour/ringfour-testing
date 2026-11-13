# Ringfour migration verification — 13 November 2026

- Isolated Linux platform/harness suite: **708 passed, 2 skipped**. Skips are optional kinematics and optional mission-checkout catalog coverage; platform testing mounts no robot state or sockets and runs with networking disabled.
- Separate isolated mission suite: **9 passed** (six exploration-helper contracts, three learning-material checks).
- Native combined dashboard/harness and mission checks: **182 passed**, including source loading independent of test collection order.
- HTTP dashboard smoke: **478 selectable guides**, including nine in the independent Mission recipes suite; assets served and history empty.
- SDK and agent wheel/sdist builds succeeded; clean wheel installation and renamed sync/async imports succeeded without an old `robocore` installation.
- Agent image: `ringfour-agent:c-preview`, ARM64, image `sha256:06999af5a89168d313a6fdcc794b38cea8344221863d89c00c8b6b8aa02bae3c`. CLI help works; all **101 installed Python files** match the sibling source checkouts.
- Generated wire schema matches the original after three description-only brand replacements; method names, protocol version and wire structure are preserved.
- Dashboard JavaScript syntax check and all four Pixi TOML manifests passed validation.

JUnit reports are local ignored files under `test-results/`. Checks are repeatable using `scripts/check-platform.sh`, `scripts/check-missions.sh`, `scripts/verify-agent-image.py` and `scripts/check-migration.py`. The last script intentionally uses the original checkout recorded in `migration-source.json`.

No live robot motion was run. No SaaS directory/container, robot launch configuration or durable state was changed. Repositories were initialized locally on `main`; changes remain uncommitted, with no remotes or registry publication. The original checkout and C Preview artifacts are preserved. Milestone D remains incomplete.
