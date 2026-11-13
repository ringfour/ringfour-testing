# Ringfour testing

Platform conformance, robot integration checks and the local testing dashboard. This repository owns the former SDK contract tests; they remain selectable in the dashboard. No historical runs have been imported.

## Local workspace

Use sibling checkouts `ringfour-sdk`, `ringfour-agent` and `ringfour-testing`. These are independent repositories; no SaaS checkout is required.

```sh
uv venv --python 3.12
uv pip install --python .venv/bin/python -e ../ringfour-sdk -e ../ringfour-agent -e .
# Complete Linux platform suite, isolated from ROS and robot stacks:
./scripts/check-platform.sh
# Dashboard/harness checks also run natively:
.venv/bin/python -m pytest integration/dashboard/tests -q
# Local dashboard; existing consoles are not restarted:
.venv/bin/python integration/dashboard/server.py --port 8767
```

The platform script builds `ringfour-testing:preview` and runs it with networking disabled. Reports go to `test-results/platform.xml`; it mounts no robot sockets or state. The complete platform suite includes Linux peer-credential and `/dev/shm` tests, so run it in Docker on macOS. The SDK itself runs natively using NET.

Robot integration is separate and opt-in. See [integration instructions](integration/README.md), [profile example](integration/conformance.example.yaml) and [preparation guide](integration/ROBOT_PREPARATION.md). Environment variables now use `RINGFOUR_`, including `RINGFOUR_CONFORMANCE`. Review fixture commands before motion; old robot stack launch files are not changed by this migration.

[Hardening plan](HARDENING.md) · [Migration and roadmap](MIGRATION.md)

Learning material now lives in [missions](../missions/README.md). If that optional checkout exists, the dashboard also exposes **Mission recipes · offline**, separate from platform acceptance. Run `./scripts/check-platform.sh` and then `./scripts/check-missions.sh` for isolated checks. Source hashes and the per-file material audit live in `../missions/material-moves.json` and `../missions/MATERIAL_REVIEW.md`.
