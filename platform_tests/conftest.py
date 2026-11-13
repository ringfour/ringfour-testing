"""Offline tests for the SDK. No robot, no ROS, no Gazebo.

Everything here runs from a plain checkout in under a minute. Anything
that needs a robot on the other end lives in `e2e_testing/`, which
has its own conftest and its own fixtures — that split is the point of
this file existing at all.

The other suite is deliberately NOT called `tests` — `e2e_testing/` says
what it is, and the two can never be confused at a glance. This one is
what the SDK can prove about itself; that one is what only a real robot,
running, can answer.

WHY THE AGENT IS ON THE PATH. Nine of the thirteen modules here import
`ringfour_agent`, which looks like a layering violation and is not: they
test the WIRE CONTRACT, and a contract has two sides. The SDK's encoder
and the agent's decoder have to agree about the same bytes, and the only
way to assert that is to have both in the room. They still need no robot,
which is what keeps them here rather than in `e2e_testing/`.

If a test needs a running agent — not merely an importable one — it belongs
in `e2e_testing/`.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_DIR = REPO_ROOT / "ringfour-sdk"
AGENT_SRC_DIR = REPO_ROOT / "ringfour-agent" / "src" / "ringfour_agent"

# Prepended rather than appended: a stale `ringfour` installed in the venv
# must not win over the checkout under test.
for extra in (str(SDK_DIR), str(AGENT_SRC_DIR)):
    if extra not in sys.path:
        sys.path.insert(0, extra)


# -- what the offline modules import from here ---------------------------------

# Profiles live with the robots (see robots/profiles/). The stub is the
# deliberately minimal one used to assert schema behaviour without a robot.
STUB_PROFILE = Path(__file__).parent / "fixtures" / "stub.yaml"

# Synthetic profile regression data belongs to the SDK, not a robot repo.
# Editing or omitting any real robot workspace must not change these tests.
PROFILES_DIR = Path(__file__).parent / "fixtures" / "profiles"


def _free_tcp_port() -> int:
    """An unused localhost port. Racy by nature — the port is free when we
    ask and could be taken before the caller binds it — but the window is
    small and the alternative is a fixed port that collides with a real
    agent."""
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def command_id(ctx):
    """Raw-wire fixtures use the production client-generated ID format."""
    import uuid
    return f"{ctx.boot_epoch}.{int((ctx.commands.now()+299)*1000)}.{uuid.uuid4().hex}"
