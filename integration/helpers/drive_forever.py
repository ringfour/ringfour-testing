#!/usr/bin/env python3
"""Exit-test helper: teleop-drive forward until killed.

The Phase 3 exit test SIGKILLs this process mid-drive and verifies the
agent's watchdog halts the robot. Deliberately no cleanup handling: the
whole point is dying without saying goodbye.

Usage: python3 drive_forever.py <uri> <watchdog> <command-json>
"""

import json
import sys
import time

from ringfour import Robot


def main() -> None:
    uri = sys.argv[1]
    watchdog = float(sys.argv[2])
    command = json.loads(sys.argv[3])
    robot = Robot(uri)
    with robot.teleop(watchdog=watchdog) as t:
        while True:
            t.drive(**command)
            time.sleep(watchdog / 5)


if __name__ == "__main__":
    main()
