#!/usr/bin/env python3
"""Configured safety-watch probe: keep issuing commands without polling a watch.

Usage: drive_ignoring_watch.py <uri> <teleop-recipe-json>
The recipe defines command axes, watchdog, duration, and the watch condition.
No robot names, sensor topics, or assumed forward velocity live here.
"""
import json
import signal
import sys
import time
from ringfour import Robot
from ringfour.errors import RingfourError


def main():
    def terminate(signum, frame):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, terminate)
    recipe = json.loads(sys.argv[2])
    with Robot(sys.argv[1]) as robot:
        settings=recipe['watch_stop']
        reference=robot.signals[settings['path']]
        comparison='above' if 'above' in settings else 'below'
        condition=getattr(reference,comparison)(settings[comparison],max_age=settings.get('max_age',.5))
        # Installation must precede motion; an unsafe/stale initial signal
        # rejects before the body rather than briefly permitting commands.
        with robot.guard(condition) as guard:
            print('GUARD ACTIVE '+guard.id,flush=True)
            with robot.teleop(watchdog=recipe['watchdog']) as session:
                while True:
                    try:session.drive(**recipe['command'])
                    except RingfourError:pass # deliberately keep trying after trip
                    time.sleep(recipe['watchdog']/5)



if __name__ == "__main__":
    main()
