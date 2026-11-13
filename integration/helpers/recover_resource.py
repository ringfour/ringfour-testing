"""Explicit integration helper: verify recovery through the supplied local URI."""
import json
import sys
from ringfour import Robot

if __name__ == '__main__':
    with Robot(sys.argv[1],timeout=15) as robot:
        report = robot._runner.run(robot._robot.connection.call('safety.recover',{'resource':sys.argv[2]},timeout=15))
        assert report['recovered'] and report['inhibit_engaged']
        assert robot.get_estop_engaged()
        print(json.dumps(report))
