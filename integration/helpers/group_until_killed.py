"""Accept one reviewed group trajectory and stay alive until the parent kills us."""
import json
import sys
import time
from ringfour import JointTrajectory, Robot

if __name__ == '__main__':
    with Robot(sys.argv[1]) as robot:
        group = robot.groups[sys.argv[2]]
        trajectory = JointTrajectory.model_validate(json.loads(sys.argv[3]))
        with group.control() as control:
            operation = control.start_trajectory(trajectory)
            print(json.dumps({'operation_id':operation.id}),flush=True)
            while True:
                time.sleep(.1)
