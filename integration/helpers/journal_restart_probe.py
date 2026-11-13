"""Local authenticated restart probe. Synthetic task only; never commands motion.

Usage: journal_restart_probe.py <unix-uri> prepare|verify
verify reads prepare's JSON from stdin. Robot lifecycle is owned by the suite.
"""
import json
import sys
import time
from ringfour import Robot, AgentEpochChanged


def main():
    with Robot(sys.argv[1], timeout=20) as robot:
        feature = robot._robot._welcome.features.get('commands')
        assert feature and feature['durable'], 'agent must advertise durable command/result storage'
        access = robot._robot._welcome.features['access']
        assert access['authenticated'], 'cross-connection queries require authenticated identity'
        if sys.argv[2] == 'prepare':
            key = robot.commands.new()
            params = {'duration': .05, 'ticks': 1}
            reply = robot.commands.retry(key, 'debug.run_task', params)
            deadline = time.monotonic()+10
            while True:
                outcome = robot.operations.get(reply['task_id'])
                if outcome.state == 'terminal':
                    break
                assert time.monotonic() < deadline, 'synthetic task did not finish'
                time.sleep(.025)
            assert outcome.snapshot.state == 'succeeded'
            print(json.dumps({'command_id': key, 'operation_id': reply['task_id'],
                'epoch': feature['epoch'], 'principal': access['principal'],
                'params': params, 'terminal': outcome.model_dump(mode='json')}))
        elif sys.argv[2] == 'verify':
            prior = json.load(sys.stdin)
            assert feature['epoch'] != prior['epoch'], 'fixture did not restart the agent'
            assert access['principal'] == prior['principal']
            accepted = robot.commands.get(prior['command_id'])
            terminal = robot.operations.get(prior['operation_id'])
            assert accepted.operation_id == prior['operation_id']
            current = terminal.model_dump(mode='json')
            previous = prior['terminal']
            assert current['snapshot'].pop('report_age',0) >= previous['snapshot'].pop('report_age',0)
            assert current == previous, 'restart rewrote terminal evidence'
            try:
                robot.commands.retry(prior['command_id'], 'debug.run_task', prior['params'])
            except AgentEpochChanged:
                pass
            else:
                raise AssertionError('old epoch replay was accepted')
            print(json.dumps({'terminal_retained': True, 'old_epoch_rejected': True,
                              'epoch': feature['epoch'], 'operation_id': terminal.operation_id}))
        else:
            raise ValueError('expected prepare or verify')


if __name__ == '__main__':
    main()
