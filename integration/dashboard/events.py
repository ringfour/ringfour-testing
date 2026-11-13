"""Pytest plugin recording completed tests without waiting for JUnit teardown."""
import json
import os
from pathlib import Path


def emit(event):
    path = os.environ.get("RINGFOUR_TEST_EVENTS")
    if path:
        with Path(path).open("a") as stream:
            stream.write(json.dumps(event) + "\n")


def pytest_collection_finish(session):
    names = {"group_name", "camera_name", "range_name", "wrench_name", "sensor_name", "action_name", "power_name", "capability_name"}
    emit({"event": "collection", "tests": [item.nodeid for item in session.items],
          "parameters": {item.nodeid: {k:v for k,v in getattr(getattr(item, "callspec", None), "params", {}).items()
                                        if k in names and isinstance(v, (str, int, float, bool, type(None)))}
                         for item in session.items}})


def pytest_runtest_logstart(nodeid, location):
    emit({"event": "start", "nodeid": nodeid})


def pytest_runtest_logreport(report):
    emit({"event": "report", "nodeid": report.nodeid, "when": report.when,
          "outcome": report.outcome, "duration": report.duration,
          "detail": str(report.longrepr) if report.longrepr else "",
          "properties": dict(report.user_properties)})
