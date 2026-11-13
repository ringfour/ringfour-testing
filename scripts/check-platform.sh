#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
docker build --build-context sdk=../ringfour-sdk --build-context agent=../ringfour-agent -f docker/Dockerfile.platform -t ringfour-testing:preview .
mkdir -p test-results
# Mount reports only; never mount a robot socket, ROS graph or state volume.
docker run --rm --network none --mount "type=bind,src=$PWD/test-results,dst=/reports" ringfour-testing:preview python -m pytest -q --tb=short --basetemp=/tmp/rf-tests --junitxml=/reports/platform.xml "$@"
