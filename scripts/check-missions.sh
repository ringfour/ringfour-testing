#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
docker build --build-context missions=../missions -f docker/Dockerfile.missions -t ringfour-missions:checks .
mkdir -p test-results
docker run --rm --network none --mount "type=bind,src=$PWD/test-results,dst=/reports" ringfour-missions:checks python -m pytest tests -q --tb=short --junitxml=/reports/missions.xml "$@"
