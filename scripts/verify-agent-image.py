"""Compare every installed SDK/agent Python file with these sibling checkouts."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
root = Path(__file__).resolve().parents[2]
expected = {}
for package, source in (("ringfour", root / "ringfour-sdk/ringfour"),
                        ("ringfour_agent", root / "ringfour-agent/src/ringfour_agent/ringfour_agent")):
    for path in source.rglob("*.py"):
        expected[f"{package}/{path.relative_to(source)}"] = hashlib.sha256(path.read_bytes()).hexdigest()
probe = '''import hashlib, json, pathlib, ringfour, ringfour_agent
result = {}
for module in (ringfour, ringfour_agent):
    root = pathlib.Path(module.__file__).parent
    for path in root.rglob("*.py"):
        result[f"{module.__name__}/{path.relative_to(root)}"] = hashlib.sha256(path.read_bytes()).hexdigest()
print(json.dumps(result))
'''
actual = json.loads(subprocess.check_output(["docker", "run", "--rm", "--network", "none",
                    sys.argv[1] if len(sys.argv) > 1 else "ringfour-agent:c-preview", "python3", "-c", probe], text=True))
if actual != expected:
    raise SystemExit(f"Image source mismatch: {sorted(k for k in expected.keys() | actual.keys() if expected.get(k) != actual.get(k))}")
print(f"Image matches all {len(expected)} installed Python source files.")
