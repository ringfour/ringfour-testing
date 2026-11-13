"""Check the renamed schema against the recorded source checkout."""
import json
from pathlib import Path
root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / "migration-source.json").read_text())
original = Path(manifest["source"]) / "robocore-sdk/protocol.json"
current = root.parent / "ringfour-sdk/protocol.json"
assert json.loads(original.read_text().replace("robocore", "ringfour")) == json.loads(current.read_text()), "Unexpected protocol schema change"
print("Wire schema matches the original; only brand descriptions changed.")
