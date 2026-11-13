"""The console must serve its bundled design assets without external services."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server


@pytest.mark.parametrize('path,mime', [
    ('/dashboard.css', 'text/css; charset=utf-8'),
    *[(f'/design-system/{name}.css', 'text/css; charset=utf-8')
      for name in ('tokens', 'base', 'components')],
    *[(f'/design-system/assets/fonts/SpaceGrotesk-{weight}.woff2', 'font/woff2')
      for weight in ('Regular', 'Medium', 'Bold')],
])
def test_design_assets(path, mime):
    handler = object.__new__(server.Handler)
    handler.path = path
    replies = []
    handler.reply = lambda data, status=200, mime='application/json': replies.append((data, status, mime))
    handler.do_GET()
    assert replies == [((server.HERE / path.lstrip('/')).read_bytes(), 200, mime)]


def test_asset_directory_traversal_is_rejected():
    handler = object.__new__(server.Handler)
    handler.path = '/design-system/../../server.py'
    replies = []
    handler.reply = lambda data, status=200, mime='application/json': replies.append(status)
    handler.do_GET()
    assert replies == [404]
