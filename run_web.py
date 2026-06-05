#!/usr/bin/env python3
"""Convenience runner for the interactive motion-planning web demo.

Usage:

    uv run python run_web.py                  # via uv
    python run_web.py                         # direct
    RELOAD=1 uv run python run_web.py         # enable auto-reload on file changes

The server starts at http://127.0.0.1:5000 by default.
Override the port with the PORT environment variable.
Enable auto-reload (and the Werkzeug debugger) with RELOAD=1.
"""

from __future__ import annotations

import os
import sys

# Allow running from the repo root without installing the package.
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from demo.web.app import app

if __name__ == "__main__":
    port = int(os.environ.get("PORT", "5000"))
    reload = os.environ.get("RELOAD", "0") == "1"
    mode = "debug + auto-reload" if reload else "production-like"
    print(f"🚀 Motion Planner Web Demo — http://127.0.0.1:{port} ({mode})")
    app.run(host="127.0.0.1", port=port, debug=reload, use_reloader=reload)
