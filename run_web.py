#!/usr/bin/env python3
"""Convenience runner for the interactive motion-planning web demo.

Usage:

    uv run python run_web.py          # via uv
    python run_web.py                 # direct

The server starts at http://127.0.0.1:5000 by default.
Override the port with the PORT environment variable.
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
    print(f"🚀 Motion Planner Web Demo — http://127.0.0.1:{port}")
    app.run(host="127.0.0.1", port=port, debug=False)
