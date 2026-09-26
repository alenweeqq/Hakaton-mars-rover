from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parent
    subprocess.run(
        [sys.executable, "-m", "pip", "install", "--editable", str(root), "--no-build-isolation"],
        check=True,
    )


if __name__ == "__main__":
    main()
