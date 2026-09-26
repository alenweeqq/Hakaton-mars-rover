from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def install(path: Path) -> None:
    subprocess.run(
        [sys.executable, "-m", "pip", "install", "--editable", str(path), "--no-build-isolation"],
        check=True,
    )


def main() -> None:
    root = Path(__file__).resolve().parent
    install(root)


if __name__ == "__main__":
    main()
