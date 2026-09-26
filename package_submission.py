from __future__ import annotations

from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


ROOT_FILES = (
    ".dockerignore",
    "Dockerfile",
    "MANIFEST.in",
    "README.md",
    "check_policy.py",
    "model.py",
    "pyproject.toml",
    "setup.py",
    "train.py",
)
SOURCE_DIRS = ("cpp", "python", "tests")
IGNORED_PARTS = {"__pycache__", "build", "dist"}
IGNORED_SUFFIXES = {".pyc", ".pyo", ".so"}


def included_files(root: Path):
    for name in ROOT_FILES:
        path = root / name
        if not path.is_file():
            raise RuntimeError(f"required submission file is missing: {name}")
        yield path
    for directory in SOURCE_DIRS:
        for path in sorted((root / directory).rglob("*")):
            relative = path.relative_to(root)
            if not path.is_file():
                continue
            if any(part in IGNORED_PARTS or part.endswith(".egg-info") for part in relative.parts):
                continue
            if path.suffix in IGNORED_SUFFIXES:
                continue
            yield path


def main() -> None:
    root = Path(__file__).resolve().parent
    output = root / "submission.zip"
    with ZipFile(output, "w", compression=ZIP_DEFLATED, compresslevel=9) as archive:
        for path in included_files(root):
            archive.write(path, path.relative_to(root).as_posix())
    print(f"created {output} ({output.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
