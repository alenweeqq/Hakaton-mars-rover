from __future__ import annotations

import argparse
from pathlib import Path


def validate_policy(path: Path) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"policy artifact is missing or empty: {path}")
    import onnxruntime as ort

    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    if len(session.get_inputs()) != 7:
        raise RuntimeError(f"expected 7 ONNX inputs, got {len(session.get_inputs())}")
    if len(session.get_outputs()) < 3:
        raise RuntimeError(f"expected at least 3 ONNX outputs, got {len(session.get_outputs())}")
    print(f"policy artifact is valid: {path} ({path.stat().st_size} bytes)")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=Path, nargs="?", default=Path("/output/policy.onnx"))
    args = parser.parse_args()
    validate_policy(args.path)


if __name__ == "__main__":
    main()
