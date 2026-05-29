"""Probe an RBY1 robot for its model name and version via the rby1-sdk Python bindings."""

from __future__ import annotations

import re
from typing import Optional, Tuple


def detect_model_and_version(
    robot_ip: str, connect_timeout_sec: float
) -> Tuple[str, Optional[str]]:
    """Connect to ``robot_ip`` and return ``(model, version)``.

    ``model`` is normalized to ``'a'``, ``'m'``, or ``'ub'``. ``version`` is the
    first ``MAJOR.MINOR`` token found in ``RobotInfo.robot_model_version``, or
    ``None`` if no such token can be extracted (caller should fall back to its
    declared default).

    The probe uses ``rby1_sdk.create_robot_a`` because the gRPC backend is the
    same for every model, so an A-template handle is sufficient to read
    ``RobotInfo``. The handle is disconnected before returning.

    Raises ``RuntimeError`` if the SDK is missing, the connection fails, or the
    reported model name is not recognized.
    """
    try:
        import rby1_sdk as rby
    except ImportError as exc:
        raise RuntimeError(
            "rby1_sdk Python package is not installed; cannot auto-detect model. "
            "Install it with 'pip install rby1-sdk' or pass model:=a|m|ub explicitly."
        ) from exc

    timeout_ms = max(1, int(round(connect_timeout_sec * 1000.0)))

    try:
        robot = rby.create_robot_a(robot_ip)
    except Exception as exc:
        raise RuntimeError(
            f"failed to create RBY1 SDK handle for '{robot_ip}': {exc}"
        ) from exc

    try:
        try:
            connected = robot.connect(max_retries=1, timeout_ms=timeout_ms)
        except Exception as exc:
            raise RuntimeError(
                f"RBY1 model probe failed to connect to '{robot_ip}': {exc}"
            ) from exc
        if not connected:
            raise RuntimeError(
                f"RBY1 model probe could not connect to '{robot_ip}' "
                f"within {connect_timeout_sec:.2f}s."
            )

        try:
            info = robot.get_robot_info()
        except Exception as exc:
            raise RuntimeError(
                f"RBY1 model probe could not read robot info from '{robot_ip}': {exc}"
            ) from exc

        raw_name = (getattr(info, "robot_model_name", "") or "").strip().lower()
        raw_version = (getattr(info, "robot_model_version", "") or "").strip()

        if raw_name.startswith("a"):
            model = "a"
        elif raw_name.startswith("m"):
            model = "m"
        elif raw_name.startswith("ub"):
            model = "ub"
        else:
            raise RuntimeError(
                f"RBY1 model probe returned unsupported model name '{raw_name}' "
                f"from '{robot_ip}'."
            )

        match = re.search(r"(\d+\.\d+)", raw_version)
        version = match.group(1) if match else None

        print(
            f"[rby1_bringup] Detected RBY1 model='{model}' version='{version}' "
            f"(raw_name='{raw_name}', raw_version='{raw_version}')"
        )
        return model, version
    finally:
        try:
            robot.disconnect()
        except Exception:
            pass


def detect_model(robot_ip: str, connect_timeout_sec: float) -> str:
    """Backward-compatible wrapper returning only the model name."""
    model, _ = detect_model_and_version(robot_ip, connect_timeout_sec)
    return model
