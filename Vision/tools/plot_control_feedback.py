"""Plot STM32 control feedback collected through MaixCAM.

The default Windows stream receiver writes ``control.csv``.  The MaixCAM
fallback recorder writes the same control fields, so this script accepts both
formats without modifying the real-time vision or motor-control processes.
"""

import argparse
import csv
from pathlib import Path


def _read_numeric_rows(path):
    rows = []
    with Path(path).open(encoding="utf-8", newline="") as source:
        for row in csv.DictReader(source):
            converted = {}
            for name, value in row.items():
                if value in (None, ""):
                    continue
                try:
                    converted[name] = float(value)
                except ValueError:
                    converted[name] = value
            rows.append(converted)
    return rows


def _series(rows, name):
    return [row.get(name, float("nan")) for row in rows]


def plot_control(control_csv, output_png):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required: python -m pip install matplotlib"
        ) from exc

    rows = _read_numeric_rows(control_csv)
    if not rows:
        raise SystemExit("control.csv contains no feedback rows")

    base_ms = rows[0].get("device_ms", 0.0)
    seconds = [
        (row.get("device_ms", base_ms) - base_ms) / 1000.0
        for row in rows
    ]

    figure, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    figure.suptitle("STM32 balance control feedback")

    axes[0].plot(
        seconds,
        _series(rows, "mcu_position_px"),
        label="vision position received by MCU (px)",
    )
    axes[0].plot(
        seconds,
        _series(rows, "control_error_px"),
        label="target - position (px)",
    )
    axes[0].set_ylabel("Position / error")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(
        seconds,
        _series(rows, "mcu_velocity_px_s"),
        label="vision velocity received by MCU (px/s)",
        color="tab:orange",
    )
    axes[1].set_ylabel("Velocity")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(
        seconds, _series(rows, "p_term"), label="outer P (rod deg)"
    )
    axes[2].plot(
        seconds, _series(rows, "i_term"), label="outer I (rod deg)"
    )
    axes[2].plot(
        seconds, _series(rows, "d_term"), label="outer D (rod deg)"
    )
    axes[2].plot(
        seconds,
        _series(rows, "motor_command"),
        label="motor command",
        linewidth=1.5,
        color="black",
    )
    axes[2].set_ylabel("Rod deg / motor")
    axes[2].legend(loc="upper right", ncol=4)
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(
        seconds,
        _series(rows, "vision_age_ms"),
        label="vision age at motor TX (ms)",
    )
    axes[3].step(
        seconds,
        _series(rows, "motor_status"),
        where="post",
        label="HAL motor status (0=OK)",
    )
    axes[3].set_ylabel("Delay / status")
    axes[3].set_xlabel("MaixCAM session time (s)")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.3)

    figure.tight_layout()
    output_path = Path(output_png)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description="Plot vision input, PID terms, and STM32 motor commands"
    )
    parser.add_argument("control_csv", help="control.csv from one test session")
    parser.add_argument(
        "--output",
        help="output PNG path (default: control_curves.png beside CSV)",
    )
    args = parser.parse_args()

    control_path = Path(args.control_csv)
    output = (
        Path(args.output)
        if args.output
        else control_path.with_name("control_curves.png")
    )
    print(plot_control(control_path, output))


if __name__ == "__main__":
    main()
