"""Create a dependency-free SVG plot from recorded STM32 feedback.

The input may be a ``stm32_feedback.csv`` file, a session directory containing
that file, or a ``telemetry.jsonl`` file containing ``stm32_feedback`` packets.
"""

import argparse
import csv
import html
import json
import math
from pathlib import Path


PLOTS = (
    (
        "Position and control error",
        "px",
        (
            ("position_px", "position", "#54c7ec"),
            ("target_px", "target", "#8ee38e"),
            ("control_error_px", "error", "#ffb454"),
        ),
    ),
    (
        "Velocity",
        "px/s",
        (("velocity_px_s", "velocity", "#a78bfa"),),
    ),
    (
        "Outer PID target-angle components",
        "rod deg",
        (
            ("p_term", "P", "#ff6b6b"),
            ("i_term", "I", "#4ecdc4"),
            ("d_term", "D", "#ffe66d"),
        ),
    ),
    (
        "Actual signed motor command",
        "speed command",
        (("motor_command", "command", "#f78c6c"),),
    ),
    (
        "Vision age at motor command",
        "ms",
        (("vision_age_ms", "vision age", "#82aaff"),),
    ),
)


def _number(value):
    if value in (None, ""):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _integer(value, default=0):
    number = _number(value)
    return int(number) if number is not None else int(default)


def _resolve_input(path):
    source = Path(path)
    if source.is_dir():
        csv_path = source / "stm32_feedback.csv"
        if csv_path.is_file():
            return csv_path
        jsonl_path = source / "telemetry.jsonl"
        if jsonl_path.is_file():
            return jsonl_path
        raise FileNotFoundError(
            "session has no stm32_feedback.csv or telemetry.jsonl"
        )
    if not source.is_file():
        raise FileNotFoundError(str(source))
    return source


def load_feedback(path):
    source = _resolve_input(path)
    if source.suffix.lower() == ".jsonl":
        rows = []
        with source.open(encoding="utf-8") as handle:
            for line in handle:
                try:
                    packet = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if packet.get("type") == "stm32_feedback":
                    rows.append(packet)
    else:
        with source.open(newline="", encoding="utf-8-sig") as handle:
            rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("STM32 feedback log has no data rows")
    return source, rows


def _unwrap_u32(values):
    result = []
    offset = 0
    previous = None
    for value in values:
        current = int(value) & 0xFFFFFFFF
        if previous is not None and current < previous:
            if previous - current > 0x80000000:
                offset += 0x100000000
        result.append(current + offset)
        previous = current
    return result


def prepare_rows(rows):
    prepared = []
    for row in rows:
        item = dict(row)
        position = _number(item.get("position_px"))
        error = _number(item.get("control_error_px"))
        if position is not None and error is not None:
            item["target_px"] = position + error
        prepared.append(item)

    mcu_values = [_integer(row.get("mcu_ms")) for row in prepared]
    unwrapped = _unwrap_u32(mcu_values)
    first = unwrapped[0]
    for row, mcu_ms in zip(prepared, unwrapped):
        row["_time_s"] = (mcu_ms - first) / 1000.0
    return prepared


def summarize(rows):
    sequences = [_integer(row.get("seq")) & 0xFFFFFFFF for row in rows]
    inferred_gaps = 0
    gap_rows = []
    for index in range(1, len(rows)):
        delta = (sequences[index] - sequences[index - 1]) & 0xFFFFFFFF
        raw_gap = rows[index].get("seq_gap")
        if raw_gap in (None, ""):
            gap = delta - 1 if 1 < delta < 0x80000000 else 0
        else:
            # New logs carry the gap observed on MaixCAM.  Do not infer from
            # rows missing on Windows, because that would mislabel UDP loss as
            # an STM32 USART6-busy drop.
            gap = _integer(raw_gap)
        if gap > 0:
            inferred_gaps += gap
            gap_rows.append(index)

    status_counts = {status: 0 for status in range(4)}
    status_rows = []
    for index, row in enumerate(rows):
        status = _integer(row.get("motor_status"), -1)
        if status in status_counts:
            status_counts[status] += 1
        if status != 0:
            status_rows.append(index)

    ages = [
        value
        for value in (_number(row.get("vision_age_ms")) for row in rows)
        if value is not None
    ]
    duration_s = rows[-1]["_time_s"] - rows[0]["_time_s"]
    return {
        "rows": len(rows),
        "duration_s": max(0.0, duration_s),
        "sequence_gaps": inferred_gaps,
        "gap_rows": gap_rows,
        "status_rows": status_rows,
        "status_counts": status_counts,
        "max_vision_age_ms": max(ages) if ages else 0.0,
    }


def _ticks(minimum, maximum, count=5):
    if maximum <= minimum:
        return [minimum]
    return [
        minimum + (maximum - minimum) * index / (count - 1)
        for index in range(count)
    ]


def _range_for(rows, series):
    values = []
    for key, _label, _color in series:
        values.extend(
            value
            for value in (_number(row.get(key)) for row in rows)
            if value is not None
        )
    if not values:
        return -1.0, 1.0
    minimum = min(values)
    maximum = max(values)
    if minimum == maximum:
        padding = max(1.0, abs(minimum) * 0.1)
    else:
        padding = (maximum - minimum) * 0.08
    return minimum - padding, maximum + padding


def _polyline_points(rows, key, x_map, y_map):
    groups = []
    current = []
    for row in rows:
        value = _number(row.get(key))
        if value is None:
            if current:
                groups.append(current)
                current = []
            continue
        current.append("{:.2f},{:.2f}".format(x_map(row["_time_s"]), y_map(value)))
    if current:
        groups.append(current)
    return groups


def render_svg(rows, summary):
    width = 1280
    left = 88
    right = 34
    top = 112
    panel_height = 160
    panel_gap = 24
    plot_width = width - left - right
    height = top + len(PLOTS) * panel_height + (len(PLOTS) - 1) * panel_gap + 64
    maximum_time = max(row["_time_s"] for row in rows)
    minimum_time = min(row["_time_s"] for row in rows)
    if maximum_time <= minimum_time:
        maximum_time = minimum_time + 1.0

    def x_map(value):
        return left + (value - minimum_time) * plot_width / (
            maximum_time - minimum_time
        )

    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" '
        'viewBox="0 0 {} {}">'.format(width, height, width, height),
        "<style>"
        "text{font-family:Segoe UI,Arial,sans-serif;fill:#cbd5e1}"
        ".title{font-size:22px;font-weight:600;fill:#f8fafc}"
        ".summary{font-size:12px;fill:#94a3b8}"
        ".panel-title{font-size:14px;font-weight:600;fill:#e2e8f0}"
        ".axis{font-size:10px;fill:#94a3b8}"
        ".grid{stroke:#263244;stroke-width:1}"
        ".zero{stroke:#52637a;stroke-width:1}"
        ".gap{stroke:#f59e0b;stroke-width:1;stroke-dasharray:5 4}"
        ".status{stroke:#ef4444;stroke-width:1;stroke-dasharray:2 3}"
        "</style>",
        '<rect width="100%" height="100%" fill="#0b1220"/>',
        '<text x="{}" y="38" class="title">STM32 control feedback</text>'.format(
            left
        ),
        (
            '<text x="{}" y="64" class="summary">'
            "{} rows · {:.3f} s · {} MCU log drops · "
            "status OK/ERROR/BUSY/TIMEOUT = {}/{}/{}/{} · "
            "max vision age {:.1f} ms</text>"
        ).format(
            left,
            summary["rows"],
            summary["duration_s"],
            summary["sequence_gaps"],
            summary["status_counts"][0],
            summary["status_counts"][1],
            summary["status_counts"][2],
            summary["status_counts"][3],
            summary["max_vision_age_ms"],
        ),
        '<text x="{}" y="86" class="summary">'.format(left)
        + html.escape(
            "Orange dashed = STM32 feedback seq gap; red dotted = motor_status != HAL_OK"
        )
        + "</text>",
    ]

    x_ticks = _ticks(minimum_time, maximum_time, 7)
    for panel_index, (title, unit, series) in enumerate(PLOTS):
        y = top + panel_index * (panel_height + panel_gap)
        plot_top = y + 24
        plot_bottom = y + panel_height
        plot_height = plot_bottom - plot_top
        y_min, y_max = _range_for(rows, series)

        def y_map(value):
            return plot_bottom - (value - y_min) * plot_height / (
                y_max - y_min
            )

        parts.append(
            '<text x="{}" y="{}" class="panel-title">{}</text>'.format(
                left, y + 14, html.escape(title)
            )
        )
        parts.append(
            '<text x="18" y="{}" class="axis">{}</text>'.format(
                plot_top + plot_height / 2, html.escape(unit)
            )
        )
        for tick in _ticks(y_min, y_max):
            tick_y = y_map(tick)
            parts.append(
                '<line x1="{}" y1="{:.2f}" x2="{}" y2="{:.2f}" class="grid"/>'.format(
                    left, tick_y, width - right, tick_y
                )
            )
            parts.append(
                '<text x="{}" y="{:.2f}" text-anchor="end" class="axis">{:.2f}</text>'.format(
                    left - 8, tick_y + 3, tick
                )
            )
        if y_min < 0 < y_max:
            zero_y = y_map(0)
            parts.append(
                '<line x1="{}" y1="{:.2f}" x2="{}" y2="{:.2f}" class="zero"/>'.format(
                    left, zero_y, width - right, zero_y
                )
            )
        for tick in x_ticks:
            tick_x = x_map(tick)
            parts.append(
                '<line x1="{:.2f}" y1="{}" x2="{:.2f}" y2="{}" class="grid"/>'.format(
                    tick_x, plot_top, tick_x, plot_bottom
                )
            )
            if panel_index == len(PLOTS) - 1:
                parts.append(
                    '<text x="{:.2f}" y="{}" text-anchor="middle" class="axis">{:.2f}s</text>'.format(
                        tick_x, plot_bottom + 18, tick
                    )
                )
        for row_index in summary["gap_rows"]:
            gap_x = x_map(rows[row_index]["_time_s"])
            parts.append(
                '<line x1="{:.2f}" y1="{}" x2="{:.2f}" y2="{}" class="gap"/>'.format(
                    gap_x, plot_top, gap_x, plot_bottom
                )
            )
        for row_index in summary["status_rows"]:
            status_x = x_map(rows[row_index]["_time_s"])
            parts.append(
                '<line x1="{:.2f}" y1="{}" x2="{:.2f}" y2="{}" class="status"/>'.format(
                    status_x, plot_top, status_x, plot_bottom
                )
            )
        legend_x = width - right
        for key, label, color in reversed(series):
            legend_width = 28 + len(label) * 7
            legend_x -= legend_width
            parts.append(
                '<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="{}" '
                'stroke-width="2"/>'.format(
                    legend_x, y + 10, legend_x + 16, y + 10, color
                )
            )
            parts.append(
                '<text x="{}" y="{}" class="axis">{}</text>'.format(
                    legend_x + 20, y + 13, html.escape(label)
                )
            )
        for key, _label, color in series:
            for points in _polyline_points(rows, key, x_map, y_map):
                parts.append(
                    '<polyline fill="none" stroke="{}" stroke-width="1.7" '
                    'stroke-linejoin="round" stroke-linecap="round" '
                    'points="{}"/>'.format(color, " ".join(points))
                )
    parts.append("</svg>")
    return "\n".join(parts)


def plot_feedback(input_path, output_path=None):
    source, raw_rows = load_feedback(input_path)
    rows = prepare_rows(raw_rows)
    summary = summarize(rows)
    if output_path is None:
        output = source.with_name("stm32_feedback.svg")
    else:
        output = Path(output_path)
    output.write_text(render_svg(rows, summary), encoding="utf-8")
    return output, summary


def main():
    parser = argparse.ArgumentParser(
        description="Plot MaixCAM-recorded STM32 control feedback as SVG."
    )
    parser.add_argument(
        "input",
        help="stm32_feedback.csv, telemetry.jsonl, or a session directory",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="output SVG path (default: stm32_feedback.svg beside the log)",
    )
    args = parser.parse_args()
    output, summary = plot_feedback(args.input, args.output)
    print("feedback rows: {}".format(summary["rows"]))
    print("duration: {:.3f} s".format(summary["duration_s"]))
    print("STM32 feedback sequence drops: {}".format(summary["sequence_gaps"]))
    print(
        "motor status OK/ERROR/BUSY/TIMEOUT: {}/{}/{}/{}".format(
            summary["status_counts"][0],
            summary["status_counts"][1],
            summary["status_counts"][2],
            summary["status_counts"][3],
        )
    )
    print("max vision age: {:.1f} ms".format(summary["max_vision_age_ms"]))
    print("plot:", output)


if __name__ == "__main__":
    main()
