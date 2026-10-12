#!/usr/bin/env python3
"""Convert an INAV CLI diff file into a targetConfiguration() C skeleton.

The output is meant to be copied into a target-specific config.c and then
completed manually with only the board-safe settings that make sense at build
time. Runtime-only values (PID tuning, profiles, blackbox flags, etc.) are
emitted as comments so the maintainer can keep them in the CLI file instead of
hard-coding them into firmware defaults.
"""

from __future__ import annotations

import argparse
import pathlib
import re
from dataclasses import dataclass


SET_RE = re.compile(r"^set\s+([A-Za-z0-9_]+)\s*=\s*(.+?)\s*$")
FEATURE_RE = re.compile(r"^feature\s+([A-Za-z0-9_]+)\s*$")
SERIAL_RE = re.compile(r"^serial\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$")
MIXER_PROFILE_RE = re.compile(r"^mixer_profile\s+(\d+)\s*$")
CONTROL_PROFILE_RE = re.compile(r"^control_profile\s+(\d+)\s*$")
BATTERY_PROFILE_RE = re.compile(r"^battery_profile\s+(\d+)\s*$")
AUX_RE = re.compile(r"^aux\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$")
MMIX_RE = re.compile(r"^mmix\s+(\d+)\s+(.+)$")
BLACKBOX_RE = re.compile(r"^blackbox\s+(.+)$")


@dataclass
class ParsedLine:
    kind: str
    text: str


BOARD_SAFE_SETTERS = {
    # Safe target-time defaults that INAV commonly keeps in target config files.
    "align_board_roll",
    "align_board_pitch",
    "align_board_yaw",
    "airmode_type",
    "motor_pwm_protocol",
    "serialrx_provider",
    "failsafe_procedure",
    "gps_sbas_mode",
    "gps_ublox_use_galileo",
    "gps_ublox_use_beidou",
    "gps_ublox_use_glonass",
    "blackbox_rate_denom",
    "i2c_speed",
}


RUNTIME_ONLY_PREFIXES = (
    "gyro_",
    "acc",
    "ins_",
    "mc_",
    "dterm_",
    "tpa_",
    "rc_",
    "ez_",
    "model_preview_type",
    "motorstop_on_low",
    "antigravity_",
    "smith_predictor_delay",
    "applied_defaults",
)


def parse_cli(path: pathlib.Path) -> list[ParsedLine]:
    parsed: list[ParsedLine] = []
    for raw in path.read_text(encoding="utf-8-sig").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or line in {"batch start", "batch end", "defaults noreboot", "save", "diff all"}:
            continue
        if FEATURE_RE.match(line):
            parsed.append(ParsedLine("feature", line))
        elif SET_RE.match(line):
            parsed.append(ParsedLine("set", line))
        elif SERIAL_RE.match(line):
            parsed.append(ParsedLine("serial", line))
        elif MIXER_PROFILE_RE.match(line) or CONTROL_PROFILE_RE.match(line) or BATTERY_PROFILE_RE.match(line):
            parsed.append(ParsedLine("profile", line))
        elif AUX_RE.match(line) or MMIX_RE.match(line) or BLACKBOX_RE.match(line):
            parsed.append(ParsedLine("runtime", line))
        else:
            parsed.append(ParsedLine("unknown", line))
    return parsed


def classify_set(line: str) -> str:
    match = SET_RE.match(line)
    assert match
    name = match.group(1)
    if name in BOARD_SAFE_SETTERS:
        return "board-safe"
    if name.startswith(RUNTIME_ONLY_PREFIXES):
        return "runtime"
    return "runtime"


def emit(parsed: list[ParsedLine], target_name: str, source_name: str) -> str:
    out: list[str] = []
    out.extend([
        "/*",
        " * Auto-generated from an INAV CLI dump.",
        " *",
        " * Copy the board-safe assignments into a target config.c file and keep",
        " * the remaining tuning/profile lines in the CLI dump.",
        " */",
        "",
        "#include <stdint.h>",
        "#include <platform.h>",
        "",
        "#include \"fc/fc_msp_box.h\"",
        "#include \"io/piniobox.h\"",
        "#include \"sensors/boardalignment.h\"",
        "#include \"sensors/battery.h\"",
        "#include \"config/feature.h\"",
        "#include \"io/serial.h\"",
        "#include \"flight/mixer.h\"",
        "",
        f"// Source: {source_name}",
        f"// Target: {target_name}",
        "void targetConfiguration(void)",
        "{",
    ])

    for item in parsed:
        if item.kind == "feature":
            out.append(f"    // CLI: {item.text}")
        elif item.kind == "serial":
            out.append(f"    // CLI: {item.text}")
        elif item.kind == "profile":
            out.append(f"    // CLI: {item.text}")
        elif item.kind == "runtime":
            out.append(f"    // CLI: {item.text}")
        elif item.kind == "set":
            kind = classify_set(item.text)
            if kind == "board-safe":
                match = SET_RE.match(item.text)
                assert match
                name = match.group(1)
                value = match.group(2)
                out.append(f"    // CLI: {item.text}")
                # Emit only a few common board-safe setters as examples; the user can
                # extend this file if they want firmware defaults baked in.
                if name == "airmode_type":
                    out.append("    // airmodeType is typically set through CLI rather than here.")
                elif name == "motor_pwm_protocol":
                    out.append("    // motor PWM protocol belongs to runtime/user config.")
                elif name == "serialrx_provider":
                    out.append("    // serial RX provider belongs to runtime/user config.")
                elif name == "failsafe_procedure":
                    out.append("    // failsafe procedure belongs to runtime/user config.")
                else:
                    out.append(f"    // board-safe value: {name} = {value}")
            else:
                out.append(f"    // CLI: {item.text}")
        else:
            out.append(f"    // Unhandled CLI line: {item.text}")

    out.extend(["}", ""])
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert INAV CLI diff to target config.c skeleton")
    parser.add_argument("input", type=pathlib.Path, help="Path to INAV_8.0.1_config_all.txt")
    parser.add_argument("--target", default="OMNIBUSF4V3", help="Target name to annotate in output")
    parser.add_argument("--output", type=pathlib.Path, help="Optional output file")
    args = parser.parse_args()

    parsed = parse_cli(args.input)
    text = emit(parsed, args.target, args.input.name)
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
