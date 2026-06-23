#!/usr/bin/env python3
"""Generate Lely-compatible CANopen artifacts from a motor profile.

The profile keeps the vendor EDS immutable. This tool writes all derived files
to the configured generated directory, then runs dcfgen to produce master.dcf.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError as exc:  # pragma: no cover - exercised by users without PyYAML
    raise SystemExit("PyYAML is required to run this generator") from exc


COUNTER_KEYS = {"SupportedObjects", "SubNumber"}
IDENTITY_SECTIONS = {"1000", "1018sub1", "1018sub2", "1018sub3", "1018sub4"}
IDENTITY_DEVICEINFO_KEYS = {"VendorNumber", "ProductNumber", "RevisionNumber"}
SECTION_RE = re.compile(r"^\[([^\]]+)\]\s*$")
KEY_VALUE_RE = re.compile(r"^([^=;\s][^=]*?)\s*=\s*(.*?)\s*$")


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def relpath(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def parse_int_expr(value: str, node_id: int | None = None) -> int | None:
    value = value.strip()
    if not value:
        return None

    if node_id is not None:
        value = value.replace("$NODEID", str(node_id))

    try:
        return int(value, 0)
    except ValueError:
        pass

    if "+" in value:
        total = 0
        for term in value.split("+"):
            term = term.strip()
            if not term:
                return None
            try:
                total += int(term, 0)
            except ValueError:
                return None
        return total

    return None


def split_inline_comment(line: str) -> tuple[str, str]:
    if ";" not in line:
        return line, ""
    body, comment = line.split(";", 1)
    return body.rstrip(), ";" + comment


def normalize_counter_value(key: str, value: str) -> str:
    if key not in COUNTER_KEYS:
        return value

    parsed = parse_int_expr(value)
    if parsed is None:
        return value
    return str(parsed)


def normalize_eds(
    source: Path,
    destination: Path,
    *,
    identity_policy: str,
) -> None:
    current_section = ""
    normalized_lines: list[str] = []

    for original_line in source.read_text(encoding="utf-8", errors="replace").splitlines():
        section_match = SECTION_RE.match(original_line.strip())
        if section_match:
            current_section = section_match.group(1)
            normalized_lines.append(original_line)
            continue

        body, comment = split_inline_comment(original_line)
        key_match = KEY_VALUE_RE.match(body)
        if not key_match:
            normalized_lines.append(original_line)
            continue

        key = key_match.group(1).strip()
        value = key_match.group(2).strip()
        new_value = normalize_counter_value(key, value)

        if identity_policy == "ignore":
            if current_section in IDENTITY_SECTIONS and key == "DefaultValue":
                new_value = "0x00000000"
            elif current_section == "DeviceInfo" and key in IDENTITY_DEVICEINFO_KEYS:
                new_value = "0x00000000"

        normalized_lines.append(f"{key}={new_value}{comment}")

    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(normalized_lines) + "\n", encoding="utf-8")


def apply_eds_overrides(
    path: Path,
    overrides: dict[str, dict[str, str]],
) -> None:
    """Rewrite specific [section] key=value pairs in a normalized EDS.

    Used to bend the vendor PDO layout into a profile-specified one (for example
    the vendor's proven CSP RxPDO layout) without hand-editing generated files.
    Only existing keys are replaced; the EDS structure is otherwise preserved.
    """
    if not overrides:
        return

    current_section = ""
    output_lines: list[str] = []

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        section_match = SECTION_RE.match(line.strip())
        if section_match:
            current_section = section_match.group(1)
            output_lines.append(line)
            continue

        body, comment = split_inline_comment(line)
        key_match = KEY_VALUE_RE.match(body)
        if key_match and current_section in overrides:
            key = key_match.group(1).strip()
            section_overrides = overrides[current_section]
            if key in section_overrides:
                output_lines.append(f"{key}={section_overrides[key]}{comment}")
                continue

        output_lines.append(line)

    path.write_text("\n".join(output_lines) + "\n", encoding="utf-8")


def parse_eds_sections(path: Path) -> dict[str, dict[str, str]]:
    sections: dict[str, dict[str, str]] = {}
    current_section = ""

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = line.strip()
        section_match = SECTION_RE.match(stripped)
        if section_match:
            current_section = section_match.group(1)
            sections.setdefault(current_section, {})
            continue

        if not current_section:
            continue

        body, _ = split_inline_comment(line)
        key_match = KEY_VALUE_RE.match(body)
        if key_match:
            sections[current_section][key_match.group(1).strip()] = key_match.group(2).strip()

    return sections


def object_name(sections: dict[str, dict[str, str]], index: int, fallback: str) -> str:
    return sections.get(f"{index:04X}", {}).get("ParameterName", fallback)


def decode_mapping_value(value: str, sections: dict[str, dict[str, str]]) -> dict[str, Any] | None:
    parsed = parse_int_expr(value)
    if parsed is None or parsed == 0:
        return None

    index = (parsed >> 16) & 0xFFFF
    subindex = (parsed >> 8) & 0xFF
    bit_length = parsed & 0xFF
    return {
        "index": f"0x{index:04X}",
        "subindex": subindex,
        "bit_length": bit_length,
        "name": object_name(sections, index, f"0x{index:04X}:{subindex:02X}"),
    }


def extract_pdo_mappings(
    sections: dict[str, dict[str, str]],
    *,
    node_id: int,
) -> dict[str, list[dict[str, Any]]]:
    result: dict[str, list[dict[str, Any]]] = {"rpdo": [], "tpdo": []}

    for direction, map_base, comm_base in (("rpdo", 0x1600, 0x1400), ("tpdo", 0x1A00, 0x1800)):
        for pdo_index in range(4):
            mapping_section = f"{map_base + pdo_index:04X}"
            comm_section = f"{comm_base + pdo_index:04X}"
            mapping = sections.get(mapping_section)
            comm = sections.get(comm_section)
            if not mapping:
                continue

            object_count = parse_int_expr(sections.get(f"{mapping_section}sub0", {}).get("DefaultValue", "0"))
            if object_count is None:
                object_count = parse_int_expr(mapping.get("SubNumber", "0"))
                object_count = max((object_count or 1) - 1, 0)

            entries = []
            for subindex in range(1, object_count + 1):
                entry_section = f"{mapping_section}sub{subindex}"
                default_value = sections.get(entry_section, {}).get("DefaultValue")
                if not default_value:
                    continue
                decoded = decode_mapping_value(default_value, sections)
                if decoded:
                    entries.append(decoded)

            cob_id = None
            transmission_type = None
            if comm:
                cob_id = sections.get(f"{comm_section}sub1", {}).get("DefaultValue")
                transmission_type = sections.get(f"{comm_section}sub2", {}).get("DefaultValue")

            result[direction].append(
                {
                    "pdo": pdo_index + 1,
                    "communication_index": f"0x{comm_base + pdo_index:04X}",
                    "mapping_index": f"0x{map_base + pdo_index:04X}",
                    "cob_id": parse_int_expr(cob_id, node_id) if cob_id else None,
                    "transmission_type": parse_int_expr(transmission_type) if transmission_type else None,
                    "entries": entries,
                }
            )

    return result


def build_dcfgen_yaml(profile: dict[str, Any], normalized_eds: Path, root: Path) -> dict[str, Any]:
    master = profile["master"]
    node = profile["node"]
    no_strict = bool(profile.get("generation", {}).get("no_strict", False))

    config: dict[str, Any] = {
        "options": {
            "dcf_path": profile["generation"].get("dcf_dir", "dcf"),
        },
        "master": {
            "node_id": master.get("node_id", 127),
            "baudrate": master.get("baudrate", 1000),
            "heartbeat_consumer": master.get("heartbeat_consumer", False),
            "heartbeat_producer": master.get("heartbeat_producer", 0),
            "sync_period": master.get("sync_period", 0),
            "sync_window": master.get("sync_window", 0),
            "start": master.get("start", True),
            "start_nodes": master.get("start_nodes", True),
            "reset_all_nodes": master.get("reset_all_nodes", False),
            "stop_all_nodes": master.get("stop_all_nodes", False),
            "boot_time": master.get("boot_time", 5000),
        },
        node.get("name", "motor"): {
            "dcf": relpath(normalized_eds, root),
            "node_id": node.get("node_id", 1),
            "mandatory": node.get("mandatory", True),
            "boot": node.get("boot", True),
            "reset_communication": node.get("reset_communication", True),
        },
    }

    if no_strict:
        config["options"]["no_strict"] = True

    return config


def run_checked(command: list[str], *, cwd: Path) -> None:
    print("+ " + " ".join(command))
    subprocess.run(command, cwd=cwd, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        default="config/motors/eyou_phu.yml",
        help="motor profile YAML path",
    )
    parser.add_argument("--skip-tools", action="store_true", help="write generated files but do not run dcfgen/dcfchk")
    args = parser.parse_args()

    root = repo_root()
    profile_path = (root / args.profile).resolve()
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))

    name = profile["name"]
    generated_dir = (root / profile.get("generation", {}).get("generated_dir", f"generated/canopen/{name}")).resolve()
    vendor_eds = (root / profile["vendor_eds"]).resolve()
    normalized_eds = generated_dir / f"{name}.normalized.eds"
    dcfgen_yaml = generated_dir / f"{name}.dcfgen.yml"
    summary_json = generated_dir / f"{name}.summary.json"
    dcf_dir = root / profile.get("generation", {}).get("dcf_dir", "dcf")

    if not vendor_eds.exists():
        raise SystemExit(f"Vendor EDS not found: {vendor_eds}")

    normalize_eds(
        vendor_eds,
        normalized_eds,
        identity_policy=profile.get("identity_policy", "strict"),
    )

    apply_eds_overrides(normalized_eds, profile.get("eds_overrides", {}) or {})

    dcfgen_config = build_dcfgen_yaml(profile, normalized_eds, root)
    dcfgen_yaml.write_text(yaml.safe_dump(dcfgen_config, sort_keys=False), encoding="utf-8")

    sections = parse_eds_sections(normalized_eds)
    summary = {
        "name": name,
        "vendor_eds": relpath(vendor_eds, root),
        "normalized_eds": relpath(normalized_eds, root),
        "master_dcf": relpath(dcf_dir / "master.dcf", root),
        "node_id": profile["node"].get("node_id", 1),
        "master_node_id": profile["master"].get("node_id", 127),
        "identity_policy": profile.get("identity_policy", "strict"),
        "pdo_policy": profile.get("pdo_policy", "vendor-default"),
        "mode_policy": profile.get("mode_policy", "vendor-default"),
        "pdo_mappings": extract_pdo_mappings(sections, node_id=profile["node"].get("node_id", 1)),
    }
    summary_json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {relpath(normalized_eds, root)}")
    print(f"Wrote {relpath(dcfgen_yaml, root)}")
    print(f"Wrote {relpath(summary_json, root)}")

    if args.skip_tools:
        return 0

    dcf_dir.mkdir(parents=True, exist_ok=True)
    dcfgen_command = ["dcfgen", "-r"]
    if profile.get("generation", {}).get("no_strict", False):
        dcfgen_command.append("-S")
    dcfgen_command.extend(["-d", relpath(dcf_dir, root)])
    dcfgen_command.append(relpath(dcfgen_yaml, root))
    run_checked(dcfgen_command, cwd=root)
    run_checked(["dcfchk", "-n", str(profile["master"].get("node_id", 127)), relpath(dcf_dir / "master.dcf", root)], cwd=root)

    return 0


if __name__ == "__main__":
    sys.exit(main())
