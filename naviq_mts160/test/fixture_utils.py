"""Helpers shared by the replay tests and the fixture tool: candump log I/O,
fixture JSON evaluation and synthetic-log generation.  No ROS imports."""

from __future__ import annotations

import json
import os
import re
import statistics
from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple

from naviq_mts160 import decoder

_LINE = re.compile(r"^\((?P<t>\d+\.\d+)\)\s+(?P<iface>\S+)\s+(?P<id>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)(?:\s+.*)?$")


@dataclass(frozen=True)
class LogFrame:
    t: float
    can_id: int
    data: bytes


def read_candump(path: str) -> List[LogFrame]:
    out: List[LogFrame] = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            m = _LINE.match(line)
            if not m:
                continue
            if "R" in m.group("data").upper():
                continue   # remote frame
            out.append(LogFrame(float(m.group("t")), int(m.group("id"), 16), bytes.fromhex(m.group("data"))))
    return out


def write_candump(path: str, frames: Iterable[Tuple[float, int, bytes]], iface: str = "can0") -> None:
    with open(path, "w", encoding="utf-8") as fh:
        for t, can_id, data in frames:
            fh.write(f"({t:.6f}) {iface} {can_id:03X}#{bytes(data).hex().upper()}\n")


def load_fixture(json_path: str) -> dict:
    with open(json_path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def fixture_pairs(directory: str) -> List[Tuple[str, str]]:
    pairs = []
    if not os.path.isdir(directory):
        return pairs
    for name in sorted(os.listdir(directory)):
        if name.endswith(".log"):
            stem = name[:-4]
            js = os.path.join(directory, stem + ".json")
            if os.path.exists(js):
                pairs.append((os.path.join(directory, name), js))
    return pairs


# ---------------------------------------------------------- decoded stream
@dataclass
class DecodedSample:
    t: float
    kind: str            # "track" | "markers" | "navicode" | "heartbeat"
    value: object


def decode_log(frames: List[LogFrame], node_id: int) -> List[DecodedSample]:
    out = []
    for f in frames:
        try:
            dec = decoder.decode_frame(f.can_id, f.data, node_id)
        except decoder.WrongLengthError:
            continue
        if dec is None:
            continue
        kind = {decoder.TrackFrame: "track", decoder.MarkerFrame: "markers",
                decoder.NavicodeFrame: "navicode", decoder.HeartbeatFrame: "heartbeat"}[type(dec)]
        out.append(DecodedSample(f.t, kind, dec))
    return out


# ---------------------------------------------------------- expectations
class ExpectationError(AssertionError):
    pass


def _get(sample_value, field: str):
    if isinstance(sample_value, dict):
        return sample_value[field]
    return getattr(sample_value, field)


def evaluate_segment(samples: List[DecodedSample], segment: dict, t0: float) -> List[str]:
    """Return a list of failure strings (empty = pass) for one fixture segment."""
    failures: List[str] = []
    ts, te = t0 + float(segment["t_start"]), t0 + float(segment["t_end"])
    seg = [s for s in samples if ts <= s.t <= te]
    tracks = [s for s in seg if s.kind == "track"]
    navis = [s for s in seg if s.kind == "navicode"]
    expect: Dict[str, object] = segment.get("expect", {})
    name = segment.get("name", "?")

    if not tracks and any(k not in ("navicode_counter_changes",) for k in expect):
        return [f"{name}: no TPDO1 frames in segment"]

    for field, rule in expect.items():
        if field == "navicode_counter_changes":
            changes = 0
            prev = None
            for s in navis:
                c = s.value.counter
                if prev is not None and c != prev:
                    changes += 1
                prev = c
            if changes != int(rule):
                failures.append(f"{name}: navicode counter changed {changes}x, expected {rule}")
            continue
        if field == "tpdo1_rate_hz":
            if len(tracks) >= 2:
                rate = (len(tracks) - 1) / max(tracks[-1].t - tracks[0].t, 1e-9)
                lo, hi = rule.get("min", 0), rule.get("max", 1e9)
                if not lo <= rate <= hi:
                    failures.append(f"{name}: TPDO1 rate {rate:.1f} Hz outside [{lo}, {hi}]")
            continue
        values = [_get(s.value, field) for s in tracks]
        if isinstance(rule, dict):
            if "mean" in rule:
                mean = statistics.fmean(float(v) for v in values)
                if abs(mean - float(rule["mean"])) > float(rule.get("tol", 0.5)):
                    failures.append(f"{name}: {field} mean {mean:.2f} vs {rule['mean']} +/-{rule.get('tol', 0.5)}")
            if "min" in rule and any(float(v) < float(rule["min"]) for v in values):
                failures.append(f"{name}: {field} below {rule['min']} (min {min(values)})")
            if "max" in rule and any(float(v) > float(rule["max"]) for v in values):
                failures.append(f"{name}: {field} above {rule['max']} (max {max(values)})")
            if "fraction_true_min" in rule:
                frac = sum(1 for v in values if v) / len(values)
                if frac < float(rule["fraction_true_min"]):
                    failures.append(f"{name}: {field} true {frac:.2%} < {rule['fraction_true_min']:.0%}")
            if "fraction_true_max" in rule:
                frac = sum(1 for v in values if v) / len(values)
                if frac > float(rule["fraction_true_max"]):
                    failures.append(f"{name}: {field} true {frac:.2%} > {rule['fraction_true_max']:.0%}")
            if "equals" in rule and any(v != rule["equals"] for v in values):
                failures.append(f"{name}: {field} not always {rule['equals']}")
        else:
            if any(v != rule for v in values):
                failures.append(f"{name}: {field} not always {rule!r} (values {sorted(set(values))[:6]})")
    return failures


def evaluate_fixture(frames: List[LogFrame], fixture: dict) -> List[str]:
    node_id = int(fixture.get("node_id", 10))
    samples = decode_log(frames, node_id)
    if not frames:
        return ["empty log"]
    t0 = frames[0].t
    failures: List[str] = []
    for seg in fixture.get("segments", []):
        failures.extend(evaluate_segment(samples, seg, t0))
    return failures


# ------------------------------------------------------------- synthetic
def synthetic_session(node_id: int = 10, seconds: float = 2.0, t0: float = 1_700_000_000.0,
                      lpos: int = 0, rpos: int = 0, lang: int = 0, rang: int = 0, strength: int = 3,
                      markers=(0, 0, 0, 0), code: int = 1, counter: int = 3, **flags) -> List[Tuple[float, int, bytes]]:
    """A plausible stream: TPDO1 @10 ms, TPDO2 @20 ms, TPDO3 @50 ms, HB @1 s."""
    ids = decoder.cob_ids(node_id)
    out = []
    n = int(seconds / 0.005)
    for k in range(n):
        t = t0 + k * 0.005
        if k % 2 == 0:
            out.append((t, ids["tpdo1"], decoder.encode_tpdo1(lpos, rpos, lang, rang, strength, **flags)))
        if k % 4 == 0:
            out.append((t + 0.0001, ids["tpdo2"], decoder.encode_tpdo2(*markers)))
        if k % 10 == 0:
            out.append((t + 0.0002, ids["tpdo3"], decoder.encode_tpdo3(code, counter)))
        if k % 200 == 0:
            out.append((t + 0.0003, ids["heartbeat"], decoder.encode_heartbeat(decoder.NMT_OPERATIONAL)))
    return out
