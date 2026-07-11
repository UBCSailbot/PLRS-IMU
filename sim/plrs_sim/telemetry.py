"""RP2040 telemetry wire format: the record schema and its codec.

The firmware emits one tagged, comma-separated line per record over USB CDC
(see fusion_task.cpp). This module mirrors that wire format:

    F,ts_ms,heading,roll,pitch,hdg_sigma,roll_sigma,pitch_sigma,
      bias,bias_sigma,mag_offset,offset_sigma,gate_rejects,
      mag_gate_rejects                                            fused estimate
    I,ts_ms,qw,qx,qy,qz,gx,gy,gz,ax,ay,az                         raw IMU
    M,ts_ms,ax,ay,az,gx,gy,gz,mx,my,mz                            raw MEMS triad
    G,ts_ms,heading,hdg_sigma,valid,mode,error                    raw GNSS attitude
    # text                                                        human diagnostic

parse_line is tolerant: a malformed or partial line (a glitched byte, an
interleaved diagnostic) yields None rather than raising, so the monitor can
ride out a noisy link.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import MISSING, dataclass, fields
from typing import Annotated, get_args, get_origin, get_type_hints

from .types import Quaternion, Vec3

# The record dataclasses ARE the wire schema: field order is token order,
# Vec3/Quaternion fields flatten to their components, the Annotated metadata
# is the wire precision, and a trailing run of defaulted fields is a
# format-version boundary (all present or all absent). parse_line,
# format_record, and the accepted field counts all derive from them, so the
# three cannot drift apart.

_Deg = Annotated[float, 3]
_Quat = Annotated[Quaternion, 5]
_Gyro = Annotated[Vec3, 5]
_Accel = Annotated[Vec3, 4]
_Mag = Annotated[Vec3, 5]


_Dps = Annotated[float, 4]


@dataclass(frozen=True, slots=True, kw_only=True)
class FusionRecord:
    timestamp_ms: int
    heading_deg: _Deg
    roll_deg: _Deg
    pitch_deg: _Deg
    heading_sigma_deg: _Deg
    roll_sigma_deg: _Deg
    pitch_sigma_deg: _Deg
    # EKF-internal states, for diagnosing heading drift. Defaults keep
    # pre-debug captures parsing; NaN marks "not in this capture" without
    # inventing a value.
    gyro_bias_dps: _Dps = math.nan
    gyro_bias_sigma_dps: _Dps = math.nan
    mag_offset_deg: _Deg = math.nan
    mag_offset_sigma_deg: _Deg = math.nan
    gate_rejects: int = 0
    mag_gate_rejects: int = 0


@dataclass(frozen=True, slots=True, kw_only=True)
class ImuRecord:
    timestamp_ms: int
    orientation: _Quat
    angular_velocity_rad_s: _Gyro
    accel_ms2: _Accel


@dataclass(frozen=True, slots=True, kw_only=True)
class MemsRecord:
    timestamp_ms: int
    accel_ms2: _Accel
    angular_velocity_rad_s: _Gyro
    magnetic_field_au: _Mag


@dataclass(frozen=True, slots=True, kw_only=True)
class GnssRecord:
    timestamp_ms: int
    heading_deg: _Deg
    heading_sigma_deg: _Deg
    valid: bool
    # Raw AttEuler mode/error, for diagnosing why valid is false (float
    # ambiguity vs no-attitude vs a flagged baseline). Default to the
    # no-solution codes so older logs without these fields still parse.
    mode: int = 0
    error: int = 0


@dataclass(frozen=True, slots=True, kw_only=True)
class DiagRecord:
    text: str


Record = FusionRecord | ImuRecord | MemsRecord | GnssRecord | DiagRecord

_NESTED_COMPONENTS = {Vec3: ("x", "y", "z"), Quaternion: ("w", "x", "y", "z")}


def _parse_float(token: str) -> float:
    # Arduino Print renders out-of-range floats as "ovf" and NaN as "nan";
    # a NaN'd filter state must stay parseable, not drop the whole line.
    if token == "ovf":
        return math.inf
    if token == "nan":
        return math.nan
    return float(token)


def _format_float(v: float, prec: int) -> str:
    if math.isnan(v):
        return "nan"
    return "ovf" if not math.isfinite(v) else f"{v:.{prec}f}"


@dataclass(frozen=True, slots=True)
class _FieldCodec:
    """Wire layout of one record field: `width` consecutive tokens."""

    name: str
    width: int
    parse: Callable[[Sequence[str]], object]
    unparse: Callable[[object], list[str]]
    optional: bool


def _field_codec(name: str, hint: type, optional: bool) -> _FieldCodec:
    t, prec = (
        (get_args(hint)[0], get_args(hint)[1])
        if get_origin(hint) is Annotated
        else (hint, None)
    )
    if t in _NESTED_COMPONENTS:
        comps = _NESTED_COMPONENTS[t]
        return _FieldCodec(
            name,
            len(comps),
            lambda toks: t(
                **{c: _parse_float(k) for c, k in zip(comps, toks, strict=True)}
            ),
            lambda v: [_format_float(getattr(v, c), prec) for c in comps],
            optional,
        )
    if t is float:
        return _FieldCodec(
            name,
            1,
            lambda toks: _parse_float(toks[0]),
            lambda v: [_format_float(v, prec)],
            optional,
        )
    if t is bool:
        return _FieldCodec(
            name,
            1,
            lambda toks: toks[0] == "1",
            lambda v: ["1" if v else "0"],
            optional,
        )
    if t is int:
        return _FieldCodec(
            name, 1, lambda toks: int(toks[0]), lambda v: [str(v)], optional
        )
    raise TypeError(f"unsupported wire field type: {t!r}")


@dataclass(frozen=True, slots=True)
class _RecordCodec:
    cls: type
    fields: tuple[_FieldCodec, ...]
    widths: tuple[int, ...]  # accepted token counts, including the tag


def _record_codec(cls: type) -> _RecordCodec:
    hints = get_type_hints(cls, include_extras=True)
    codecs = tuple(
        _field_codec(f.name, hints[f.name], f.default is not MISSING)
        for f in fields(cls)
    )
    full = 1 + sum(f.width for f in codecs)
    required = full - sum(f.width for f in codecs if f.optional)
    widths = (required,) if required == full else (required, full)
    return _RecordCodec(cls, codecs, widths)


_CODECS = {
    "F": _record_codec(FusionRecord),
    "I": _record_codec(ImuRecord),
    "M": _record_codec(MemsRecord),
    "G": _record_codec(GnssRecord),
}
_TAGS = {codec.cls: tag for tag, codec in _CODECS.items()}


def parse_line(line: str) -> Record | None:
    """Parse one telemetry line into a record, or None if it is not valid.

    A `#`-prefixed line is a diagnostic; anything else is dispatched on its
    tag. Wrong field counts and unparseable numbers return None.
    """
    line = line.strip()
    if not line:
        return None
    if line.startswith("#"):
        return DiagRecord(text=line[1:].strip())

    tokens = line.split(",")
    codec = _CODECS.get(tokens[0])
    if codec is None or len(tokens) not in codec.widths:
        return None

    kwargs = {}
    at = 1
    try:
        for fc in codec.fields:
            if at >= len(tokens):
                break  # absent optional tail keeps its defaults
            kwargs[fc.name] = fc.parse(tokens[at : at + fc.width])
            at += fc.width
        return codec.cls(**kwargs)
    except ValueError:
        return None


def format_record(r: FusionRecord | ImuRecord | MemsRecord | GnssRecord) -> str:
    """Render a record as its wire line, the inverse of parse_line."""
    tag = _TAGS[type(r)]
    tokens = [tag]
    for fc in _CODECS[tag].fields:
        tokens += fc.unparse(getattr(r, fc.name))
    return ",".join(tokens)
