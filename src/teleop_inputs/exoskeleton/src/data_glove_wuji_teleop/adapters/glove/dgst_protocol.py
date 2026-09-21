"""Bounded, strict DGST v3 metadata decoding; firmware calibration is descriptive only."""

from __future__ import annotations

import hashlib
import http.client
import json
import math
import tarfile
from dataclasses import dataclass
from typing import Any


MAX_METADATA_BYTES = 1024 * 1024
METADATA_FILES = frozenset({
    "device.json", "sensors.json", "calibration.json", "encoder_calibration.json",
})


class ProtocolError(RuntimeError):
    """The peer sent an unsupported or malformed stream."""


def _object(pairs: list[tuple[str, Any]]) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _constant(value: str) -> None:
    raise ProtocolError(f"non-finite JSON number: {value}")


def _float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ProtocolError("non-finite JSON number")
    return number


def json_object(data: bytes, *, limit: int) -> dict:
    if not data or len(data) > limit:
        raise ProtocolError("invalid JSON payload length")
    try:
        result = json.loads(
            data.decode("utf-8"), object_pairs_hook=_object,
            parse_constant=_constant, parse_float=_float,
        )
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise ProtocolError("invalid UTF-8 JSON") from exc
    if not isinstance(result, dict):
        raise ProtocolError("JSON payload must be an object")
    return result


@dataclass(frozen=True)
class StreamMetadata:
    device: dict
    sensors: dict
    sha256: str
    calibration: dict
    encoder_calibration: dict


def parse_metadata(
    data: bytes,
    etag: str | None,
    *,
    expected_device_id: str | None = None,
    expected_hand: str | None = None,
) -> StreamMetadata:
    if not data or len(data) > MAX_METADATA_BYTES:
        raise ProtocolError("metadata.tar exceeds the 1 MiB limit or is empty")
    digest = hashlib.sha256(data).hexdigest()
    if etag != f'"{digest}"':
        raise ProtocolError("metadata.tar ETag SHA256 mismatch")

    # Inspect physical headers, not getmembers(): tarfile otherwise silently accepts
    # PAX/GNU extension records. Nothing is ever extracted to the filesystem.
    files: dict[str, bytes] = {}
    offset = 0
    while offset + 512 <= len(data):
        block = data[offset:offset + 512]
        if block == bytes(512):
            if any(data[offset:]) or len(data) - offset < 1024:
                raise ProtocolError("invalid tar end marker")
            break
        try:
            member = tarfile.TarInfo.frombuf(block, "utf-8", "strict")
        except (tarfile.HeaderError, UnicodeError, ValueError) as exc:
            raise ProtocolError("invalid metadata tar header") from exc
        if member.type not in (tarfile.REGTYPE, tarfile.AREGTYPE):
            raise ProtocolError("metadata tar members must be ordinary files")
        if member.name not in METADATA_FILES or member.name in files:
            raise ProtocolError(f"unexpected or duplicate metadata member: {member.name!r}")
        offset += 512
        if member.size <= 0 or member.size > MAX_METADATA_BYTES or offset + member.size > len(data):
            raise ProtocolError("invalid metadata member size")
        files[member.name] = data[offset:offset + member.size]
        padded_size = (member.size + 511) // 512 * 512
        if any(data[offset + member.size:offset + padded_size]):
            raise ProtocolError("nonzero tar member padding")
        offset += padded_size
    else:
        raise ProtocolError("missing metadata tar end marker")
    if files.keys() != METADATA_FILES:
        raise ProtocolError("metadata tar must contain exactly four JSON files")
    docs = {name: json_object(raw, limit=MAX_METADATA_BYTES) for name, raw in files.items()}
    device = docs["device.json"]
    sensors = docs["sensors.json"]
    for doc, schema in ((device, "dataglove.stream_device"), (sensors, "dataglove.stream_sensors")):
        if doc.get("schema") != schema or type(doc.get("schema_version")) is not int or doc["schema_version"] != 1:
            raise ProtocolError(f"unsupported metadata schema: {schema}")
    device_id = device.get("device_id")
    if not isinstance(device_id, str) or not device_id.strip():
        raise ProtocolError("invalid device_id")
    if expected_device_id is not None and device_id != expected_device_id:
        raise ProtocolError("device_id does not match the selected glove profile")
    if device.get("hand") not in ("left", "right"):
        raise ProtocolError("invalid device hand")
    if expected_hand is not None and device["hand"] != expected_hand:
        raise ProtocolError("device hand does not match the selected glove profile")
    boot = device.get("boot_id")
    if not isinstance(boot, str) or not boot.strip():
        raise ProtocolError("invalid device boot_id")
    for key in ("calibration", "encoder_calibration"):
        reference = device.get(key)
        if not isinstance(reference, dict) or reference.get("file") != f"{key}.json":
            raise ProtocolError(f"invalid {key} metadata reference")
        if reference.get("sha256") != hashlib.sha256(files[f"{key}.json"]).hexdigest():
            raise ProtocolError(f"{key} SHA256 mismatch")
    encoder = sensors.get("encoder")
    if not isinstance(encoder, dict):
        raise ProtocolError("encoder metadata missing")
    channels = encoder.get("channels")
    mapping = encoder.get("cs_by_joint")
    if type(channels) is not int or not 1 <= channels <= 65535:
        raise ProtocolError("invalid encoder channel count")
    if (not isinstance(mapping, list) or len(mapping) != channels
            or any(type(cs) is not int for cs in mapping)
            or sorted(mapping) != list(range(channels))):
        raise ProtocolError("invalid encoder cs_by_joint mapping")
    if (encoder.get("unit") != "deg"
            or encoder.get("angle_convention") != "raw_absolute_0_360"
            or encoder.get("range") != [0, 360]
            or encoder.get("range_upper_exclusive") is not True
            or encoder.get("joint_order") != "J1..Jn"
            or encoder.get("calibration_applied_to_samples") is not False):
        raise ProtocolError("encoder samples must be uncalibrated raw absolute degrees [0,360)")
    timestamp = sensors.get("timestamp")
    if not isinstance(timestamp, dict) or any(timestamp.get(k) != v for k, v in {
        "field": "timestamp_ns", "unit": "ns", "clock": "CLOCK_MONOTONIC",
        "scope": "same_device_boot_only",
    }.items()):
        raise ProtocolError("unsupported sensor timestamp semantics")
    return StreamMetadata(device, sensors, digest, docs["calibration.json"], docs["encoder_calibration.json"])


def fetch_metadata(
    host: str, *, port: int = 5570, timeout: float = 15.0,
    source_address: str | None = None,
    expected_device_id: str | None = None, expected_hand: str | None = None,
) -> StreamMetadata:
    """Use a direct HTTP socket: proxy environment variables are never consulted."""
    connection = http.client.HTTPConnection(
        host, port, timeout=timeout,
        source_address=(source_address, 0) if source_address is not None else None,
    )
    try:
        connection.request("GET", "/api/stream/metadata", headers={"Accept": "application/x-tar"})
        response = connection.getresponse()
        if response.status != 200:
            raise ProtocolError(f"metadata HTTP status {response.status}")
        length = response.getheader("Content-Length")
        if length is not None:
            try:
                size = int(length)
            except ValueError as exc:
                raise ProtocolError("invalid metadata Content-Length") from exc
            if not 0 < size <= MAX_METADATA_BYTES:
                raise ProtocolError("metadata.tar exceeds the 1 MiB limit or is empty")
        data = response.read(MAX_METADATA_BYTES + 1)
        if length is not None and len(data) != size:
            raise ProtocolError("metadata Content-Length mismatch")
        return parse_metadata(data, response.getheader("ETag"), expected_device_id=expected_device_id, expected_hand=expected_hand)
    finally:
        connection.close()
