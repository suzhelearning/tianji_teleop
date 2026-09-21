# Viewer Native PICO TJVR Recording Design

## Goal

Allow a live PICO + MuJoCo run to capture the exact protocol-valid UDP input
stream so the same corrected skeleton trajectory can later be replayed through
SPARK and the velocity QP for deterministic parameter comparison.

## Command-line interface

Recording is opt-in:

```text
--pico-record PATH.tjvr
```

The option requires `--pico-teleop` and a non-empty path.  Viewer startup
fails before the control thread starts if the destination already exists or
cannot be created.  This prevents accidental overwrite and prevents a test
from appearing to record when it cannot do so.  Without the option, receiver
and control behavior remain unchanged.

## File format

The writer produces the existing TJVR trace container consumed by
`vr_data/tools/tjvr_format.py` and `replay_pico_udp_trace.py`:

```text
header  = little-endian <4sHHQ>
          magic "TJVT", container version 1,
          packet size, record count

record  = little-endian <q> relative_receive_time_ns
          followed by the unmodified TJVR packet bytes
```

The first protocol-valid packet fixes the packet size and recording epoch.
Relative time is measured from that packet's monotonic receive time, making
the first record time zero.  Every subsequent packet must have the same size.
The recorder stores raw packet bytes without rebasing sequence, source time,
bridge time, tracking epoch, corrected skeleton, or CRC.

## Data flow and acceptance point

```text
recvfrom
  -> decode/CRC/protocol validation
  -> recorder append of original bytes and receive time
  -> stream gate (jump/order/epoch decisions)
  -> controller exchange
```

Recording occurs after packet-format and CRC validation but before the stream
gate.  Therefore a deterministic replay reproduces jump, ordering, epoch, and
resynchronization decisions instead of silently deleting the packets that
caused them.  Malformed and CRC-invalid datagrams are not recorded.

## Components

`PicoTraceRecorder` is a receiver-thread-owned streaming writer in dedicated
header/source files.  It creates the destination when recording starts,
writes a placeholder header when the first valid packet establishes packet
size, appends records without queueing or allocating per packet, and seeks
back to write the final count during `finish()`.

`PicoUdpReceiverOptions` carries the optional path.  `PicoUdpReceiver` invokes
the recorder at the validation point and exposes recording status in
`PicoReceiverStats`: configured, active, failed, record count, and a bounded
diagnostic string.  No file I/O runs on the 200 Hz control thread.

The Viewer parses and validates `--pico-record`, passes it to the receiver,
prints `pico_record_path=...` at startup, and prints final record count/status
after stopping the receiver.

## Failure handling

- Existing destination: fail Viewer startup; never overwrite.
- No valid packet: finish with zero records and report `no_valid_packets`.
- Mixed valid packet sizes or runtime write/seek failure: disable further
  recording, preserve receiver/control operation, and report a failed status.
- Normal Viewer close, duration expiry, exception cleanup, or headless exit:
  stop/join the receiver and finalize the record count exactly once.
- A successfully finalized trace must be readable immediately by the existing
  Python loader and replay script.

## Testing and acceptance

Tests cover exact binary header/records, original-packet byte preservation,
relative timing, existing-file refusal, malformed/CRC packet omission, mixed
packet-size failure, idempotent finish, and receiver integration.  A headless
Viewer integration test records replayed TJVR v4 packets, loads the result with
the existing Python format helper, and confirms frame count, packet size, and
source timestamps.

Acceptance requires all repository tests to pass and a manual live command to
produce a trace that can be replayed with:

```bash
python3 /home/zj/current_robotics/TJ_arm/vr_data/tools/replay_pico_udp_trace.py \
  --input PATH.tjvr --port 15000 --lead 0.7
```
