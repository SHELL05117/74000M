# Protocol contract

`log_file_schema.yaml` is the PC-facing contract for the current robot writer:
V5L2 file format 2.0 and LogFrame schema 3.1. The executable layout is encoded
in `pc/src/statusmonitor/protocol/schema31.py`; its NumPy ABI assertions lock the
160/28/1536/48-byte structures.

The tests build deterministic valid, truncated, sequence-gap, byte-corrupt and
identity-mismatch recordings. `test_protocol.py` also checks the current C++
fixture when a Release build is present.

Compatibility is intentionally strict. A major schema change, frame-size
change, endian mismatch, or unknown file format fails before any analysis.
Missing availability bits produce `NOT AVAILABLE`, not inferred values.
