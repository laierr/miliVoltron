# Capture fixtures

Place sanitized UART captures here. Timestamped `*.raw.log` files are preferred
because they preserve observed packet cadence. The test harness automatically
discovers:

- `*.raw.log` — miliVoltron raw logs, including packet timestamps;
- `*.bin` — raw UART bytes;
- `*.hex` — hexadecimal bytes accepted by `--hex`;
- `*.jsonl` — miliVoltron JSONL records containing `raw_hex`.

Binary and plain hexadecimal fixtures remain useful for framing and decoder
tests, but they do not carry original capture timing and are excluded from
cadence assertions.

Each non-empty supported file must contain at least one complete frame. Keep
different scenarios in separate, descriptively named files, for example:

```text
passive-startup.raw.log
inquisitor-normal-cycle.jsonl
bms-reconnect.bin
synthetic-checksum-errors.raw.log
```

Before committing captures, replace battery and controller serial numbers with
same-length values and check for other identifying data. Preserve framing,
payload lengths, checksums, packet order, and malformed traffic relevant to the
scenario.

`iot-boot*` is intentionally ignored by Git. IoT boot traffic may contain an
important but sensitive handshake and should remain a local-only diagnostic
fixture until it is understood and safely sanitized.

The committed synthetic checksum fixture contains one valid frame and the same
frame with a deliberately corrupted checksum. It does not contain captured
device data.

The sanitized `boot.raw.log` fixture acts as a cadence baseline. Tests currently
check its 10 Hz ECU heartbeat and approximately three-second BMS status, cell,
and temperature cycles using tolerance ranges rather than exact timestamps.
