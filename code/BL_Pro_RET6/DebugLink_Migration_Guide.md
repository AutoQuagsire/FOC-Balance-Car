# DebugLink Migration Guide

## Purpose

This document summarizes the **current implemented** DebugLink host communication stack in this project and explains how to migrate it into another firmware project.

Use this guide when you want to reuse:

- binary host communication over UART
- runtime parameter read/write
- low-rate live telemetry stream
- high-rate buffered capture via FastRing
- GUI / CLI based tuning tools

This guide describes the code that is currently wired up in:

- MCU firmware: `MyCode/System/debug_link*`, `Core/Src/main.c`
- host tools: `tools/debuglink_*`

It is different from `DebugLink_Protocol_V1.md`, which is closer to an earlier design document.

## Architecture Overview

There are two communication paths in this project:

1. `DebugLink`
   - machine-oriented binary protocol
   - transport: `USART1`
   - used by GUI, CLI, and scripts

2. `usb_debug`
   - human-readable text console
   - transport: USB CDC
   - used for logs and legacy text commands

If you only want the reusable tuning/data-link system, migrate **DebugLink first**.

## Physical Link

Current implementation:

- UART: `USART1`
- baud rate: `921600`
- mode: `8N1`
- byte order: little-endian
- RX: DMA circular reception, polled in main loop
- TX: short blocking transmit

Recommended migration rule:

- keep fast control ISR free of packet parsing and serial TX
- let ISR only update memory buffers / snapshots
- perform parsing, command dispatch, and packet TX in main loop

## Protocol Frame Format

Every frame uses this layout:

| Field | Size | Notes |
| --- | ---: | --- |
| `SOF0` | 1 | fixed `0x5A` |
| `SOF1` | 1 | fixed `0xA5` |
| `VER` | 1 | current `0x01` |
| `MSG` | 1 | message type |
| `SEQ` | 1 | request sequence |
| `LEN` | 2 | payload length, little-endian |
| `PAYLOAD` | N | up to `240` bytes |
| `CRC16` | 2 | CRC-CCITT-FALSE of `VER..PAYLOAD` |

Constants are defined in `MyCode/System/debug_link_protocol.h`.

### CRC

- polynomial: `0x1021`
- init: `0xFFFF`
- xorout: `0x0000`

Implementation exists in both:

- firmware: `DL_Protocol_CalcCrc16()`
- host: `_crc16()` / `crc16_ccitt()`

## Message Types

Current message IDs:

### Host -> Device

| Name | ID | Purpose |
| --- | ---: | --- |
| `PING_REQ` | `0x01` | link check |
| `GET_DEVICE_INFO_REQ` | `0x02` | board/fw/capability read |
| `STREAM_CONTROL_REQ` | `0x10` | start/stop low-rate status stream |
| `GET_PARAM_REQ` | `0x11` | read one parameter |
| `SET_PARAM_REQ` | `0x12` | write one parameter |
| `SAVE_PARAMS_REQ` | `0x13` | reserved, currently unsupported |
| `POWER_STAGE_REQ` | `0x15` | driver gate / power stage enable |
| `ATTITUDE_CONTROL_REQ` | `0x16` | balance control enable |
| `FAST_RING_REQ` | `0x17` | buffered high-rate capture access |

### Device -> Host

| Name | ID | Purpose |
| --- | ---: | --- |
| `ACK` | `0x80` | request accepted |
| `NACK` | `0x81` | request rejected |
| `DEVICE_INFO_RSP` | `0x82` | board/fw/capabilities |
| `PARAM_VALUE_RSP` | `0x83` | parameter readback |
| `STATUS_STREAM` | `0x90` | periodic telemetry frame |
| `EVENT` | `0x92` | reserved, currently unused |
| `FAST_RING_DATA` | `0x93` | FastRing status / snapshot / chunk data |

## ACK / NACK Rules

`ACK` and `NACK` payloads are both:

```text
<u8 req_msg> <u8 status_or_reason>
```

Current firmware behavior:

- `ACK.seq` is always `0`
- `NACK.seq` is always `0`
- matching is done by `payload[0] == original request msg`

If you want a cleaner new implementation, you can improve this by echoing request `seq`, but if you want tool compatibility, keep the current behavior.

### NACK Reasons

| Name | Value |
| --- | ---: |
| `DL_NACK_BAD_LENGTH` | `1` |
| `DL_NACK_BAD_PARAM_ID` | `2` |
| `DL_NACK_BAD_PARAM_VALUE` | `3` |
| `DL_NACK_BUSY` | `4` |
| `DL_NACK_UNSUPPORTED` | `5` |
| `DL_NACK_BAD_FRAME` | `6` |

## Firmware Receive/Transmit Mechanism

The firmware side works like this:

1. `HAL_UART_Receive_DMA()` continuously fills `s_rx_dma_buf`.
2. `DebugLink_Process()` polls the DMA write pointer.
3. New bytes are fed one-by-one into `DL_Protocol_InputByte()`.
4. When a full frame is parsed, it is copied into `s_rx_frame`.
5. `DebugLink_ProcessRxFrame()` dispatches the message in the main loop.
6. Responses use `HAL_UART_Transmit()` directly.

Important design traits:

- parsing is SOF-synchronized and byte-stream safe
- no packet work is done inside fast control ISR
- RX is robust against framing loss because parser can resync on SOF
- TX is intentionally simple and blocking

## Main Loop Integration

Current call flow in `main()`:

1. initialize system
2. call `DebugLink_Init()`
3. run application init / calibration
4. inside `while(1)`:
   - run control/application logic
   - collect telemetry snapshot
   - call `DebugLink_UpdateStatusSnapshot()`
   - call `DebugLink_Process()`

The key idea is:

- DebugLink does **not** compute control values
- DebugLink only consumes already-prepared telemetry and control APIs

## STATUS_STREAM Mechanism

`STATUS_STREAM` is a periodic push message controlled by `STREAM_CONTROL_REQ`.

Current stream state in firmware:

- `s_stream_enabled`
- `s_stream_rate_hz`
- `s_stream_period_ms`
- `s_last_stream_tick_ms`

`DebugLink_Process()` sends a stream frame when:

```text
now - s_last_stream_tick_ms >= s_stream_period_ms
```

### Stream Control Payload

`STREAM_CONTROL_REQ` payload:

```text
<u8 enable> <u8 stream_id> <u16 rate_hz> <u32 reserved>
```

Current firmware constraints:

- only `stream_id == 0` is supported
- `rate_hz` must be `1..200`

### Stream Payload Layout

Current `STATUS_STREAM` payload is 44 bytes and is built from `DebugLink_StatusSnapshot_t`.

Fields:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | `u32` | `tick_ms` |
| 4 | `i16` | `pitch_target_deg_x100` |
| 6 | `i16` | `speed_p_term_deg_x100` |
| 8 | `i16` | `speed_i_term_deg_x100` |
| 10 | `i16` | `pitch_meas_deg_x100` |
| 12 | `i16` | `pitch_rate_dps_x100` |
| 14 | `i16` | `speed_target_radps_x100` |
| 16 | `i16` | `speed_meas_radps_x100` |
| 18 | `i16` | `attitude_p_term_ma` |
| 20 | `i16` | `attitude_d_term_ma` |
| 22 | `i16` | `iq_cmd_ma` |
| 24 | `i16` | `iq_cmd_clamped_ma` |
| 26 | `i16` | `speed_output_limit_deg_x100` |
| 28 | `i16` | `attitude_output_limit_ma` |
| 30 | `i16` | `iq_l_x1000` |
| 32 | `i16` | `iq_r_x1000` |
| 34 | `i16` | `uq_l_mv` |
| 36 | `i16` | `uq_r_mv` |
| 38 | `u16` | `bus_mv` |
| 40 | `u16` | `fault_flags` |
| 42 | `i16` | `speed_raw_radps_x100` |

Parsing lives in `tools/debuglink_parser.py`.

### Status Flags

Current `fault_flags` bit usage:

| Bit | Name |
| --- | --- |
| 0 | `SPEED_FAULT_L` |
| 1 | `SPEED_FAULT_R` |
| 8 | `STACK_READY` |
| 9 | `CONTROL_IT_ENABLED` |
| 10 | `BUS_VALID` |
| 11 | `CURRENT_LOOP_ACTIVE` |
| 12 | `SPEED_LOOP_ENABLED` |
| 13 | `CURRENT_LOOP_ENABLED` |
| 14 | `POWER_STAGE_OFF` |
| 15 | `ATTITUDE_CONTROL_ON` |

## Parameter Read/Write Mechanism

The protocol supports typed parameter access.

### GET_PARAM_REQ

Payload:

```text
<u8 param_id>
```

### SET_PARAM_REQ

Payload:

```text
<u8 param_id> <u8 type> <4-byte raw value>
```

### PARAM_VALUE_RSP

Payload:

```text
<u8 param_id> <u8 type> <4-byte raw value>
```

### Supported Data Types

| Type | Value |
| --- | ---: |
| `float` | `0x01` |
| `uint8` | `0x02` |

### Current Parameter IDs

#### Current Loop

| Name | ID | Type |
| --- | ---: | --- |
| `CURRENT_KP` | `0x01` | float |
| `CURRENT_KI` | `0x02` | float |
| `CURRENT_KD` | `0x03` | float |
| `CURRENT_ILIM` | `0x04` | float |
| `CURRENT_OUTPUT_LIMIT` | `0x05` | float |
| `CURRENT_I_ERR_MIN` | `0x06` | float |
| `CURRENT_I_SEP_RATIO` | `0x07` | float |
| `CURRENT_MODE` | `0x08` | uint8 |

#### Speed Loop

| Name | ID | Type |
| --- | ---: | --- |
| `SPEED_KP` | `0x10` | float |
| `SPEED_KI` | `0x11` | float |
| `SPEED_PITCH_LIMIT` | `0x12` | float |
| `SPEED_UNWIND_GAIN` | `0x13` | float |

#### Attitude Loop

| Name | ID | Type |
| --- | ---: | --- |
| `ATTITUDE_KP` | `0x20` | float |
| `ATTITUDE_KD` | `0x21` | float |
| `ATTITUDE_IQ_LIMIT` | `0x22` | float |
| `ATTITUDE_SHUTDOWN_RAD` | `0x23` | float |

### Migration Rule for Parameters

Do not let `debug_link.c` modify internal variables directly.

Instead:

1. define parameter IDs in one place
2. expose business-layer wrappers like `App_Xxx_GetFoo()` / `App_Xxx_SetFoo()`
3. let DebugLink only do:
   - payload validation
   - type validation
   - dispatch
   - ACK/NACK

That keeps the protocol layer portable.

## Control Command Mechanism

Current control commands are intentionally tiny.

### POWER_STAGE_REQ

Payload:

```text
<u8 enable>
```

Dispatch target:

- `App_FOC_SetPowerStageEnabled(enable)`

### ATTITUDE_CONTROL_REQ

Payload:

```text
<u8 enable>
```

Dispatch target:

- `App_Attitude_SetControlEnabled(enable)`

### Migration Rule for Control Commands

Keep protocol payloads small and semantic.

Good:

- `enable/disable`
- `set param`
- `start stream`

Avoid:

- directly shipping internal structure layouts
- exposing unsafe low-level register operations

## FastRing Mechanism

FastRing is the high-rate capture path.

It avoids streaming 10kHz data directly over UART by using:

1. a live circular memory buffer
2. a snapshot copy
3. chunked host-side readback

### Why this design is useful

- control ISR stays fast
- UART bandwidth stays manageable
- host can request consistent windows after an event

### Current FastRing Operations

| Op | Value | Meaning |
| --- | ---: | --- |
| `STATUS` | `0x01` | query live ring metadata |
| `SNAPSHOT` | `0x02` | freeze current ring window |
| `READ_CHUNK` | `0x03` | read part of frozen snapshot |

### Request / Response Pattern

All use `FAST_RING_REQ` / `FAST_RING_DATA`.

#### STATUS request

Payload:

```text
<u8 op=0x01>
```

Response payload:

```text
<u8 op_echo> <u16 total_count> <u16 capacity> <u16 head> <u32 write_seq>
```

#### SNAPSHOT request

Payload:

```text
<u8 op=0x02>
```

Response payload:

```text
<u8 op_echo> <u16 total_count> <u16 capacity> <u16 reserved_head=0> <u32 write_seq>
```

#### READ_CHUNK request

Payload:

```text
<u8 op=0x03> <u32 snapshot_write_seq> <u16 start_idx> <u8 max_samples>
```

Response payload:

```text
<u8 op_echo>
<u16 total_count>
<u16 capacity>
<u32 write_seq>
<u16 start_idx>
<u8 sample_count>
<sample[0] ... sample[n-1]>
```

Current max samples per chunk:

- `8`

This is chosen because:

```text
(240 - 12) / 26 = 8
```

### Current FastRing Sample Layout

Each sample is 26 bytes:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | `i16` | `target_iq_l_ma` |
| 2 | `i16` | `iq_ref_l_ma` |
| 4 | `i16` | `filtered_iq_l_ma` |
| 6 | `i16` | `raw_iq_l_ma` |
| 8 | `i16` | `uq_final_l_mv` |
| 10 | `i16` | `target_iq_r_ma` |
| 12 | `i16` | `iq_ref_r_ma` |
| 14 | `i16` | `filtered_iq_r_ma` |
| 16 | `i16` | `raw_iq_r_ma` |
| 18 | `i16` | `uq_final_r_mv` |
| 20 | `u16` | `bus_mv` |
| 22 | `u16` | `sample_idx` |
| 24 | `u16` | `status_flags` |

## Host-Side Software Structure

The Python host implementation is split into layers:

### `tools/debuglink_transport.py`

Responsibilities:

- serial open/close
- frame build/check
- request/response retries
- background stream reader thread
- public API for GUI / CLI

### `tools/debuglink_parser.py`

Responsibilities:

- decode `STATUS_STREAM`
- decode `FAST_RING_DATA`
- convert wire values to scaled floats

### `tools/debuglink_models.py`

Responsibilities:

- typed dataclasses for stream/fastring objects

### `tools/debuglink_gui.py`

Responsibilities:

- user interface
- connect/disconnect
- start/stop stream
- CSV recording
- fast ring dump
- parameter tuning widgets

### `tools/debuglink_cli.py`

Responsibilities:

- quick diagnostics
- manual scripting
- protocol bring-up and smoke tests

## Migration Steps

## 1. Minimum MCU files to copy

Copy these first:

- `MyCode/System/debug_link_protocol.h`
- `MyCode/System/debug_link_protocol.c`
- `MyCode/System/debug_link.h`
- `MyCode/System/debug_link.c`

Then adapt:

- UART handle names
- DMA receive buffer size if needed
- application callback functions

## 2. Provide MCU-side hook functions

Your new project must provide equivalents for:

- telemetry getter/update source
- parameter getters/setters
- power-stage enable/disable
- control enable/disable
- optional FastRing APIs

For example:

- `App_Xxx_SetPowerStageEnabled()`
- `App_Xxx_SetControlEnabled()`
- `App_Xxx_GetParamFoo()`
- `App_Xxx_SetParamFoo()`

## 3. Define your telemetry snapshot

If your new project has different control variables:

1. change `DebugLink_StatusSnapshot_t`
2. change `DebugLink_SendStatusStream()`
3. change `tools/debuglink_parser.py`
4. change `tools/debuglink_models.py`
5. update GUI/CSV field names if needed

These five places must stay in sync.

## 4. Integrate into main loop

Add:

```c
DebugLink_Init();
```

after peripheral init.

Then in the main loop:

```c
DebugLink_UpdateStatusSnapshot(&st);
DebugLink_Process();
```

If your system has an RTOS, the same rule still applies:

- collect snapshot in a safe place
- run DebugLink processing in a non-ISR task/thread

## 5. Port the host tools

At minimum, copy:

- `tools/debuglink_transport.py`
- `tools/debuglink_parser.py`
- `tools/debuglink_models.py`

If you also want the ready-made tools, copy:

- `tools/debuglink_gui.py`
- `tools/debuglink_cli.py`

## 6. Verify in this order

1. `ping`
2. `get_info`
3. `driver on/off`
4. one `get_param`
5. one `set_param`
6. `stream_start`
7. `stream_stop`
8. FastRing status
9. FastRing snapshot + chunk read

Do not start with GUI first; bring up the protocol with CLI or a tiny script.

## Common Pitfalls

### 1. Host and firmware payloads drift apart

Most likely breakage point:

- firmware stream struct changed
- parser not updated

Symptom:

- values look scaled wrong
- fields appear shifted
- CSV columns become nonsense

### 2. Stream rate comments and actual limits diverge

Current example:

- firmware enforces `rate_hz <= 200`
- transport comment still says `1-500 Hz`

When migrating, trust code, not comments.

### 3. Fast ISR tries to build UART packets

Do not do this.

Keep ISR work limited to:

- updating control variables
- pushing samples into ring buffers

### 4. Direct protocol layer writes into internal state

Avoid letting `debug_link.c` touch algorithm internals directly.

Always go through application-layer wrappers.

### 5. Missing sequence/compatibility decisions

If you improve protocol behavior, decide early whether you want:

- compatibility with the current GUI/tools
- or a cleaner V2 protocol

Both are fine, but mixing them halfway causes confusion.

## Recommended Reuse Strategy

If the new project is similar to this one, reuse in this order:

1. reuse `debug_link_protocol.*` unchanged
2. reuse `debug_link.c` structure unchanged
3. adapt only:
   - parameter table
   - telemetry snapshot fields
   - control callback targets
   - FastRing sample structure if needed
4. reuse host transport/parser/models with minimal field edits
5. only then adapt GUI screens

This gives you the lowest migration risk.

## Suggested Future Improvements

If you build a cleaner second-generation version, these are good upgrades:

- echo request `seq` in `ACK/NACK`
- implement `SAVE_PARAMS_REQ`
- add protocol versioned capability bits for stream layout
- add explicit stream layout ID in `DEVICE_INFO_RSP`
- add optional event packets for faults / state transitions

## File Map

Current implementation file map:

- protocol definitions: `MyCode/System/debug_link_protocol.h`
- protocol parser/builder: `MyCode/System/debug_link_protocol.c`
- firmware transport/dispatch: `MyCode/System/debug_link.c`
- firmware public API: `MyCode/System/debug_link.h`
- firmware integration: `Core/Src/main.c`
- host parser: `tools/debuglink_parser.py`
- host models: `tools/debuglink_models.py`
- host transport: `tools/debuglink_transport.py`
- host CLI: `tools/debuglink_cli.py`
- host GUI: `tools/debuglink_gui.py`

## Final Recommendation

If your target project only needs:

- parameter tuning
- on/off control
- moderate-rate telemetry

then migrate:

- frame protocol
- request/response dispatch
- stream snapshot path

and skip FastRing first.

If your target project also has high-rate current/observer/debug signals, migrate FastRing too. It is one of the most reusable parts of this stack because it decouples fast sampling from slow serial transport.
