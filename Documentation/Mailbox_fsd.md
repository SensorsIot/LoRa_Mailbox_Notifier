# Mailbox Notifier — Functional Specification

This repository collects successive iterations of a LoRa-based mailbox notifier built around the Ebyte E32 (SX1278, 433 MHz) module. The shared goal across every iteration is the same: detect mail being deposited or removed in a remote mailbox and surface that state in Home Assistant.

The iterations differ in target MCU, wire protocol, gateway transport (MQTT vs. ESPHome native API), and robustness of the E32 programming sequence. Each chapter below is self-contained and describes one iteration as found in its source folder. Chapters are ordered by initial commit date, newest first.

Source repository: <https://github.com/SensorsIot/LoRa_Mailbox_Notifier>

---

# ESPHomeGateway

*Folder: `ESPHomeGateway/` — initial commit 2025-12-26*

ESPHome-based gateway running on a Wemos D1 mini (ESP8266). No MQTT broker required: the device speaks the ESPHome native API directly to Home Assistant. This is the most recent gateway iteration.

## Files

- `briefkastengw.yaml` — top-level ESPHome configuration.
- `esphome/components/uart_mailbox_notifier/__init__.py` — Python config schema registering the external component (text_sensor + uart device).
- `esphome/components/uart_mailbox_notifier/text_sensor.py` — codegen wiring.
- `esphome/components/uart_mailbox_notifier/uart_Mailbox_Notifier.h` — C++ implementation that reads the E32 UART byte-by-byte and publishes `"true"` for `0x55` (full) / `"false"` for `0xAA` (empty).

## Wire protocol

Bytes received from the sensor on UART (9600 8N1):

| Byte   | Meaning |
| ------ | ------- |
| `0x55` | Mailbox full  |
| `0xAA` | Mailbox empty |

Gateway → sensor:

| Byte   | Meaning |
| ------ | ------- |
| `0x25` | ACK     |

## Pin map (D1 mini)

| Function | Pin |
| -------- | --- |
| UART TX  | D7  |
| UART RX  | D6  |
| `M0`     | D1  |
| `M1`     | D2  |

## Behaviour

- **One-shot E32 programming on boot.** `on_boot` raises `M0`/`M1` (parameter mode), writes `C0 00 01 1A 17 47` (30 dBm profile), waits 500 ms, then drops `M0`/`M1` LOW for transparent mode.
- **Custom external component.** Replaces the older "custom lambda + include uart_Mailbox_Notifier.h" pattern with a proper external component referenced via:
  ```yaml
  external_components:
    - source:
        type: local
        path: ./esphome/components

  text_sensor:
    - platform: uart_mailbox_notifier
      id: uart_readline
      uart_id: uart_bus
  ```
- **State derivation.** A `template` `binary_sensor` named "Mailbox State" reads the text sensor and resolves to `ON`/`OFF`.
- **ACK on every state change.** The `binary_sensor`'s `on_state` automation writes `0x25` over UART.

---

# LetterBoxE32_ATTINY_V2 (production sensor)

*Folder: `LetterBoxE32_ATTINY_V2/` — initial commit 2025-03-25*

Final ATtiny1614-based sensor firmware. Wakes from `SLEEP_MODE_PWR_DOWN` on either switch interrupt, transmits a single status byte, waits for ACK, retransmits up to 5×, then sleeps again.

## Wire protocol

| Byte   | Direction       | Meaning |
| ------ | --------------- | ------- |
| `0x55` | sensor → gateway | `FULL`  — mail deposited |
| `0xAA` | sensor → gateway | `EMPTY` — mailbox emptied |
| `0x25` | gateway → sensor | `ACKNOWLEDGE` |

## E32 module configuration

Verified-before-programmed. Two profiles, autodetected by reading byte 3 of the C3 response:

| Variant | 6-byte parameter set     |
| ------- | ------------------------ |
| 20 dBm  | `C0 00 01 1A 17 44`      |
| 30 dBm (response byte 3 = `0x1E`) | `C0 00 01 1A 17 47` |

Programming sequence (`M0=M1=HIGH` parameter mode):

1. `C3 C3 C3` → read module type, decide expected profile.
2. `C1 C1 C1` → read back the stored 6-byte parameter set.
3. Write the expected profile **only if** the read-back differs.

This avoids writing the parameter EEPROM on every wake-up.

## Pin map (ATtiny1614)

| Function          | Arduino pin | ATtiny pin |
| ----------------- | ----------- | ---------- |
| `SWITCH_OPENING`  | 10          | PA3        |
| `SWITCH_DOOR`     | 0           | PA4        |
| `M0`              | 6           | PB0        |
| `M1`              | 7           | PB1        |
| `AUX`             | 3           | PA7        |
| `LED` (debug)     | 1           | PA5        |
| `Serial` (to E32) | TX/RX       | PB2/PB3    |

## Toolchain

- Boards Manager URL: `http://drazzy.com/package_drazzy.com_index.json`
- Board: **megaTinyCore → ATtiny1614** (UPDI)
- Programmer: **SerialUPDI** (USB-UART adapter, 4.7 kΩ on the UPDI line)
- Clock: 1 MHz internal, BOD disabled to minimise sleep current

## Behaviour

- **State machine:** `boxfilled → waitackfull → boxfull → boxemptied → waitackempty → boxempty`, each retry-bounded (5 retransmits, 1 s spacing).
- **Power-down hygiene.** All unused pins set `INPUT_PULLUP` in `setup()`; `M0`/`M1` driven HIGH (E32 sleep) before MCU sleep.
- **Wake sources.** Pin-change interrupts on `SWITCH_OPENING` and `SWITCH_DOOR` (`LOW` level).

## PCB

Custom PCB with ATtiny1614, E32 module, two switch inputs and a battery connector. OSHWLab project: *Mailbox Notifier*.

---

# LetterBoxGatewayV2 (production gateway, MQTT)

*Folder: `LetterBoxGatewayV2/` — initial commit 2025-03-25*

ESP32 + Arduino + PubSubClient gateway with Home Assistant MQTT autodiscovery. Recommended Arduino-side gateway.

## Wire protocol

| Byte   | Direction        | Meaning |
| ------ | ---------------- | ------- |
| `0x55` | sensor → gateway | `ARRIVED` — mail deposited |
| `0xAA` | sensor → gateway | `EMPTY`   — mailbox emptied |
| `0x25` | gateway → sensor | `ACKNOWLEDGE` |

## E32 module configuration

Same verify-before-program logic as the V2 sensor: send `C3 C3 C3` to identify module type (response byte 3 = `0x1E` → 30 dBm), then `C1 C1 C1` to read back the parameter set, write only on mismatch.

| Variant | 6-byte parameter set |
| ------- | -------------------- |
| 20 dBm  | `C0 00 01 1A 17 44`  |
| 30 dBm  | `C0 00 01 1A 17 47`  |

## Pin map / wiring (ESP32 ↔ E32)

| Function | ESP32 GPIO | Wire colour | E32 module |
| -------- | ---------- | ----------- | ---------- |
| `M0`     | 21         | Blue        | `M0`       |
| `M1`     | 19         | Grey        | `M1`       |
| `AUX`    | 4          | Orange      | `AUX`      |
| UART TX  | 17         | Yellow      | `RX`       |
| UART RX  | 16         | Green       | `TX`       |
| 3.3 V    | —          | Red         | `VCC`      |
| GND      | —          | Black       | `GND`      |

## Secrets

`secrets.h`: `ssid`, `password`, `mqtt_server`.

## Behaviour

- **Wi-Fi + MQTT.** Connects with credentials from `secrets.h`, then maintains a persistent connection to `mqtt_server:1883`.
- **Home Assistant autodiscovery.** Derives a stable `unique_id` from the last 4 hex chars of the MAC, publishes a retained config message:
  - Topic: `homeassistant/binary_sensor/<uniqueID>/config`
  - Payload: `device_class: occupancy`, `name: Mailbox`, manufacturer/model/sw/hw fields, `state_topic: .../state`.
- **State publishing.** On `0x55` publishes `ON` (retained) and ACKs immediately; on `0xAA` publishes `OFF` (retained) and ACKs.
- **ACK timing.** Sent inline from the receive path (one ACK per received status byte), unlike the older HA-gateway revision.

---

# LetterBoxE32_Test_ESP32

*Folder: `LetterBoxE32_Test_ESP32/` — initial commit 2025-03-25*

Bare-ESP32 bring-up transmitter. Used to exercise either gateway implementation without needing the real mailbox sensor in front of you. No Wi-Fi, no MQTT, no ACK handling.

## Wire protocol

| Byte   | Meaning |
| ------ | ------- |
| `0x55` | `FULL`  |
| `0xAA` | `EMPTY` |

## E32 module configuration

Programs the 20 dBm profile unconditionally on boot:

`C0 00 01 1A 17 44`

(The 30 dBm variant `C0 00 01 1A 17 47` is left commented in the source for users with the 30 dBm module.)

## Pin map (ESP32)

| Function | GPIO |
| -------- | ---- |
| UART TX  | 17   |
| UART RX  | 16   |
| `M0`     | 7    |
| `M1`     | 6    |
| `AUX`    | 3    |

## Behaviour

- Configures `M0`/`M1` HIGH, writes the 6-byte profile, waits for `AUX` HIGH, drops `M0`/`M1` LOW.
- Loops forever: `FULL → delay 1 s → EMPTY → delay 1 s → repeat`.

---

# LetterBoxE32_ATTINY_Battery

*Folder: `LetterBoxE32_ATTINY_Battery/` — initial commit 2025-03-06*

ATtiny1614-based sensor with the same state machine as the V2 sensor, plus battery-voltage telemetry appended to every transmission. Written before the verify-before-program rewrite, so it programs the E32 unconditionally on boot.

## Wire protocol

Each status transmission is a 3-byte frame:

| Offset | Byte         | Meaning |
| ------ | ------------ | ------- |
| 0      | `0x55`/`0xAA`| Status: `FULL` or `EMPTY` |
| 1      | `Vcc[15:8]`  | Battery voltage in mV, high byte |
| 2      | `Vcc[7:0]`   | Battery voltage in mV, low byte  |

Reverse direction:

| Byte   | Meaning |
| ------ | ------- |
| `0x25` | `ACKNOWLEDGE` |

> The current gateway implementations only consume the leading status byte; the trailing two bytes are reserved for a future battery sensor on the gateway side.

## E32 module configuration

Programs the 20 dBm profile (`C0 00 01 1A 17 44`) unconditionally on boot. No verify, no module-type autodetection.

## Battery measurement

`readBatteryVoltage()` measures Vcc against the internal 1.1 V bandgap reference:

```cpp
ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
delay(2);
ADCSRA |= _BV(ADSC);
while (ADCSRA & _BV(ADSC)) ;
uint16_t result = (ADCH << 8) | ADCL;
uint32_t vcc = 1125300UL / result;   // mV
```

## Pin map (ATtiny1614)

| Function          | Arduino pin | ATtiny pin |
| ----------------- | ----------- | ---------- |
| `SWITCH_OPENING`  | 10          | PA3        |
| `SWITCH_DOOR`     | 0           | PA4        |
| `M0`              | 7           | PB0        |
| `M1`              | 6           | PB1        |
| `AUX`             | 3           | PA7        |
| `LED` (debug)     | 1           | PA5        |

> Note: `M0`/`M1` are swapped relative to the V2 sensor — this variant follows the V1 PCB pinning.

## Behaviour

- **State machine** identical to V2: `boxfilled → waitackfull → boxfull → boxemptied → waitackempty → boxempty`, retry-bounded at 5 retransmits with 1 s spacing.
- **Sleep.** `SLEEP_MODE_PWR_DOWN` between events, all unused pins `INPUT_PULLUP`, `M0`/`M1` driven HIGH (E32 sleep) before MCU sleep.
- **Wake sources.** Pin-change interrupts on `SWITCH_OPENING` and `SWITCH_DOOR` (`LOW` level).

---

# LetterBoxE32_ATTINY_Testprogram

*Folder: `LetterBoxE32_ATTINY_Testprogram/` — initial commit 2025-03-06*

ATtiny1614-based bring-up transmitter. Used during range and antenna tuning, paired with a test gateway. No sleep, no acknowledgement handling.

## Wire protocol

| Byte   | Meaning |
| ------ | ------- |
| `0x55` | `FULL`  |
| `0xAA` | `EMPTY` |

## E32 module configuration

- Wake-up sequence `C4 C4 C4` to recover modules left in an unknown state.
- Programs the 20 dBm profile (`C0 00 01 1A 17 44`) unconditionally.
- 30 dBm profile (`C0 00 01 1A 17 47`) provided as a commented alternative.

## Pin map (ATtiny1614)

| Function | Arduino pin | ATtiny pin |
| -------- | ----------- | ---------- |
| `M0`     | 6           | PB0        |
| `M1`     | 7           | PB1        |
| `AUX`    | 3           | PA7        |

## Behaviour

Loops forever: `FULL → delay 5 s → EMPTY → delay 2 s → repeat`. Waits for `AUX` HIGH between writes.

---

# LetterboxGatewayHA (legacy gateway, MQTT)

*Folder: `LetterboxGatewayHA/` — initial commit 2024-10-24*

The earliest gateway revision in the repo. ESP32 + Arduino + PubSubClient with Home Assistant MQTT autodiscovery — same overall surface as `LetterBoxGatewayV2`, but predates the verify-before-program rewrite.

## Wire protocol

| Byte   | Direction        | Meaning |
| ------ | ---------------- | ------- |
| `0x55` | sensor → gateway | `FULL`  |
| `0xAA` | sensor → gateway | `EMPTY` |
| `0x25` | gateway → sensor | `ACKNOWLEDGE` |

## E32 module configuration

Programs `C0 00 01 1A 17 47` (30 dBm) unconditionally on every boot. No autodetection. Users with a 20 dBm module must edit the source.

## Pin map / wiring (ESP32 ↔ E32)

| Function | ESP32 GPIO | Wire colour | E32 module |
| -------- | ---------- | ----------- | ---------- |
| `M0`     | 21         | Blue        | `M0`       |
| `M1`     | 19         | Grey        | `M1`       |
| `AUX`    | 4          | Orange      | `AUX`      |
| UART TX  | 17         | Yellow      | `RX`       |
| UART RX  | 16         | Green       | `TX`       |
| 3.3 V    | —          | Red         | `VCC`      |
| GND      | —          | Black       | `GND`      |

## Secrets

`secrets.h` (in the user's Arduino library path, named `Secrets1.h` in this folder for reference): `ssid`, `password`, `mqtt_server`.

## Behaviour

- **Wi-Fi + MQTT.** Connects with credentials from `secrets.h`, persistent connection to `mqtt_server:1883`.
- **Home Assistant autodiscovery.** Same retained discovery message as V2: `device_class: occupancy`, MAC-derived `unique_id`, manufacturer/model/sw/hw fields.
- **State publishing.** On `0x55` publishes `ON` (retained); on `0xAA` publishes `OFF` (retained).
- **ACK timing.** Differs from V2: instead of replying inline from the receive path, sets a `transmissionSuccess` flag and writes `0x25` from the main loop on the next iteration.

Kept for users who don't need module-type autodetection. New deployments should use `LetterBoxGatewayV2`.
