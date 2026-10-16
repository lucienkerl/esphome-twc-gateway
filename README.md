<p align="center">
  <img src="assets/logo.svg" alt="" width="120" height="120">
</p>

<h1 align="center">TWC Gateway</h1>

<p align="center">
  Turn a Tesla Wall Connector Gen 2 into a smart, network-controllable charger —<br>
  no Home Assistant, no Tesla cloud, no vehicle API.
</p>

<p align="center">
  <a href="https://github.com/lucienkerl/esphome-twc-gateway/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/github/license/lucienkerl/esphome-twc-gateway"></a>
  <a href="https://github.com/lucienkerl/esphome-twc-gateway/actions/workflows/build-firmware.yml"><img alt="Build status" src="https://img.shields.io/github/actions/workflow/status/lucienkerl/esphome-twc-gateway/build-firmware.yml?branch=main&label=firmware%20build"></a>
  <a href="https://esphome.io"><img alt="ESPHome external component" src="https://img.shields.io/badge/ESPHome-external%20component-2aa8c9"></a>
  <a href="https://lucienkerl.github.io/esphome-twc-gateway/"><img alt="Flash in browser" src="https://img.shields.io/badge/flash-in%20browser-5ee6d0"></a>
</p>

<p align="center">
  <a href="https://lucienkerl.github.io/esphome-twc-gateway/"><strong>Flash it now, straight from your browser →</strong></a>
</p>

This is an [ESPHome](https://esphome.io) external component — a plugin, not a fork of ESPHome itself — that adds support for the Tesla Wall Connector Gen 2 to any ESP32 project.

The Gen 2 has no IP interface of its own: no WiFi, no Modbus, no local API. The only thing it speaks is Tesla's proprietary RS-485 load-sharing protocol, designed to let up to four TWCs share power between themselves. This project puts an ESP32 on that bus, has it impersonate a "primary" (master) TWC, and exposes everything it learns — and everything it can command — as plain ESPHome entities: readable and writable over REST, MQTT, or the Home Assistant API, whichever you already have.

That makes it usable from **any** home automation system that can make an HTTP request, including systems like [Loxone](https://www.loxone.com/) that have no Tesla or Home Assistant integration at all. That is the specific goal this fork is built and tested around: get vehicle-connected/charging status, live power draw, and control over the charge current and on/off state into Loxone with no intermediate service required — just this one ESP32.

This started as a standalone Arduino application and was rewritten as an ESPHome external component so that WiFi, OTA updates, and network transport are no longer something you have to build yourself — only the load-sharing protocol itself is project-specific code.

## Contents

- [How it works](#how-it-works)
- [Features](#features)
- [Hardware](#hardware)
- [Quick start: flash from your browser](#quick-start-flash-from-your-browser)
- [Installation (ESPHome CLI)](#installation)
- [Loxone Wallbox block](#loxone-wallbox-block)
- [Configuration reference](#configuration-reference)
- [Controlling and reading over REST](#controlling-and-reading-over-rest)
- [How the current is controlled](#how-the-current-is-controlled)
- [Understanding `state`](#understanding-state)
- [Known limitations & safety notes](#known-limitations--safety-notes)
- [Development](#development)
- [Credits & related projects](#credits--related-projects)

## How it works

In its factory configuration, a Gen 2 TWC's rotary switch is set to a numbered position (its own address) and it acts as its own master. Turning the switch to position **F** puts it into slave mode: it stops deciding its own charge current and instead waits for a master TWC to tell it, once a second, how many amps it's allowed to draw. This project's ESP32 *becomes* that master — a "fake master" in the same sense TWCManager uses the term — talking to the real TWC over two wires (D+/D−) on a shared RS-485 bus.

**This protocol is reverse-engineered, not documented by Tesla.** Every value and command byte in this project was worked out by the community (see [Credits](#credits--related-projects)) by observing real traffic, not from a spec. A malformed frame can, in the worst case, put the TWC into a state that isn't recoverable without a factory reset or a call to Tesla support. Firmware differences between hardware revisions and protocol versions are common — don't assume behavior confirmed on one TWC applies unchanged to another. Set `max_current` in your configuration to what your circuit and breaker can actually deliver, not just what the TWC reports — the wallbox no longer has its own upper limit in slave mode; whatever the master sends is the only ceiling left.

## Features

- **Loxone Wallbox block integration** — pushes *vehicle connected*, *charging active*, *charging power* (kW) and the *meter reading* (kWh) straight into the block's API connector, and takes the block's *charging allowed* / *target power* (kW) back over one HTTP endpoint. One Virtual Text Input and one Virtual Output on the Loxone side; Wallbox Manager, PV surplus charging and statistics work as with a natively supported wallbox. See [Loxone Wallbox block](#loxone-wallbox-block).
- **Set Current** — a `number` entity for the current (in amps) offered to the vehicle. Values between 0 and 6 A are raised to 6 A, anything above `max_current` is capped, and the entity always shows what is *actually* offered.
- **Allow Charging** — a `switch` that offers 0 A while off. Optionally (`use_start_stop_commands`) it also sends the protocol's dedicated start/stop frames.
- **Setpoint watchdog** — if nothing has commanded the gateway for `setpoint_timeout`, it falls back to `failsafe_current` until a controller is back. With the Loxone push enabled, every successful round trip to the Miniserver counts as "alive".
- **Link supervision** — a TWC that stops answering for 26 s is dropped (as a real primary does) and every value is reset, instead of showing "charging at 16 A" from a wallbox that has lost power. A `link` binary sensor shows the state.
- **Vehicle Connected / Charging Active** — derived from the TWC's plug-state query and heartbeat, so you don't have to interpret raw state codes. Each has a numeric `0`/`1` twin for MQTT.
- **Live measurements** — charging power (kW, computed), per-phase voltage and current, actual and offered current, a meter reading with Wh resolution (`energy`), and the TWC's own lifetime kWh counter.
- **Identification** — serial number, firmware version, and (firmware permitting) the connected vehicle's VIN.
- **REST, MQTT, or Home Assistant API** — every entity is a normal ESPHome entity.

## Hardware

You need an ESP32 (any variant with a free UART) and an RS-485 transceiver wired to the TWC's D+/D− terminals, plus — unless the transceiver switches direction by itself — a GPIO controlling its driver-enable (DE/RE) pin for half-duplex switching. A few things that matter in practice, learned the hard way:

- **Level shifting.** The common red "MAX485 TTL to RS485" breakout is a 5 V part. Its receive output (RO) will drive ~5 V into an ESP32 GPIO, which is not 5 V-tolerant. Use a 3.3 V-native transceiver (MAX3485 / SP3485 / SN65HVD75) instead, or a module built for 3.3 V.
- **Galvanic isolation.** The TWC sits next to mains/EV-charging voltages. Prefer an isolated ESP32 board (e.g. Olimex ESP32-POE-**ISO**) powered independently (PoE is ideal), or an isolated transceiver, so your control electronics don't share a ground reference with the wallbox.
- **Termination.** 120 Ω across A/B is usually unnecessary for a short, direct run to a single TWC, but add it if you see corrupted frames.
- **Bench-test first.** An M5 Atom Lite + RS-485 base (or any small dev board) is a fine way to validate wiring and see real frames in the logs before committing to a permanent, isolated install.

Boards known to work: Olimex ESP32-POE / ESP32-POE-ISO with a MOD-RS485, and M5Stack Atom Lite with the Atomic RS485 Base. Any ESP32 + 3.3 V RS-485 transceiver combination should work equally well.


## Quick start: flash from your browser

**[lucienkerl.github.io/esphome-twc-gateway](https://lucienkerl.github.io/esphome-twc-gateway/)** flashes a ready-built firmware straight from Chrome or Edge — no ESPHome installation, no config file to write first. It's built from [`examples/generic-portal.yaml`](examples/generic-portal.yaml), a **single, credential-free binary**: nothing is baked in, so it works for anyone.

1. Plug your ESP32 board in over USB and open the link above — it flashes directly from the browser (desktop Chrome/Edge only; Web Serial isn't supported elsewhere).
2. With no WiFi configured yet, the device opens its own access point (`twc-setup`). Connect to it and a setup page for picking your WiFi network should open automatically.
3. Once it joins your network, open its IP in a browser. MQTT broker/username/password are plain text fields there under "Text" — fill them in and it connects immediately, no reflash or reboot needed, and it reconnects with the same settings after every future restart.

This works because ESPHome's `wifi: ap:` + `captive_portal:` handle WiFi provisioning natively. MQTT has no equivalent built-in setup flow, so the config wires three `text:` entities to `MQTTClientComponent`'s runtime setters (`set_broker_address()` etc., confirmed against ESPHome's own source) instead, applying them once at boot and again on every edit.

**Before you rely on this for actual charging:** the `max_current` baked into that build is a placeholder — see [Known limitations & safety notes](#known-limitations--safety-notes) before treating a quick-start flash as a finished installation. If you're comfortable with the ESPHome CLI and want one self-contained config with your credentials baked in instead (no post-flash setup step), use the example below.

**Loxone build.** The same page also installs [`examples/loxone-portal.yaml`](examples/loxone-portal.yaml), a credential-free build for the Loxone Wallbox block on an M5Stack Atom Lite with the Atomic RS485 Base. No Home Assistant needed: after the WiFi step above, open the device's IP and fill in the *Loxone* settings (Miniserver IP, user, password, API input `TWC_API`, block `Wb`, and the optional Vc input). They apply immediately and survive reboots. Also set **Max Current** there (default 16 A, at most 32 A) to what your circuit can carry. Firmware updates: upload a newer `*.ota.bin` on the same page. The binaries are on the [`loxone-portal-latest`](https://github.com/lucienkerl/esphome-twc-gateway/releases/tag/loxone-portal-latest) release.

## Installation

```yaml
esphome:
  name: twc
  friendly_name: Tesla Wall Connector

esp32:
  board: esp32dev # match your actual board - e.g. m5stack-atom for an
                   # M5Stack Atom Lite

# On a PoE board, prefer `ethernet:` over `wifi:` here — one less thing
# that can drop out on a device that's about to control your car charger.
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

logger:
  level: INFO

ota:
  - platform: esphome

# REST interface (entity URLs, plus /loxone and /loxone/set).
# version: 1 is deliberately minimal - v2/v3 pull in a full web UI.
web_server:
  port: 80
  version: 1

uart:
  id: twc_uart
  # GPIO19 (TX) / GPIO22 (RX) are confirmed working for an M5Stack Atom Lite
  # with the Atomic RS485 Base - M5's own pinout docs are ambiguous here, so
  # this was verified empirically. Different board, different pins.
  tx_pin: GPIO19
  rx_pin: GPIO22
  baud_rate: 9600

external_components:
  - source: github://lucienkerl/esphome-twc-gateway
    components: [twc-gateway]

twc-gateway:
  id: twc
  uart_id: twc_uart
  twc_id: 0xAB32          # this device's fake TWC ID - must not collide
                           # with a real TWC's ID on the bus
  flow_control_pin: GPIO33 # DE/RE pin of your RS-485 transceiver. Omit it
                            # for a transceiver that switches direction by
                            # itself (e.g. the Atomic RS485 Base).
  min_current: 0           # 0 = "offer nothing" is a valid setting
  max_current: 16          # hard ceiling - what your circuit can deliver
  set_current: 6           # offered after boot until something writes it

  setpoint_timeout: 120s   # optional watchdog, see below
  failsafe_current: 0

  power:
    name: "Charging Power"         # kW
  energy:
    name: "Energy"                 # kWh, Wh resolution
  actual_current:
    name: "Actual Current"
  offered_current:
    name: "Offered Current"
  vehicle_connected:
    name: "Vehicle Connected"
  charging_active:
    name: "Charging Active"
  allow_charging:
    name: "Allow Charging"
  link:
    name: "TWC Link"
  state:
    name: "State"
  serial:
    name: "Serial"
  firmware_version:
    name: "Firmware Version"
```

Every entity is optional; configure only what you use. [`examples/loxone.yaml`](examples/loxone.yaml) is a complete configuration for a wired, isolated Loxone installation.

## Loxone Wallbox block

The Loxone *Wallbox* function block wants four values from a third-party wallbox — **Vc** (vehicle connected), **Cac** (charging active), **Cp** (charging power, kW) and **Mr** (meter reading, kWh) — and produces **Ca** (charging allowed) and **Tp** (target charging power, kW). This component speaks exactly that:

```
ESP -> Miniserver   GET /dev/sps/io/TWC_API/SET(Wb;Vc;1)&SET(Wb;Cac;1)&SET(Wb;Cp;7.36)&SET(Wb;Mr;1234.567)
                    (on every change, at most every min_interval; plus a keepalive)
Miniserver -> ESP   GET http://<esp>/loxone/set?ca=1
                    GET http://<esp>/loxone/set?tp=7.4      (kW - converted to amps here)
```

### Loxone Config

1. Place a **Wallbox** block and name it `Wb` (or set `block:` to its name). Set its *Max. charging power* to `max_current × 3 × 230 V` (16 A → 11 kW) and leave *Min. charging power* at 4.16 kW (6 A × 3 × 230 V); for a single-phase car the minimum is 1.38 kW.
2. Create a **Virtual Text Input** `TWC_API` and connect it to the block's **API** connector.
3. Create a **user** (e.g. `twc`) and give it rights on `TWC_API` only.
4. Create a **Virtual Output** with address `http://<esp-ip>` (no user/password) and two **Virtual Output Commands**, HTTP method GET:
   - **Ca**, *Use as digital output* ticked: command for ON `/loxone/set?ca=1`, command for OFF `/loxone/set?ca=0`. Connect it to the block's **Ca**.
   - **Tp**, *Use as digital output* not ticked: command for ON `/loxone/set?tp=<v.2>` (kW, two decimals), command for OFF empty. Connect it to the block's **Tp**.

   Test from a browser: `http://<esp-ip>/loxone/set?ca=1&tp=4.16` answers with `offer=6` once the TWC is linked.
5. On the ESP: with the Loxone build from the [Quick start](#quick-start-flash-from-your-browser), enter the same values on its web page. In your own config:

```yaml
twc-gateway:
  # ...
  setpoint_timeout: 120s  # no Miniserver for 2 min -> failsafe_current
  failsafe_current: 0
  loxone:
    miniserver: 192.168.1.10   # IPv4 address
    username: twc
    password: !secret loxone_password
    api_input: TWC_API
    block: Wb
```

`loxone:` needs `web_server:` (or at least `web_server_base:`) for its endpoints. With `loxone: {}` and no `miniserver`, only the endpoints are served (no push) — the browser-flash firmware does exactly that.

**Known Loxone quirks.** A Virtual Text Input only reacts when its text changes, so the keepalive briefly blanks it before re-sending. Some Config versions are reported not to apply **Vc** through the API connector; if the block doesn't see the car, create a Virtual Input `TWC_Vc`, wire it to the block's Vc input and set `vc_input: TWC_Vc` — Vc is then pushed there instead. Block outputs cannot be read through the API connector, which is why Ca/Tp come back via the Virtual Output.

**kW → A.** `Tp` is converted with the *nominal* voltage (`nominal_voltage`, 230 V) — the same assumption Loxone uses for its own min/max power — and the number of phases the car is actually drawing on (`phases`, 3, until the power status shows otherwise), rounded to whole amps. Below 6 A the gateway offers 0 A.

**Pull instead of push.** `GET /loxone` returns all values on one line, for a single Virtual HTTP Input with command recognition (`vc=\v`, `cp=\v`, …):

```
vc=1;cac=1;cp=7.36;mr=1234.567;a=10.65;offer=11;twc=11.00;set=11;ca=1;st=1;ps=1;link=1;failsafe=0;ms=1
```

`a` actual current, `offer` currently offered amps, `twc` the limit the TWC reports back, `set` requested amps, `ca` charging allowed, `st` vehicle state, `ps` plug state, `link` TWC linked, `failsafe` watchdog expired, `ms` last push to the Miniserver succeeded.

### UDP instead of the API connector

Instead of (or as well as) the API connector, the gateway can send its values to a **Virtual UDP Input**: the same line as `GET /loxone`, as one datagram, on every change (at most every `udp_interval`, 1 s) and at least every 10 s.

1. Create a **Virtual UDP Input**, *Sender IP address* = the gateway's IP, *UDP receive port* e.g. `7000`.
2. Add one **Virtual UDP Input Command** per value and connect it to the Wallbox block's input of the same name:

   | Command | Command recognition | Block input |
   |---|---|---|
   | Vc | `vc=\v` | Vc |
   | Cac | `cac=\v` | Cac |
   | Cp | `cp=\v` | Cp (kW) |
   | Mr | `mr=\v` | Mr (kWh) |

   Further fields (`;a=\v` actual current - with the `;`, plain `a=` would also match inside `ca=` -, `offer=\v`, `twc=\v`, …) can be picked up the same way for display.

   Or import the ready-made template [`loxone/VIU_TWC_Gateway.xml`](loxone/VIU_TWC_Gateway.xml) (Vc, Cac, Cp, Mr plus actual/offered current, link and failsafe; port 7000): in Loxone Config, *Virtual Inputs* → *Virtual UDP Input* → *Import template*, or copy it to `Documents/Loxone/Loxone Config/Templates/VirtualIn/`. Then set the sender IP and, if you changed it, the port.
3. Disconnect `TWC_API` from the block's API connector (or delete it), so the block isn't fed twice.
4. On the ESP: enter the port as **Loxone UDP Port** on its web page and clear **Loxone API Input**. In your own config: `udp_port: 7000` under `loxone:`, without `api_input`.

Ca/Tp still come back through the Virtual Output. With UDP only, nothing proves the Miniserver alive, so every `keepalive` (30 s) the gateway sends `GET /jdev/cfg/version` without credentials; any HTTP answer feeds the setpoint watchdog and sets `ms=1`.

## Configuration reference

### `twc-gateway:` options

| Option | Default | Description |
|---|---|---|
| `id` | — | ID of this instance (also the `number` entity). |
| `uart_id` | — | The `uart:` bus wired to the RS-485 transceiver (9600 8N1). |
| `flow_control_pin` | — | GPIO driving the transceiver's DE/RE pin. Omit for auto-direction transceivers. |
| `twc_id` | `0xABCD` | The fake TWC ID this device presents. Must be unique on the bus. |
| `min_current` | `6` | Lower bound of the number entity. `0` allows "offer nothing"; `1`–`5` are rejected, the protocol has no valid value there. |
| `max_current` | `32` | Upper bound. The *only* limit left once the TWC is a secondary — set it to what your wiring can carry. The TWC's own reported maximum is applied on top. |
| `set_current` | `min_current` | Offered after boot until something writes the number entity. |
| `setpoint_timeout` | `0s` (off) | If no command (number, switch, `/loxone/set`, successful Loxone push) arrives for this long, offer `failsafe_current`. Counted from boot. |
| `failsafe_current` | `0` | Current offered while the watchdog has expired. |
| `min_on_time` | `60s` | Once a car is drawing, keep offering at least 6 A this long before going to 0 A (relay protection, as TWCManager does). |
| `min_off_time` | `30s` | After going to 0 A, wait this long before offering current again. |
| `use_start_stop_commands` | `false` | Also send the protocol's START/STOP_CHARGING frames when the offer goes above/to 0 A. Experimental. |
| `reassert_current` | `false` | Send the current limit in every heartbeat instead of until the TWC confirms it (the pre-2.0 behaviour). |
| `restore_current` | `false` | Keep the last requested current across reboots instead of starting at `set_current`. Use it with Loxone: a Virtual Output only sends on change, so an unchanged `Tp` isn't sent again after the ESP restarts. |
| `query_vin` | `true` | Ask the TWC for the connected VIN every 30 s. |
| `phases` | `3` | Phases assumed for kW ↔ A until the car shows how many it uses. |
| `nominal_voltage` | `230` | Per-phase voltage for the kW → A conversion. |
| `passive_mode` | `false` | Never transmit; only listen to a TWC that already has a real primary. |
| `loxone` | — | Loxone Wallbox block bridge, see above. Options: `miniserver`, `port` (80), `username`, `password`, `api_input`, `block` (`Wb`), `vc_input`, `min_interval` (5 s), `keepalive` (30 s), `feed_watchdog` (true). |

### Entities

| Config key | Entity type | Notes |
|---|---|---|
| *(the `twc-gateway:` block itself)* | `number` | Requested current in amps; shows the clamped value. |
| `allow_charging` | `switch` | Off = offer 0 A. Restored after reboot. |
| `power` | `sensor` | Charging power in kW (actual current × voltage of the phases in use). |
| `energy` | `sensor` | Meter reading in kWh with Wh resolution; integrated power, kept within 1 kWh of the TWC's counter and persisted across reboots. |
| `total_kwh_delivered` | `sensor` | The TWC's own lifetime counter (whole kWh). |
| `actual_current` | `sensor` | Current the car draws (A, 0.01 A resolution). |
| `current` | `sensor` | Same as `actual_current` (kept for older configs). |
| `offered_current` | `sensor` | Current offered to the car right now, after watchdog, switch and min on/off time. |
| `reported_current` | `sensor` | The current limit the TWC reports back in its heartbeat (A): what it actually took over and offers the car. Lags `offered_current` by a heartbeat or two; if it stays at 0 while `offered_current` is not, the TWC isn't accepting the limit. |
| `max_allowable_current` | `sensor` | The TWC's own hardware maximum (A). |
| `phase_1/2/3_voltage` | `sensor` | Per-phase voltage (V). |
| `phase_1/2/3_current` | `sensor` | Per-phase current (A, 0.5 A steps). |
| `state` | `sensor` | Vehicle state code, see below. |
| `vehicle_connected` / `charging_active` | `binary_sensor` | Derived, see below. |
| `vehicle_connected_numeric` / `charging_active_numeric` | `sensor` | Same as `0`/`1`. |
| `link` | `binary_sensor` | TWC linked and answering. |
| `failsafe` | `binary_sensor` | Setpoint watchdog expired, `failsafe_current` is offered. |
| `serial` / `firmware_version` / `connected_vin` | `text_sensor` | Identification. VIN is `0` when no car (or no VIN) is reported. |

## Controlling and reading over REST

With `web_server: version: 1`, every entity is at `http://<ip>/<domain>/<Entity Name>` (URL-encoded name):

| Purpose | Request |
|---|---|
| Set current setpoint | `POST /number/Set%20Current/set?value=10` |
| Stop / resume charging | `POST /switch/Allow%20Charging/turn_off` / `turn_on` |
| Read power | `GET /sensor/Charging%20Power` |
| Everything at once | `GET /loxone` (needs `loxone:`) |
| Allow + power target (kW) | `GET /loxone/set?ca=1&tp=7.4` (needs `loxone:`) |
| Current in amps | `GET /loxone/set?a=10` (needs `loxone:`) |

## How the current is controlled

The gateway acts as the TWC's primary and sends it a heartbeat once a second. Following TWCManager's findings:

- **0 A goes out as command `0x05`**, never as `0x09` with 0 A — the latter makes the TWC bounce between states 01 and 03 and click its relay every second.
- **A non-zero limit is sent as `0x09` until the TWC echoes it** in its own heartbeat (and again while it reports state 08, "starting"); after that the heartbeat carries "no change". Set `reassert_current: true` to send it every time instead.
- Between two heartbeats exactly one other frame goes out: start/stop (if enabled and changed), the plug-state query (every 3 s), the power-status query (every 5 s), serial/firmware (until known), VIN (every 30 s).
- Protocol version 1 TWCs (7-byte heartbeats) are detected and get `0x05` for every limit.
- Everything runs in ESPHome's main loop as a non-blocking state machine — no extra FreeRTOS task, no `delay()`.

## Understanding `state`

The `state` sensor is the vehicle state from the TWC's heartbeat (per TWCManager's `TWCSlave.py`). State `9` — the TWC's echo of our own `0x09` command — is filtered out, so the sensor shows the last real state:

| Code | Meaning |
|---|---|
| 0 | Ready. Car may or may not be plugged in. |
| 1 | Plugged in, charging. |
| 2 | Error (e.g. heartbeat timeout from the master). |
| 3 | Plugged in, not charging (doesn't reliably stay set). |
| 4 | Plugged in, ready to charge or charge scheduled. |
| 5 | Busy (about a second at a time). |
| 8 | Starting to charge. |
| 0x0A | Amp adjustment period complete. |

```
charging_active   = state in {1, 8}  OR  actual_current >= 1 A
vehicle_connected = plug_state != 0            (once GET_PLUG_STATE has answered)
                  = charging_active OR state in {3, 4}   (before that)
```

Some TWCs report ~0.25 A while idle, hence the 1 A threshold.

### Logs

Link changes, vehicle state changes (`Vehicle state changed for ...`) and plug changes (`Plug state changed for ...`) are logged at `INFO` under the `twc-gateway` tag, Loxone push problems under `twc-gateway.loxone`. To see raw bytes on the wire, add `debug:` to `uart:`.

For a car that is plugged in but won't charge, these `INFO` lines under `twc-gateway` tell most of the story:

- `Status ABCD: state 4 (raw 9), plug 3, actual 0.00 A, TWC limit 16.00 A | sent cmd 09 16.00 A, offered 16 A, ...` every 10 s while linked.
- `Heartbeat to ABCD now command 09, limit 16.00 A` whenever the command or limit we send changes, and `TWC ABCD now reports limit ...` whenever the limit the TWC echoes changes. If the two disagree for long, the TWC hasn't accepted our limit.
- `Sent START_CHARGING/STOP_CHARGING` (only with `use_start_stop_commands`), and any frame from the TWC that this gateway doesn't decode, with its command.
- `/loxone/set: ca=.. tp=.. a=..` for every command from Loxone (under `twc-gateway.loxone`).

Every frame on the bus is logged as hex at `DEBUG` under `twc-gateway.frames` (`RX`/`TX`). `examples/loxone-portal.yaml` turns this on; set `twc-gateway.frames: INFO` under `logger: logs:` to silence it.

## Known limitations & safety notes

- **The protocol is reverse-engineered.** Treat every command as capable of putting the TWC into an unrecoverable state until proven otherwise on your hardware/firmware. Don't invent your own frames.
- **One TWC per gateway.** A second secondary on the bus is ignored with a warning. Use one gateway (and one Loxone Wallbox block) per wall connector and let Loxone's Wallbox Manager share the power.
- **0 A stops the *offer*, not necessarily the vehicle.** Whether a car reliably resumes after a long 0 A period without being re-plugged depends on the car. Test stop *and* resume.
- **`use_start_stop_commands` is experimental.** The frames now have the length TWCManager uses (16 bytes), but TWCManager itself still controls start/stop through the car's API. Verify before relying on it.
- **The watchdog is off by default.** Enable `setpoint_timeout` if anything automates this device.
- **`max_current` is your ceiling.** In secondary mode, the TWC's rotary switch no longer limits anything.

## Development

The protocol (`twc_frames.*`), the current policy (`twc_policy.*`) and the primary state machine (`twc_master.*`) have no ESPHome dependency and are unit-tested on the host:

```sh
make -C tests
```

CI runs those tests and compiles `tests/ci/minimal.yaml` and both examples against the checked-out component.

## Credits & related projects

Credit to [WinterDragoness](https://teslamotorsclub.com/tmc/members/winterdragoness.40930/) and others on the [Tesla Motors Club forum thread](https://teslamotorsclub.com/tmc/threads/new-wall-connector-load-sharing-protocol.72830/) for reverse-engineering the protocol, and to [Craig Peacock](https://teslamotorsclub.com/tmc/members/craig-128.113283/) for the reference C implementation this project builds on. None of this would exist without that work.

- [craigpeacock/TWC](https://github.com/craigpeacock/TWC) — the C reference implementation
- [TWCManager](https://github.com/ngardiner/TWCManager) — the most actively maintained Python fake-master, MQTT/HTTP control, Tesla API integration
- [TWCManager (original)](https://github.com/dracoventions/TWCManager) — the original project, useful wiring/installation documentation
- [erwin314/twc_gen2](https://github.com/erwin314/twc_gen2) — an independent ESPHome implementation; the source for this fork's `Allow Charging` switch
- [yoziru/esphome-tesla-ble](https://github.com/yoziru/esphome-tesla-ble) — controls the vehicle directly over BLE instead of the wallbox; the more reliable option for start/stop if this component's RS-485 approach proves insufficient
