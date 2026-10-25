# Knob firmware rewrite: design

Date: 2026-10-21. Status: implemented, in daily use since 2026-10-24.

**Measured on the real knobs (2026-10-21 to 2026-10-24):**
- Wi-Fi after a wake: 82–177 ms (static IP, cached access point).
- Volume: the first change lands 0.47 s after touching the knob, hub handshake included. The hub acknowledges each SET in 4–40 ms.
- Lamp: the bridge acknowledges the first command 0.29–0.72 s after touching the knob. Later dims are held to one per second by the bridge's group rate limit, so the lamp misses the 0.15 s target for later clicks.
- Soak: zero spontaneous wakes on either knob in 2.7 days. The warm mode was not pursued, because the cold path is at or near the 0.6 s target.
- Open, hardware: the knobs often go to sleep between two detents (9 of 27 sleeps on the volume knob), leaving a contact closed. Current then flows through the external pull-ups while asleep, which software cannot prevent.

## Intent

Big 3D-printed rotary knobs on battery that sleep until touched and act **as fast as possible**. Two physical knobs, same hardware, one codebase:

| Knob | Where | Turn | Press |
|------|-------|------|-------|
| **volume** | Jorrit's desk, next to the monitor | Buchardt A500 volume, directly via the hub | Play/pause on the MacBook (via BuchardtBar) |
| **lamp** | Hidde's room | Dim a Philips Hue room (group of several lamps) | Toggle that room on/off |

Why a rewrite: the old firmware went knob → MQTT → Home Assistant → speaker, which added seconds of latency. It also drained its battery by waking up repeatedly. That could not be reproduced on 2026-10-21 (see Background), but it must stay impossible by construction. The old code may be thrown away entirely (Jorrit).

## Goals

- **Latency.** First action after waking from deep sleep in ≲ 0.6 s; later clicks while awake in ≲ 0.15 s. These are **targets, measured on the stock build first** *(my call)*.
  - If the cold path misses, the next step is boot/connect optimisation.
  - Possibly also a "warm" mode: pioarduino `custom_sdkconfig` with `CONFIG_PM_ENABLE` for automatic light sleep while staying connected for a few minutes after use.
  - That is decided with Jorrit after measuring, not up front.
- **Battery.** Wakes only when touched. No wake loops. Never stays awake indefinitely because of network trouble.
- **Diagnosable.** Every wake is logged in a way that survives deep sleep and can be read over USB serial or over the network, without a cable.

## Non-goals

- Home Assistant / MQTT (dropped, Jorrit).
- Runtime configuration (no Wi-Fi portal, no web UI). All config is compile-time; changing it means reflashing (Jorrit).
- Discovery of the hub or bridges. Fixed IPs from config.
- OTA updates, battery-level reporting, long-press / double-press actions *(my call; easy to add later)*.

## Anti-goals (failure even if it "works")

- A knob that wakes by itself, or gets stuck in a wake loop.
- Volume jumping because the knob counted from a stale value.
- Firmware that sits awake with the radio on when Wi-Fi, the hub or the bridge is unreachable.

## Hardware (verified on the real device, 2026-10-21)

- Olimex ESP32-C3-DevKit-Lipo Rev B, LiPo battery. [Schematic](https://www.electrokit.com/upload/product/41020/41020799/ESP32-C3-DevKit-Lipo_Rev_B-2.pdf).
- Encoder A = GPIO0, B = GPIO1, push button = GPIO2, all active LOW. All three have **external pull-ups** (measured with a pull probe). GPIO2 also has a 33 kΩ board pull-up (strapping pin).
- Encoder: one detent = one **full** quadrature cycle; at rest A=1 B=1 (seen on every one of 7 wakes).
- LED on GPIO8, active LOW (strapping pin). GPIO3/4 are battery/power sense behind open jumpers; unused.
- Framework: pioarduino platform 55.03.35 = Arduino-ESP32 3.3.5 / ESP-IDF 5.5. `platformio.ini` currently says plain `espressif32`, which only resolves to this because of what is installed locally; **pin it explicitly.**
- The Arduino sdkconfig has no `CONFIG_PM_ENABLE`, so automatic light sleep with Wi-Fi kept connected is not available. Manual light sleep and deep sleep are.

## Architecture

One PlatformIO project, two environments, `volume` and `lamp`. A shared **core** plus one **action** module per variant, selected at compile time. The core knows nothing about speakers or lamps. It hands the action module events: "connected", "N clicks (signed)", "pressed", "going to sleep".

### Core

1. **Boot / wake.** Record the wake in the RTC wake log (below). Attach encoder and button interrupts **before** starting Wi-Fi, so clicks made during the connect are counted and applied once the action module is ready.
   - **The waking touch** *(my call)*:
     - A wake by the button counts as a press.
     - The encoder click that wakes the chip usually completes during boot, before interrupts are attached; the log shows `edges=0` after a one-click wake. Recover its direction from the wake pins where that is unambiguous, otherwise drop it. In practice the first click mostly just wakes the knob.
2. **Fast Wi-Fi.** Static IP from config. Cache BSSID and channel in RTC memory and connect with them. If that fails, fall back to one normal connect and refresh the cache. Hard cap on total connect time *(my call: 5 s)*, then log and go to sleep.
3. **Input.**
   - Encoder decoding counts one step per detent, at rest state 11.
   - Button debounced *(my call: 30 ms)*. Short press only.
   - Any input resets the awake timer. This includes the button; the old code forgot that.
4. **Awake window.** Stay connected for `AWAKE_MS` after the last input *(default 30 s)*, then hand the action module "going to sleep" and sleep.
5. **Sleep entry: no wake loops by construction.** Read A, B and the button right before sleeping. Configure each pin to wake on the **opposite** of its current level: pins that are HIGH wake on LOW, pins that are LOW wake on HIGH. This uses two `esp_deep_sleep_enable_gpio_wakeup` calls, one per level. A knob resting between detents, or a button held down by the enclosure, then wakes on change instead of looping.
   - Note: IDF 5.5 enables internal pull-up/pull-down resistors on wake pins automatically (`CONFIG_ESP_SLEEP_GPIO_ENABLE_INTERNAL_RESISTORS=1`). For a HIGH-trigger pin that means an internal pull-down working against the external pull-up. That is harmless for detection, but costs a little current while that contact is closed.
   - Drop the old `esp_sleep_config_gpio_isolate()` / `gpio_deep_sleep_hold_dis()` calls unless measurement shows they help.
6. **Wake log.**
   - `RTC_NOINIT_ATTR` ring buffer of the last 20 boots, validated by a magic number, so it survives deep sleep and soft resets.
   - Each record holds: boot number, reset reason, wake cause, which pins woke it, pin levels at boot and at sleep entry, awake time, encoder edge count, and timings (wake → Wi-Fi up → first action acknowledged).
   - Printed on USB serial at every boot. The current record is also broadcast as a UDP log line (see Network messages).
   - The current diagnostic firmware already has this; port it.

### Variant `volume`: Buchardt A500 via LUCI

Protocol reference: `/Users/jorrit/Development/Projects/Buchardt/BuchardtBar/docs/protocol.md` (§3 LUCI framing, §4 volume, §12 bytes verified on the real hub). Python reference: `.../BuchardtBar/tools/probe.py`. Swift codec: `.../Sources/BuchardtKit/Protocol/LuciCodec.swift`.

- **Connect.** TCP to the hub (`192.168.1.47:7777`, plain TCP, no auth).
  - REGISTER: MB 3 SET, payload = own IPv4 as ASCII.
  - Then MB 64 GET returns the current volume, ASCII 0–100.
  - Requests use little-endian headers; replies are big-endian (see protocol.md).
- **Turning.**
  - target = current + clicks × `VOLUME_STEP`, clamped to 0..100.
  - Clicks made before the GET answer are added once it arrives.
  - SET MB 64 with an ASCII decimal payload, at most one every 100 ms, only the latest value, never a value equal to the last one sent.
- **Stay in sync.**
  - The hub pushes MB 64 notifications after changes. When the hub reports a value different from the target once a SET has settled (for example clamped by the hub's max volume, or changed by another client), adopt the reported value as the new base. Turning past a limit then never accumulates.
  - Adopting a reported value also resets "last sent" *(my call)*. Otherwise turning back to the previous value would be skipped while the hub sits elsewhere.
- **Hub maximum volume** (currently 50 in the hub settings). Jorrit: use the hub's limit, no separate setting. It is unverified whether the hub enforces its max on LUCI SETs. **Test once with Jorrit present:** SET max+1 and watch the echo.
  - If enforced: rely on the echo (above).
  - If not: read the max via the MCU tunnel (TCP 50006, command `02 8B`). Do it rarely (cold boot, then every N wakes), cache it in RTC, and clamp to it. Reading it costs time, so not on every wake (Jorrit).
- **Other LUCI clients.** BuchardtBar on the Mac and the phone app may be connected too.
  - Expected to be fine: Libre modules support multiple controllers, and the conflict seen so far was two connections from the *same* IP.
  - Confirm with an early test on the real hub with the knob and BuchardtBar connected at the same time.
  - If they do conflict, **the knob wins** (Jorrit). It is connected only while awake, and the other clients re-register afterwards; BuchardtBar's own fallback is to connect only while its panel is open. No routing of volume through the Mac.
- **Before sleep.** DEREGISTER (MB 4 SET, own IP), then close.
- **Press.** Play/pause on the Mac via BuchardtBar (see Network messages). This works independently of the hub connection.

### Variant `lamp`: Philips Hue room via the bridge's local v1 API

- Plain HTTP REST, v1 API. Verified: both bridges answer `http://<ip>/api/0/config` with API 1.78. No TLS on the ESP.
- There are **two bridges**: "Zolder" `192.168.1.220` and "Beneden" `192.168.1.164`. **Hidde's room is on Zolder** (Jorrit).
  - Find the group id with `GET /api/<key>/groups` after creating the app key.
  - Config holds bridge IP and group id.
- **One-time setup:** press the bridge's link button and create an app key (`POST /api {"devicetype":"knob#hidde"}`). Provide a tiny documented curl/Python step for this; the key goes into `config.h`.
- **Turning:**
  - `PUT /api/<key>/groups/<id>/action` with `bri_inc` = clicks × `DIM_STEP` (bri scale 1–254).
  - When turning up, include `"on": true`, so turning up from off switches the room on.
  - Turning down never switches off; only the press does. Relative commands mean no read is needed first.
- **Press:** toggle. Prefetch the group state (`GET /api/<key>/groups/<id>` → `state.any_on`) right after connecting and track it locally while awake. Press sends `{"on": !any_on}`.
- **Rate limits.** Hue guidance: about 1 command/s per group, about 10/s total for individual lights. The first command goes out immediately; later clicks are coalesced. **Builder's call, by measurement on the real bridge:** group commands at ≤ 1/s with a matching `transitiontime`, or per-light `bri_inc` commands within the 10/s budget. Criterion: feels smooth, no queued or dropped commands.
- HTTP keep-alive (reusing one TCP connection) if the bridge supports it: builder's call.

### Network messages (agreed with the BuchardtBar session)

- UDP broadcast to the subnet broadcast address (`192.168.1.255`) on port `47777`, one ASCII line, optional trailing `\n`. No ack (fire-and-forget).
  - Changed during the build from `255.255.255.255`. On macOS, BuchardtBar's wildcard socket holds the port exclusively, and a debug listener can only share it bound to the subnet broadcast address, which never sees limited broadcasts. BuchardtBar receives both.
- `VOLKNOB/1 PLAYPAUSE <seq>`. `seq` is a uint32 kept in RTC memory, incremented per press. Each message is sent 3× with 30 ms in between. BuchardtBar de-duplicates on (sender IP, seq) within 2 s and ignores lines it does not know.
- `VOLKNOB/1 LOG <knob-name> <text>` for the wake log. BuchardtBar ignores it; a Mac-side logger listens for debugging.
- The lamp knob sends only LOG lines.
- Mac side (BuchardtBar, other project) synthesizes the ⏯ media key. That needs the macOS **Accessibility** permission. The BuchardtBar session confirms with Jorrit whether and when it builds this.

### Configuration

- `include/config.h`: gitignored, per knob. Commit a `config.example.h` with:
  - Wi-Fi SSID and password;
  - static IP, gateway `192.168.1.1`, subnet, DNS;
  - `KNOB_NAME`, `AWAKE_MS`;
  - volume: `HUB_IP`, `VOLUME_STEP` (default 1, Jorrit wants it adjustable);
  - lamp: `HUE_BRIDGE_IP` (`192.168.1.220`), `HUE_APP_KEY`, `HUE_GROUP_ID`, `DIM_STEP`.
- Jorrit's router: DHCP reservations for the hub and the bridges, and a fixed address for each knob.

### Dependencies

- Remove: WiFiManager, PubSubClient, AsyncTCP, ESPAsyncWebServer, the WebSerial submodule (`lib/WebSerial`), Bounce2.
- Keep or replace RotaryEncoder and ArduinoJson (for parsing Hue's GET reply): builder's call. Fewer libraries means faster boot and smaller firmware.

## Testing

- **Host unit tests** (PlatformIO `native` env + Unity) for the pure logic:
  - LUCI frame encode/decode;
  - volume target/coalescing/sync rules;
  - wake-level computation;
  - Hue request building;
  - wake-log record formatting.
- **On hardware:**
  1. Wake-loop test: go to sleep with the button held, and with the knob between detents. It must wake once on release or move, not loop.
  2. Latency: timestamps in the wake log (wake → Wi-Fi → first ack), checked against the goals.
  3. Hub max-volume enforcement test (with Jorrit present).
  4. Soak: on battery overnight with the UDP log, zero spontaneous wakes.
- Existing debug tools from 2026-10-21 (serial logger, MQTT logger) live in the session scratchpad. Replace them with a small UDP listener in `tools/`.

## Background: the wake investigation (2026-10-21)

- The old code woke on LOW for A, B and the button. ESP32-C3 deep-sleep GPIO wake is **level**-triggered, so any pin that is LOW at sleep entry causes an immediate wake. That gives a loop of about 30 s, consistent with "every minute".
- Diagnostic firmware logged 7 wakes over about 2.5 hours across USB, battery without knob, and battery with the knob mounted. **Zero spontaneous wakes**; every wake was a real touch; all pins were HIGH at every sleep entry.
- Root cause of the original drain is therefore **not proven**. Remaining candidates:
  - a pin LOW at sleep entry in some physical state;
  - behaviour of the older framework (the project was last built against IDF 5.3.2).
- The rewrite removes the first candidate by construction (opposite-level wake) and keeps the wake log to catch anything else.

## Left to the builder

Module layout and naming; encoder decoding approach; Hue rate strategy (by measurement); HTTP keep-alive; exact timing constants marked *my call*; the UDP listener tool.
