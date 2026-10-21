# Knob Firmware Rewrite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the MQTT/Home Assistant knob firmware with one codebase that builds two battery knobs:
- `volume`: Buchardt hub over LUCI; a press broadcasts play/pause to BuchardtBar.
- `lamp`: Hue room over the bridge's v1 HTTP API.

Both sleep until touched and can never wake-loop.

**Architecture:**
- All decision logic is pure, Arduino-free C++ in `lib/knoblogic/`, unit-tested on the host (PlatformIO `native` + Unity). This covers the encoder decoder, debouncer, wake masks, wake-log formatting, the line protocol, the LUCI codec, the volume controller, Hue request building and the rate coalescer.
- Thin hardware glue lives in `src/`:
  - `src/core/` holds what both knobs share: RTC state, input ISRs, fast Wi-Fi + UDP, sleep entry;
  - one `Action` implementation per variant (`src/volume/`, `src/lamp/`), chosen at compile time by `build_src_filter`.

**Tech Stack:**
- pioarduino platform 55.03.35 (Arduino-ESP32 3.3.5, ESP-IDF 5.5), board `esp32-c3-devkitm-1`.
- PlatformIO Unity tests on `native`.
- No third-party libraries.

**Spec:** `docs/superpowers/specs/2026-10-21-knob-firmware-design.md`

## Global Constraints

- `platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.35/platform-espressif32.zip` (pinned; same source as the installed platform).
- Pins: encoder A = GPIO0, B = GPIO1, button = GPIO2, all active LOW with external pull-ups. LED = GPIO8, active LOW.
- No HA, MQTT, Wi-Fi portal, web server, WebSerial, OTA. No third-party libs.
- Config is compile-time only: `include/config.h` (gitignored), template `include/config.example.h` (committed).
- Timing constants:
  - `AWAKE_MS` default 30000;
  - Wi-Fi budget 5000 ms total;
  - hub TCP connect timeout 1000 ms;
  - SET min interval 100 ms;
  - echo window 500 ms;
  - button debounce 30 ms;
  - PLAYPAUSE 3× with 30 ms gaps.
- UDP: broadcast `255.255.255.255:47777`, ASCII lines.
  - `VOLKNOB/1 PLAYPAUSE <seq>` (seq uint32 in RTC).
  - `VOLKNOB/1 LOG <knob-name> <text>`.
- LUCI framing:
  - requests little-endian (MB at bytes 3–4, length at bytes 8–9);
  - replies big-endian, same positions; bytes 6–7 ignored (CRC);
  - volume is MB 64, ASCII 0..100.
- Hue: v1 API over plain HTTP, `bri_inc` range −254..254, bridge "Zolder" `192.168.1.220`.
- Volume target is always clamped to 0..100. The hub's own max (50) is handled per Task 7's test.
- Each task commits on branch `knob-rewrite`. Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **Hub unreachable** (off, standby, wrong IP). Connect fails within 1 s and is logged; press still broadcasts PLAYPAUSE; knob sleeps after `AWAKE_MS`. Test: Task 7 step "unreachable hub".
2. **Wi-Fi trouble.** AP down or wrong SSID: give up ≤ 5 s after boot and sleep. Stale cached BSSID/channel: fall back to a normal connect and refresh the cache. Test: Task 4 steps "wrong SSID" and "stale cache".
3. **Button held through sleep entry.** Wakes once on release, and that release-wake is *not* a press. Test: `wakeIsPress` unit tests (Task 2) plus Task 4 step "held button".
4. **Fast spinning.** Many clicks inside one 100 ms window give one SET per window, and the final SET equals start + clicks × step. Test: `test_fast_spin_coalesces` (Task 6).
5. **Hue rejects the request** (bad app key, bridge error JSON). Logged, no crash; the knob keeps working and sleeps normally. Test: `parseAnyOn` / `isHueError` unit tests (Task 8) plus Task 9 step "bad key".

---

### Task 1: Project skeleton, pinned platform, host test harness

**Files:**
- Modify: `platformio.ini` (replace entirely)
- Modify: `.gitignore` (add `include/config.h`)
- Delete: `lib/WebSerial` (submodule) and `.gitmodules`
- Replace: `src/main.cpp` with a stub
- Create: `include/config.example.h`, `include/config.h` (local, untracked)
- Create: `lib/knoblogic/library.json`, `lib/knoblogic/src/knoblogic.h`
- Test: `test/test_smoke/test_main.cpp`

**Interfaces:**
- Produces:
  - envs `volume`, `lamp` (ESP32-C3) and `native` (host tests);
  - macros `KNOB_VARIANT_VOLUME` / `KNOB_VARIANT_LAMP`;
  - config macros (below).

- [ ] **Step 1: Write `platformio.ini`**

```ini
[platformio]
default_envs = volume

[esp32]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.35/platform-espressif32.zip
board = esp32-c3-devkitm-1
framework = arduino
monitor_speed = 115200
build_flags =
	-DARDUINO_USB_CDC_ON_BOOT=1
	-DARDUINO_USB_MODE=1

[env:volume]
extends = esp32
build_src_filter = +<*> -<lamp/>
build_flags = ${esp32.build_flags} -DKNOB_VARIANT_VOLUME

[env:lamp]
extends = esp32
build_src_filter = +<*> -<volume/>
build_flags = ${esp32.build_flags} -DKNOB_VARIANT_LAMP

[env:native]
platform = native
test_framework = unity
build_flags = -std=c++17 -Wall
```

- [ ] **Step 2: Remove the old code and the submodule.** Run `git rm -r lib/WebSerial .gitmodules`. Replace `src/main.cpp` with an empty `setup()`/`loop()` that includes `config.h`. Add `include/config.h` to `.gitignore`. Leave the untracked `Volume.code-workspace` alone.

- [ ] **Step 3: Create `include/config.example.h`** with these macros and comments:
  - `WIFI_SSID`, `WIFI_PASSWORD`;
  - `KNOB_STATIC_IP`, `KNOB_GATEWAY`, `KNOB_SUBNET`, `KNOB_DNS` as `"a.b.c.d"` strings; an empty `KNOB_STATIC_IP` means DHCP;
  - `KNOB_NAME`, `AWAKE_MS`, `ENCODER_REVERSE` (0/1);
  - `HUB_IP`, `VOLUME_STEP`;
  - `HUE_BRIDGE_IP`, `HUE_APP_KEY`, `HUE_GROUP_ID`, `DIM_STEP`, `HUE_MIN_INTERVAL_MS`, `HUE_TRANSITION_DS`.

  Copy it to `include/config.h` with these values:
  - `WIFI_SSID "Aether"`;
  - `WIFI_PASSWORD ""` (Jorrit fills this in);
  - `KNOB_STATIC_IP ""` (DHCP until the reservation exists);
  - `KNOB_GATEWAY "192.168.1.1"`;
  - `KNOB_NAME "volume"`;
  - `HUB_IP "192.168.1.47"`, `VOLUME_STEP 1`;
  - `HUE_BRIDGE_IP "192.168.1.220"`, `DIM_STEP 25`, `HUE_MIN_INTERVAL_MS 1000`, `HUE_TRANSITION_DS 4`.

- [ ] **Step 4: Create `lib/knoblogic`.** `library.json` = `{"name":"knoblogic","version":"1.0.0","platforms":"*"}`. `src/knoblogic.h` is an umbrella header (empty for now). Nothing in this library may include `Arduino.h`.

- [ ] **Step 5: Write `test/test_smoke/test_main.cpp`.** Unity `setUp`/`tearDown`, one `test_harness_runs` asserting `TEST_ASSERT_EQUAL(1, 1)`, and `main()` calling `UNITY_BEGIN/RUN_TEST/UNITY_END`.

- [ ] **Step 6: Verify.** Run `~/.platformio/penv/bin/pio test -e native`. Expected: `test_smoke` PASSED. Then `~/.platformio/penv/bin/pio run -e volume && ~/.platformio/penv/bin/pio run -e lamp`. Expected: both `SUCCESS`, and no platform download (the URL matches the installed one).

- [ ] **Step 7: Commit** with message `Replace firmware with two-variant skeleton and host tests`.

---

### Task 2: Pure input and sleep logic

**Files:**
- Create: `lib/knoblogic/src/detent_decoder.h/.cpp`, `debouncer.h/.cpp`, `wake_masks.h/.cpp`
- Test: `test/test_input_logic/test_main.cpp`

**Interfaces:**
- Produces:
  - `class DetentDecoder { public: int8_t update(uint8_t a, uint8_t b); };`
    - Feed the current levels (1 = HIGH). Returns +1 or −1 when a detent completes at rest state A=1 B=1, else 0.
    - +1 is the sequence (A,B) 11→01→00→10→11.
  - `class Debouncer { public: Debouncer(uint32_t debounceMs, bool initiallyPressed); int8_t update(bool pressed, uint32_t nowMs); };`
    - Returns +1 on a stable press, −1 on a stable release, else 0.
  - `struct WakeMasks { uint64_t onLow; uint64_t onHigh; };`
  - `WakeMasks computeWakeMasks(uint8_t levels, const uint8_t pins[3]);`
    - Bit i of `levels` = pin i HIGH; order A, B, BTN.
  - `bool wakeIsPress(uint8_t levelsAtSleep, uint8_t wakePins);`
    - Bit 2 = button. True only if the button woke the chip and was HIGH at sleep entry.

- [ ] **Step 1: Write the failing tests**

```cpp
// DetentDecoder: feed (a,b) pairs, sum the returns
test_cw_detent_is_plus_one:        11,01,00,10,11      -> sum == +1
test_ccw_detent_is_minus_one:      11,10,00,01,11      -> sum == -1
test_bounce_is_zero:               11,01,11,01,11      -> sum == 0
test_half_turn_back_is_zero:       11,01,00,01,11      -> sum == 0
test_missed_edge_still_counts:     11,01,10,11         -> sum == +1   // 01->10 invalid, ignored; |acc|>=2 at rest counts
test_two_detents:                  11,01,00,10,11,01,00,10,11 -> sum == +2
test_repeated_state_no_effect:     11,11,01,01,00,10,11 -> sum == +1
// Debouncer(30, false)
test_press_after_stable_30ms:  update(true,0)==0; update(false,5)==0; update(true,10)==0; update(true,39)==0; update(true,40)==+1; update(true,100)==0
test_release_reported:         after press: update(false,200)==0; update(false,230)==-1
// Debouncer(30, true): woken by a held button, no second press
test_initially_pressed_no_press: update(true,0)==0; update(true,100)==0; update(false,150)==0; update(false,180)==-1
// computeWakeMasks, pins {0,1,2}
test_all_high_wake_on_low:  levels 0b111 -> onLow 0b111, onHigh 0
test_button_low_wakes_high: levels 0b011 -> onLow 0b011, onHigh 0b100
test_all_low_wake_on_high:  levels 0b000 -> onLow 0, onHigh 0b111
// wakeIsPress
test_button_wake_from_high_is_press:    wakeIsPress(0b111, 0b100) == true
test_release_wake_is_not_press:         wakeIsPress(0b011, 0b100) == false
test_encoder_wake_is_not_press:         wakeIsPress(0b111, 0b011) == false
```

- [ ] **Step 2: Run** `pio test -e native -f test_input_logic`. Expected: FAIL (headers missing).

- [ ] **Step 3: Implement.**
  - `DetentDecoder`: keep the previous state (A<<1|B, initial 3) and an accumulator.
    - Transition delta comes from the 16-entry quadrature table: +1 for 3→1, 1→0, 0→2, 2→3; −1 for the reverse; 0 for no change or a double-bit jump.
    - On reaching state 3: emit sign(acc) if |acc| ≥ 2, then reset acc to 0.
  - `Debouncer`: track the raw state and the time of the last raw change; report when raw ≠ stable for ≥ debounceMs.
  - `computeWakeMasks` and `wakeIsPress`: per the interface.

- [ ] **Step 4: Run** `pio test -e native -f test_input_logic`. Expected: all PASS.

- [ ] **Step 5: Commit** `Add encoder decoder, debouncer and wake mask logic`.

---

### Task 3: Line protocol and wake-record formatting

**Files:**
- Create: `lib/knoblogic/src/line_protocol.h/.cpp`, `wake_record.h/.cpp`
- Test: `test/test_lines/test_main.cpp`

**Interfaces:**
- Produces:
  - `size_t formatPlayPause(char* out, size_t cap, uint32_t seq);`
  - `size_t formatLog(char* out, size_t cap, const char* knobName, const char* text);`
    - Both return the length written; return 0 with `out[0]=0` if it does not fit.
  - `struct WakeRecord { uint32_t boot; uint8_t resetReason; uint8_t wakeCause; uint8_t wakePins; uint8_t levelsAtBoot; uint8_t levelsAtSleep; bool slept; uint32_t awakeMs; uint32_t encoderEdges; uint16_t wifiMs; uint16_t firstAckMs; };`
    - `wifiMs` and `firstAckMs` are ms since boot; 0 = n/a.
  - `size_t formatWakeRecord(const WakeRecord& r, const char* resetName, const char* wakeName, char* out, size_t cap);`

- [ ] **Step 1: Write the failing tests**

```cpp
test_playpause:        formatPlayPause(buf, 64, 42) -> "VOLKNOB/1 PLAYPAUSE 42"
test_log:              formatLog(buf, 64, "volume", "boot") -> "VOLKNOB/1 LOG volume boot"
test_too_small_is_empty: formatPlayPause(buf, 10, 42) == 0 && buf[0] == 0
test_record_slept: {boot 12, wakePins 0b011, levelsAtBoot 0b111, levelsAtSleep 0b111, slept true,
                    awakeMs 30100, encoderEdges 4, wifiMs 412, firstAckMs 530}, "DEEPSLEEP", "GPIO" ->
  "#12 reset=DEEPSLEEP wake=GPIO by=A B | boot: A=1 B=1 BTN=1 | sleep: A=1 B=1 BTN=1 | awake=30.1s edges=4 wifi=412ms ack=530ms"
test_record_not_slept: {boot 3, wakePins 0b100, levelsAtBoot 0b011, slept false}, "PANIC", "NONE" ->
  "#3 reset=PANIC wake=NONE by=BTN | boot: A=1 B=1 BTN=0 | did not reach deep sleep"
test_record_no_timings: slept true, wifiMs 0, firstAckMs 0 -> ends with "edges=0 wifi=- ack=-"
```

- [ ] **Step 2: Run** `pio test -e native -f test_lines`. Expected: FAIL.

- [ ] **Step 3: Implement** with `snprintf`. "by=" lists `A`, `B`, `BTN` separated by spaces, or `-` if none.

- [ ] **Step 4: Run** `pio test -e native -f test_lines`. Expected: all PASS.

- [ ] **Step 5: Commit** `Add UDP line protocol and wake record formatting`.

---

### Task 4: Core firmware (sleep, input, wake log, fast Wi-Fi, UDP)

**Files:**
- Create: `src/core/action.h`, `src/core/rtc_state.h/.cpp`, `src/core/input.h/.cpp`, `src/core/net.h/.cpp`, `src/core/sleep.h/.cpp`, `src/core/null_action.cpp` (temporary; deleted in Task 7)
- Modify: `src/main.cpp`
- Create: `tools/udplog.py`

**Interfaces:**
- Consumes: everything from Tasks 2–3.
- Produces:

```cpp
// src/core/action.h
class Action {
 public:
  virtual ~Action() = default;
  virtual void begin(uint32_t nowMs) = 0;   // Wi-Fi is up
  virtual void onClicks(int clicks) = 0;    // signed detents, + = clockwise (after ENCODER_REVERSE)
  virtual void onPress() = 0;
  virtual void loop(uint32_t nowMs) = 0;    // pump I/O, flush coalesced commands
  virtual uint32_t firstAckMs() const = 0;  // millis() of first acknowledged command, 0 if none
  virtual void end() = 0;                   // before deep sleep
};
Action& makeAction();                       // one definition per variant
```

- `rtc_state.h`: `RTC_NOINIT_ATTR` struct validated by a magic number. It holds:
  - the wake-log ring (20 `WakeRecord`) and `bootCount`;
  - the Wi-Fi cache (`uint8_t bssid[6]; uint8_t channel; bool valid`);
  - `uint32_t playPauseSeq`.

  Functions:
  - `void rtcBegin();`
  - `WakeRecord& rtcCurrentWake();`
  - `const WakeRecord* rtcPreviousWake();`
  - `void rtcDumpToSerial();`
  - `uint32_t rtcNextPlayPauseSeq();`
  - `WifiCache& rtcWifiCache();`
- `input.h`:
  - `void inputBegin(bool wokeByPress);` attaches CHANGE ISRs on A/B that feed a `DetentDecoder`. The button is polled through `Debouncer(30, wokeByPress)`.
  - `int inputTakeClicks();` negated when `ENCODER_REVERSE` is 1.
  - `bool inputTakePress();`
  - `uint32_t inputLastActivityMs();` updated on every detent and every button press or release.
  - `uint32_t inputEdgeCount();`
  - `uint8_t inputReadLevels();`
- `net.h`:
  - `bool netConnect(uint32_t budgetMs);`
  - `void netBroadcastLine(const char* line);`
  - `void netLog(const char* fmt, ...);` formats, then writes to Serial and broadcasts `VOLKNOB/1 LOG <KNOB_NAME> ...`.
- `sleep.h`: `[[noreturn]] void sleepNow();` reads the levels, applies `computeWakeMasks`, and starts deep sleep.

- [ ] **Step 1: Implement `rtc_state`, `input`, `net` and `sleep`.** Details:
  - **Wake capture.** In `setup()` before anything else: reset reason, wake cause, `esp_sleep_get_gpio_wakeup_status()` mapped to bits A/B/BTN, and `inputReadLevels()`.
  - **Press on wake.** `wokeByPress = wakeIsPress(previous.levelsAtSleep, wakePins)`, false when there is no previous record. If true, emit one press to the action once it has begun.
  - **No recovery of the waking encoder click.** The wake status latches both A and B, so the direction is unknown; the spec allows dropping it.
  - **Wi-Fi.** `WiFi.persistent(false)`, `WiFi.mode(WIFI_STA)`, then `WiFi.config(...)` if `KNOB_STATIC_IP` is non-empty.
    - If the cache is valid: `WiFi.begin(ssid, pass, channel, bssid)` and wait up to 2500 ms.
    - On failure: `WiFi.disconnect()`, then plain `WiFi.begin(ssid, pass)` for the rest of the 5000 ms budget.
    - On success: store `WiFi.BSSID()`/`WiFi.channel()` and set `wifiMs = millis()`.
  - **UDP.** `NetworkUDP` `beginPacket(IPAddress(255,255,255,255), 47777)`. If the step-3 check shows no packets arrive, switch to a raw lwIP socket with `SO_BROADCAST` set.
  - **Sleep.** Two `esp_deep_sleep_enable_gpio_wakeup` calls, `onLow` with `ESP_GPIO_WAKEUP_GPIO_LOW` and `onHigh` with `ESP_GPIO_WAKEUP_GPIO_HIGH`; skip a call if its mask is 0. LED off (GPIO8 HIGH) before sleeping. No `esp_sleep_config_gpio_isolate`, no `gpio_deep_sleep_hold_dis`.

- [ ] **Step 2: Wire up `src/main.cpp`.** `setup`:
  1. `Serial.begin`, `rtcBegin`, capture the wake, `inputBegin`.
  2. `rtcDumpToSerial()`.
  3. `netConnect(5000)`. On failure: `netLog("wifi failed")`, record, `sleepNow()`.
  4. Broadcast the previous record (formatted), then `makeAction().begin(millis())`, then the pending wake press.

  `loop`: forward clicks and presses. When `millis() - inputLastActivityMs() > AWAKE_MS` (activity starts at 0 = boot):
  1. fill `levelsAtSleep`, `awakeMs`, `encoderEdges`, `firstAckMs`, set `slept=true`;
  2. broadcast the formatted record;
  3. `action.end()`, then `sleepNow()`.

  `null_action.cpp` implements `Action` by logging clicks and presses through `netLog`.

- [ ] **Step 3: Write `tools/udplog.py`.** Bind UDP `0.0.0.0:47777` with `SO_REUSEADDR`, print `HH:MM:SS.mmm <sender> <line>`, and append to a log file given as argv[1]. Run it with `/usr/bin/python3` (Homebrew and pio Python get `EHOSTUNREACH` because of macOS Local Network privacy).

- [ ] **Step 4: Ask Jorrit to fill in `WIFI_PASSWORD` in `include/config.h`.** Then flash `pio run -e volume -t upload` while the knob is awake (press the button first; the USB port disappears while it sleeps) and run `udplog.py`. Expected in the UDP log:
  - a `LOG volume` line with the previous record;
  - clicks and presses logged when you turn or press;
  - a record with `wifi=<ms>` at sleep entry.

  Note the `wifi=` value; it is the baseline for Task 7.

- [ ] **Step 5: Held button.** Hold the button down through the 30 s awake window until the knob sleeps, then release. Expected:
  - record N ends with `sleep: ... BTN=0`;
  - record N+1 is `by=BTN` with boot `BTN=1`, and **no** press is logged;
  - after that it sleeps again normally; no loop for 2 minutes.

  Repeat with the knob held halfway between two detents through sleep entry. Expected: it wakes once when you let go, and does not loop.

- [ ] **Step 6: Wrong SSID.** Build with `WIFI_SSID "nope"`. Expected:
  - serial (no UDP without Wi-Fi) shows `wifi failed`;
  - the knob sleeps ≤ 5.5 s after the wake (USB port disappears);
  - a later boot's dump shows that record with `awake≈5s`.

  Restore the SSID.

- [ ] **Step 7: Stale cache.** Temporarily build with `-DKNOB_TEST_STALE_WIFI_CACHE`, which overwrites the cache with channel 13 and BSSID `02:00:00:00:00:01` at boot. Expected: it connects through the fallback with `wifi=` under 5000, and the next boot uses the refreshed cache (smaller `wifi=`). Remove the flag.

- [ ] **Step 8: Commit** `Add core firmware: wake log, fast Wi-Fi, opposite-level wake, UDP log`.

---

### Task 5: LUCI codec

**Files:**
- Create: `lib/knoblogic/src/luci.h/.cpp`
- Test: `test/test_luci/test_main.cpp`

**Interfaces:**
- Produces, in namespace `luci`:
  - constants `MB_NOT_REGISTERED = 0`, `MB_REGISTER = 3`, `MB_DEREGISTER = 4`, `MB_VOLUME = 64`; `GET = 1`, `SET = 2`;
  - `size_t encode(uint8_t type, uint16_t box, const char* payload, uint8_t* out, size_t cap);` returns 0 if it does not fit;
  - `struct Message { uint16_t box; uint8_t type; uint8_t status; uint16_t length; char payload[64]; };` the payload is NUL-terminated and truncated to 63;
  - `class Decoder { public: void push(const uint8_t* data, size_t len); bool next(Message& out); };`
    - The internal buffer is 512 bytes.
    - A frame longer than what fits is consumed and dropped without losing sync.
  - `bool parseVolume(const Message& m, int* volume);` true for MB 64 with an ASCII integer payload.

- [ ] **Step 1: Write the failing tests** (bytes from protocol.md §3.2 and §12):

```cpp
test_encode_volume_set_30:  encode(SET, 64, "30") == {00 00 02 40 00 00 00 00 02 00 33 30}
test_encode_volume_get:     encode(GET, 64, "")   == {00 00 01 40 00 00 00 00 00 00}
test_encode_register:       encode(SET, 3, "192.168.1.50") == {00 00 02 03 00 00 00 00 0C 00} + "192.168.1.50"
test_decode_volume_reply:   push {00 00 01 00 40 01 ef a1 00 02 33 36} -> box 64, type 1, status 1, payload "36"; parseVolume -> 36
test_decode_register_ack:   push {00 00 02 00 03 01 27 ac 00 00} -> box 3, status 1, length 0
test_decode_split_across_pushes: same volume reply pushed as 3 + 9 bytes -> one message
test_decode_two_frames_one_push: register ack + volume reply in one push -> two messages in order
test_oversized_frame_skipped: MB 42 frame with length 0x0165 (357 bytes of 'x') then the volume reply -> next() yields only the volume reply
test_parse_volume_rejects_other_box: MB 90 "Buchardt A500" -> false
```

- [ ] **Step 2: Run** `pio test -e native -f test_luci`. Expected: FAIL.

- [ ] **Step 3: Implement.**
  - Requests use little-endian MB/length.
  - Replies are parsed big-endian: box = `b[3]<<8 | b[4]`, length = `b[8]<<8 | b[9]`.
  - Oversized frames are tracked with a `skipRemaining` counter.

- [ ] **Step 4: Run** `pio test -e native -f test_luci`. Expected: all PASS.

- [ ] **Step 5: Commit** `Add LUCI codec`.

---

### Task 6: Volume controller

**Files:**
- Create: `lib/knoblogic/src/volume_controller.h/.cpp`
- Test: `test/test_volume/test_main.cpp`

**Interfaces:**
- Produces:

```cpp
class VolumeController {
 public:
  VolumeController(int step, uint32_t minIntervalMs = 100, uint32_t echoWindowMs = 500);
  void addClicks(int clicks);
  void onHubVolume(int volume, uint32_t nowMs);  // GET reply or MB 64 notification
  int takeSet(uint32_t nowMs);                   // value to SET now, or -1
  bool known() const;
  int target() const;
};
```

Rules (from the spec):
- Clicks before the first hub value are kept and applied to it.
- Target is clamped to 0..100.
- At most one SET per `minIntervalMs`, and never a value equal to `lastSent`.
- A hub value equal to any value we sent within `echoWindowMs` is our own echo: ignore it.
- Any other hub value is adopted: `target = clamp(v + (target - lastSent))` (clicks still in flight are kept), and `lastSent = v`.

- [ ] **Step 1: Write the failing tests**

```cpp
test_clicks_before_known_applied:   addClicks(3); takeSet(0)==-1; onHubVolume(36,10); takeSet(10)==39
test_step_multiplies:               VolumeController(5); onHubVolume(20,0); addClicks(2); takeSet(0)==30
test_clamp_low_high:                onHubVolume(2,0); addClicks(-5); takeSet(0)==0; onHubVolume(99,1000) [adopted]; addClicks(5); takeSet(1000)==100
test_rate_limit_coalesces:          onHubVolume(30,0); addClicks(1); takeSet(0)==31; addClicks(1); takeSet(50)==-1; addClicks(1); takeSet(100)==33
test_never_resend_same:             ...takeSet(0)==31; takeSet(200)==-1
test_own_echo_ignored:              onHubVolume(30,0); addClicks(1); takeSet(0)==31; addClicks(1); takeSet(100)==32; onHubVolume(31,130); target()==32; takeSet(200)==-1
test_hub_clamp_adopted:             onHubVolume(50,0); addClicks(1); takeSet(0)==51; onHubVolume(50,30); target()==50; takeSet(200)==-1; addClicks(-1); takeSet(300)==49
test_clicks_in_flight_kept_on_adopt: onHubVolume(50,0); addClicks(1); takeSet(0)==51; addClicks(1); onHubVolume(50,30); target()==51; takeSet(100)==51
test_external_change_resets_last_sent: onHubVolume(30,0); addClicks(1); takeSet(0)==31; onHubVolume(31,20); onHubVolume(20,1000); addClicks(11); takeSet(1100)==31
test_fast_spin_coalesces:           onHubVolume(10,0); addClicks(1); takeSet(0)==11; 19x addClicks(1); takeSet(50)==-1; takeSet(100)==30; takeSet(200)==-1
```

- [ ] **Step 2: Run** `pio test -e native -f test_volume`. Expected: FAIL.

- [ ] **Step 3: Implement.** Keep the last 4 `(value, sentAt)` pairs for the echo check.

- [ ] **Step 4: Run** `pio test -e native -f test_volume`. Expected: all PASS.

- [ ] **Step 5: Commit** `Add volume controller`.

---

### Task 7: Volume action on hardware

**Files:**
- Create: `src/volume/volume_action.cpp`
- Delete: `src/core/null_action.cpp`
- Modify: `README.md` (rewrite: what the two knobs do, config, flashing, `udplog.py`)

**Interfaces:**
- Consumes:
  - `Action` and `netLog`/`netBroadcastLine`/`rtcNextPlayPauseSeq` (Task 4);
  - `luci::*` (Task 5);
  - `VolumeController` (Task 6);
  - `formatPlayPause` (Task 3).
- Produces: `Action& makeAction()` for env `volume`.

- [ ] **Step 1: Implement `VolumeAction`.**
  - **`begin`.** `WiFiClient` with `setNoDelay(true)`; connect to `HUB_IP:7777` with a 1000 ms timeout. Send REGISTER with `WiFi.localIP().toString()`, then GET MB 64.
    - On connect failure: `netLog("hub connect failed")`, and retry at most once every 2000 ms in `loop`.
  - **`onClicks`.** Forward to `addClicks`.
  - **`onPress`.** `formatPlayPause(rtcNextPlayPauseSeq())`, broadcast 3× with `delay(30)` in between, `netLog("playpause <seq>")`. This works even when the hub is down.
  - **`loop`.** Read all available bytes into the decoder.
    - For each message: if `parseVolume` → `onHubVolume`. If box 0 → resend REGISTER + GET.
    - Then `takeSet` → send SET.
    - The first MB 64 received after the first SET sets `firstAckMs`.
  - **`end`.** If connected: DEREGISTER, `flush()`, `stop()`.

- [ ] **Step 2: Flash, run `udplog.py`, turn slowly.** Expected:
  - the speaker volume follows the knob, one step per detent (`VOLUME_STEP 1`);
  - clockwise turns it up (if not, set `ENCODER_REVERSE 1`);
  - the records show `ack=` values.

  Compare `ack` with the 0.6 s target and `wifi` with the Task 4 baseline. Report both numbers to Jorrit; do not optimise yet (spec: decide after measuring).

- [ ] **Step 3: Unreachable hub.** Build with `HUB_IP "192.168.1.250"`. Expected:
  - `hub connect failed` in the UDP log;
  - a press still produces `VOLKNOB/1 PLAYPAUSE <seq>` (seen by `udplog.py` as 3 identical lines);
  - it sleeps after `AWAKE_MS`.

  Restore `HUB_IP`.

- [ ] **Step 4: Concurrent client.** With BuchardtBar connected to the hub, turn the knob. Expected: the knob works, and BuchardtBar keeps (or regains) its connection. If they conflict, the knob wins (spec); note the BuchardtBar behaviour for the other session.

- [ ] **Step 5: Max-volume test, only with Jorrit present.** Set the volume to 50 via the knob, then one more click (target 51). Watch the MB 64 echo in a temporary `netLog`.
  - Echo 50: the hub enforces its max. Nothing more to build.
  - Echo 51: the hub does not enforce it. Set the volume back to 50 immediately, and do Task 7b.

- [ ] **Step 6: Run** `pio test -e native` (all suites still pass) **and commit** `Add volume action: LUCI volume and play/pause broadcast`.

### Task 7b (only if the hub does not enforce its max)

**Files:**
- Modify: `volume_controller.h/.cpp` (add `void setMax(int max)`, default 100, used by the clamp)
- Modify: `test/test_volume/test_main.cpp`
- Modify: `volume_action.cpp`, `rtc_state`

- [ ] **Step 1: Failing test.** `test_set_max_clamps: setMax(50); onHubVolume(49,0); addClicks(3); takeSet(0)==50`.
- [ ] **Step 2: Implement `setMax`; run tests to PASS.**
- [ ] **Step 3: Read the max over the tunnel** (TCP 50006, request `00 03 8D 02 8B`, reply `00 04 00 02 8b <max>`). Do it on cold boot and then every 50 wakes, cache it in RTC, and call `setMax` with it.
- [ ] **Step 4: Repeat the Task 7 step 5 check.** The knob must stop at 50. Commit `Clamp volume to the hub's max`.

---

### Task 8: Hue request building and rate coalescer

**Files:**
- Create: `lib/knoblogic/src/hue.h/.cpp`, `rate_coalescer.h/.cpp`
- Test: `test/test_hue/test_main.cpp`

**Interfaces:**
- Produces, in namespace `hue`. Each function returns the length written, or 0 if it does not fit.
  - `size_t groupPath(char* out, size_t cap, const char* key, int group);` gives `/api/<key>/groups/<id>`.
  - `size_t groupActionPath(char* out, size_t cap, const char* key, int group);` gives `.../action`.
  - `size_t dimBody(char* out, size_t cap, int briInc, int transitionDs);`
  - `size_t onBody(char* out, size_t cap, bool on);`
  - `size_t buildRequest(char* out, size_t cap, const char* method, const char* host, const char* path, const char* body);`
  - `bool parseAnyOn(const char* json, bool* anyOn);`
  - `bool isHueError(const char* body);`
- Also produces `class RateCoalescer { public: explicit RateCoalescer(uint32_t intervalMs); void add(int clicks); int take(uint32_t nowMs); };`
  - The first take is immediate.
  - After that, accumulated clicks are released at most once per interval.

- [ ] **Step 1: Write the failing tests**

```cpp
test_group_paths:   groupPath(k="abc",3)=="/api/abc/groups/3"; groupActionPath=="/api/abc/groups/3/action"
test_dim_up_turns_on:  dimBody(30,4)=={"on":true,"bri_inc":30,"transitiontime":4}
test_dim_down:         dimBody(-30,4)=={"bri_inc":-30,"transitiontime":4}
test_dim_clamps:       dimBody(400,4) contains "\"bri_inc\":254"; dimBody(-400,4) contains "\"bri_inc\":-254"
test_on_body:          onBody(true)=={"on":true}; onBody(false)=={"on":false}
test_build_request:    buildRequest("PUT","192.168.1.220","/api/k/groups/3/action","{\"on\":true}") ==
  "PUT /api/k/groups/3/action HTTP/1.1\r\nHost: 192.168.1.220\r\nContent-Type: application/json\r\nContent-Length: 11\r\nConnection: keep-alive\r\n\r\n{\"on\":true}"
test_build_get_no_body: buildRequest("GET",...,"/api/k/groups/3",nullptr) has no Content-Type/Content-Length, ends "\r\n\r\n"
test_parse_any_on:     {"state":{"all_on":false,"any_on":true}} -> true,ok; "any_on": false (with space) -> false,ok
test_parse_any_on_missing: {"name":"x"} -> returns false
test_hue_error:        [{"error":{"type":1,"address":"/","description":"unauthorized user"}}] -> isHueError true, parseAnyOn false
// RateCoalescer(1000)
test_first_take_immediate: add(2); take(0)==2
test_coalesces_within_interval: add(2); take(0)==2; add(1); take(500)==0; add(3); take(1000)==4; take(2500)==0
```

- [ ] **Step 2: Run** `pio test -e native -f test_hue`. Expected: FAIL.

- [ ] **Step 3: Implement** with `snprintf`/`strstr`; no JSON library.

- [ ] **Step 4: Run** `pio test -e native -f test_hue`. Expected: all PASS.

- [ ] **Step 5: Commit** `Add Hue request building and rate coalescer`.

---

### Task 9: Lamp action on hardware

**Files:**
- Create: `src/lamp/lamp_action.cpp`, `tools/hue_pair.sh`
- Modify: `README.md` (lamp section: pairing, group id)

**Interfaces:**
- Consumes:
  - `Action`, `netLog` (Task 4);
  - `hue::*` and `RateCoalescer` (Task 8).
- Produces: `Action& makeAction()` for env `lamp`.

- [ ] **Step 1: Write `tools/hue_pair.sh <bridge-ip>`.** It runs `curl -s -X POST http://<ip>/api -d '{"devicetype":"knob#hidde"}'` and prints the `username`. Then it lists groups with `curl -s http://<ip>/api/<username>/groups`, printing id and name. **Ask Jorrit to press the link button on the Zolder bridge**, run the script against `192.168.1.220`, and put `HUE_APP_KEY` plus Hidde's room group id into `include/config.h`.

- [ ] **Step 2: Implement `LampAction`.**
  - A keep-alive `WiFiClient` to `HUE_BRIDGE_IP:80`, reconnected on demand.
  - A blocking `request()` that sends `buildRequest`, reads the status line and headers, reads the body by `Content-Length` with a 500 ms timeout, and returns the status code.
  - **`begin`.** GET the group; `parseAnyOn` sets local `anyOn`. If parsing fails: `netLog("hue error: <first 80 chars>")`.
  - **`onClicks`.** `RateCoalescer(HUE_MIN_INTERVAL_MS)` `add`.
  - **`loop`.** `n = take(now)`; if n ≠ 0, PUT `dimBody(n * DIM_STEP, HUE_TRANSITION_DS)`, and set `anyOn = true` when n > 0.
  - **`onPress`.** `anyOn = !anyOn`; PUT `onBody(anyOn)`.
  - Any `isHueError` reply is logged. `firstAckMs` is set on the first 200 reply to a PUT.

- [ ] **Step 3: Flash the knob as `lamp`** (`pio run -e lamp -t upload`, with `KNOB_NAME "lamp"`). Turn and press. Expected:
  - turning dims Hidde's room;
  - turning up from off switches it on;
  - a press toggles it.

  Record `ack=`.

- [ ] **Step 4: Rate check.** Spin fast for 5 s at `HUE_MIN_INTERVAL_MS 1000`, then repeat at 300. Watch whether the lamps lag behind or skip after you stop (that means the bridge is queueing). Keep the fastest interval that has no queueing. If even 1000 feels too slow, report that to Jorrit with a proposal for per-light commands; do not build it unasked.

- [ ] **Step 5: Bad key.** Build with `HUE_APP_KEY "invalid"`. Expected: `hue error: [{"error"...unauthorized user` in the UDP log, no crash, and the knob sleeps after `AWAKE_MS`. Restore the key.

- [ ] **Step 6: Run** `pio test -e native`, **then commit** `Add lamp action: Hue group dim and toggle`.

---

### Task 10: Soak test and handoff

**Files:**
- Modify: `README.md` (final check), `docs/superpowers/specs/2026-10-21-knob-firmware-design.md` (status line → "implemented"; note the measured latencies and the max-volume outcome)

- [ ] **Step 1: Reflash the knob as `volume`**, unplug USB, and run `udplog.py` overnight. Expected next morning: only records from real use; every `sleep:` record has the pins where they were; zero records with an unexplained `by=`.
- [ ] **Step 2: Report to Jorrit:**
  - measured `wifi`/`ack` against the 0.6 s / 0.15 s targets;
  - the max-volume outcome;
  - the Hue rate chosen;
  - the soak result;
  - whether a "warm mode" is worth discussing.
- [ ] **Step 3: Commit** `Document measured results`.
