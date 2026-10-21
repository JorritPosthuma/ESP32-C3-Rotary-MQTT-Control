# Wireless big round knob (ESP32-C3)

Battery-powered rotary knobs that sleep until touched and act directly on the device they control. Design: [Printables](https://www.printables.com/model/1079386-wireless-big-round-rotary-button).

![](https://media.printables.com/media/prints/1079386/images/8162594_0e0c6d5d-6caa-4589-a98b-1c3578e633c3_6d4e8c19-58a0-4041-bdb4-3e4dee450cc3/thumbs/inside/1600x1200/jpg/img_0792.webp)

One codebase, two knobs, chosen at build time:

| Env | Turn | Press |
|-----|------|-------|
| `volume` | Buchardt A500 volume, directly over LUCI (TCP 7777) | Play/pause on the Mac via BuchardtBar (UDP broadcast) |
| `lamp` | Dim a Philips Hue room via the bridge's local API | Toggle that room on/off |

Hardware: Olimex ESP32-C3-DevKit-Lipo; encoder A = GPIO0, B = GPIO1, push button = GPIO2 (active LOW, external pull-ups).

## Setup

1. Copy `include/config.example.h` to `include/config.h` (gitignored) and fill it in.
2. Give the hub, the Hue bridge and the knob a DHCP reservation, and set the knob's `KNOB_STATIC_IP`. A static IP skips DHCP on every wake.
3. Flash while the knob is awake (touch it first; the USB port disappears while it sleeps):
   `pio run -e volume -t upload` or `pio run -e lamp -t upload`

### Lamp knob: Hue app key and room

1. Press the link button on the bridge, then within 30 s run `tools/hue_pair.sh 192.168.1.220`.
2. Put the printed `HUE_APP_KEY` and the room's `HUE_GROUP_ID` into `include/config.h`.

If the bridge ignores its link button, reuse the key Home Assistant already has:

```
ssh hassio@homeassistant.local "jq -r '.data.entries[] | select(.domain==\"hue\") | \"\(.data.host) \(.data.api_key)\"' /config/.storage/core.config_entries"
```

`tools/hue_pair.sh` also lists the rooms with their ids; with an existing key use `curl http://<bridge>/api/<key>/groups`.

Run the host unit tests with `pio test -e native`.

## How it behaves

- **Waking.** Deep sleep between uses; any turn or press wakes it.
  - Each pin wakes on the *opposite* of its level at sleep entry, so a knob left between detents or a held button cannot cause a wake loop.
  - The click that wakes the knob is not counted.
- **Awake window.** It stays awake for `AWAKE_MS` (30 s) after the last input.
  - Wi-Fi connects in about 0.1–0.2 s using the cached access point and channel.
  - Without Wi-Fi it gives up after 5 s and sleeps.
  - An unreachable hub is retried every 2 s until the awake window ends; presses still go out.
  - After a brownout reset (weak battery) it sleeps without Wi-Fi until touched.
- **Volume variant.**
  - Steps are `VOLUME_STEP` per detent, never above `VOLUME_MAX`.
  - Updates are coalesced to at most 10 per second.
  - The knob follows the volume the hub reports, so changes from the phone or the app are picked up.

## Debugging

Every boot is recorded in an RTC wake log: reset reason, which pin woke it, pin levels at wake and at sleep, Wi-Fi and acknowledgement times. The last 20 records are printed on USB serial at boot, and each record is broadcast as `VOLKNOB/1 LOG <name> ...` on UDP port 47777.

Listen on the Mac with:

```
/usr/bin/python3 tools/udplog.py udplog.txt 192.168.1.255
```

Use Apple's Python (macOS Local Network privacy blocks the others). Bind to the subnet broadcast address while BuchardtBar holds the port.
