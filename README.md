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
   `pio run -e volume -t upload`

Run the host unit tests with `pio test -e native`.

## How it behaves

- **Waking.** Deep sleep between uses; any turn or press wakes it.
  - Each pin wakes on the *opposite* of its level at sleep entry, so a knob left between detents or a held button cannot cause a wake loop.
  - The click that wakes the knob is not counted.
- **Awake window.** It stays awake for `AWAKE_MS` (30 s) after the last input.
  - Wi-Fi connects in about 0.1–0.2 s using the cached access point and channel.
  - If Wi-Fi or the hub is unreachable, it gives up and goes back to sleep.
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
