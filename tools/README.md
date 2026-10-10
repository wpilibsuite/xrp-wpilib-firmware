# BTstack L2CAP build override

`btstack_l2cap.py` corrects the channel ID allocator in the BTstack source bundled
with Arduino-Pico. LE dynamic channel IDs must stay within `0x0040..0x007f`.
The bundled allocator instead increments through the Classic range; Linux
rejects its first LE response containing CID `0x0080`. Repeated reconnects can
then leave a stale device channel and force subsequent attempts onto GATT.
This is independent of the XRP control packet sequence counter.

The PlatformIO post script copies and patches the matching framework source into
`$BUILD_DIR/btstack_l2cap/src/l2cap.c`, builds it with the framework's final flags,
and places its archive before the framework's prebuilt libraries. The shared
PlatformIO installation is untouched. Classic retains its original range, and
allocation continues to skip IDs belonging to active channels.

The patch requires exactly one match for each replacement and fails the build
if those source fragments change. Review it when updating Arduino-Pico/BTstack;
remove the override once the dependency supplies the equivalent correction.

Validation:

```sh
pio run -t clean
pio run
python3 test/run_native.py
```

The native regression compiles the same patched source and exercises real
BTstack allocation, channel lookup, and channel release. See
[`test/README`](../test/README) for coverage and hardware retesting, and
[`test/hardware-validation.md`](../test/hardware-validation.md) for observed
failure boundaries and hardware results.

Linux's validation is in
[`l2cap_le_connect_rsp()`](https://github.com/torvalds/linux/blob/v6.17/net/bluetooth/l2cap_core.c#L4451).

Hardware validation on September 26, 2026 completed 159 successful connection
attempts (158 L2CAP, one GATT fallback), with control/status traffic on every
attempt and two legal CID wraps. See the
[hardware validation record](../test/hardware-validation.md). The L2CAP
reconnects reused a physical LE link; occasional fresh-link HCI failures remain
a separate observation.
