# Bluetooth reconnect hardware validation

Validated September 26, 2026 with the Linux halsim_xrp client, Linux
6.17.0-29-generic, and the project-local BTstack CID correction. Firmware builds
used Arduino-Pico `1.40502.0+sha.227d71e` from the pinned PlatformIO platform.

Before the correction, the device allocated LE CID `0x0080` after `0x007f`.
Linux rejected that response; the device retained the channel and rejected later
requests using the same peer CID. Those attempts persistently fell back to GATT.

After the correction:

| Observation | Result |
| --- | ---: |
| Successful application connections | 159 / 159 |
| L2CAP connections | 158 |
| GATT fallback connections | 1 |
| Attempts with control and status traffic | 159 / 159 |
| Connection timeouts or terminal client errors | 0 |
| Legal CID wraps, `0x007f` to `0x0040` | 2 |
| Device CIDs outside `0x0040..0x007f` | 0 |
| Classic Bluetooth inquiry commands | 0 |

The first physical-link attempt failed during remote feature exchange with
controller status `0x3e` (Connection Failed to be Established). GATT fallback
recovered. Subsequent L2CAP reconnects reused the physical LE connection after
GATT notifications were enabled. These results validate channel reconnects and
CID rollover; they do not represent 159 separate radio connections. The cause
of intermittent fresh-link failures remains unresolved.

Other issues isolated during the investigation:

- GATT controls could arrive while pending status transmission remained pinned
  to a credit-starved L2CAP channel. Transport-switch regressions cover status
  delivery after fallback and delayed callbacks from the prior transport.
- The Linux client previously ignored libuv poll errors, turning immediate
  connection failures into eight-second timeouts. Client-side regressions cover
  error handling, stale callbacks, and a bounded GATT ENOMEM retry.
- A newer host kernel had a separate L2CAP signaling-identifier leak; switching
  kernels removed that failure mode. Classic discovery from KDE Connect also
  delayed LE setup and was disabled for the final run.

Raw captures and detailed investigation notes are retained locally under
`diagnostics/bluetooth/2026-09-26/`; they are not part of the source commit.
Run the native sanitizer suites and both firmware builds as described in
[README](README). Additional hardware tests, including full physical-link
reconnects, are also listed there.
