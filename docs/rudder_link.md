# Rudder link

## Delivery

One-way and fire-and-forget: heading streams out at a fixed rate, never
acknowledged, because it is latest-wins.

- A dropped frame is superseded by the next, not retransmitted.
- `seq` lets the rudder count drops.
- A receiver timeout makes the rudder fail safe when frames stop.

## COBS

COBS (Consistent Overhead Byte Stuffing) rewrites a byte string so one chosen
value, here `0x00`, never appears inside it, freeing that value to mark frame
boundaries.

- Every zero becomes a code byte holding the distance to the next zero.
- The receiver resyncs at the next `0x00`.

```
11 22 00 33        payload
03 11 22 02 33 00  encoded (03 = two bytes then a zero, 02 = one byte then end)
```

## Framing

Before COBS a frame is, all little-endian:

```
[ver:u8][msg_id:u8][seq:u8][payload][crc16:u16]
```

- **`ver`** rejects a firmware mismatch between the two ends.
- **`msg_id`** selects the payload layout.
- **`seq`** wraps at 256; lets the rudder count drops.
- **`payload`** the body named by `msg_id`; today a single float32 heading.
- **`crc16`** CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) over `ver..payload`.

No length field: COBS bounds the frame and `msg_id` gives its size.

Fields sit at unaligned offsets, so a read cannot reinterpret a pointer into the
frame (undefined, and a fault on the M0+); a small helper gathers the bytes into
an aligned value and `bit_cast`s it. Little-endian only means that gather needs
no byte swap.

## Messages

| `msg_id` | Name | Payload |
|---|---|---|
| `0x01` | Heading | float32 heading, compass degrees in `-180..180` (`attitude.md`) |

A new sensor or command is a new `msg_id` with its own payload; the rudder
ignores ids it does not recognise.

