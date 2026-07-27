#ifdef ARDUINO
#include "rtt.h"

namespace {

// Layout fixed by the SEGGER RTT spec so openocd's scanner can parse it: a
// ring buffer descriptor is name/buffer pointers, size, then the two offsets
// (target owns write, host owns read) and flags.
struct Buffer {
  const char *name;
  char *buffer;
  unsigned size;
  volatile unsigned write_off; // advanced by the target
  volatile unsigned read_off;  // advanced by the host (openocd)
  unsigned flags;
};

struct ControlBlock {
  char id[16];
  int max_up;
  int max_down;
  Buffer up[1];
  Buffer down[1];
};

// 64 KB so a single SWD RAM dump captures a full magnetometer tumble (~30 s of
// 10 Hz telemetry), not just the last few lines a small ring would hold.
constexpr unsigned UP_SIZE = 65536;
char g_up_buffer[UP_SIZE];
char g_down_buffer[16]; // unused; present so the layout is well-formed

// `used` keeps it in RAM (.data) for the scanner; the "SEGGER RTT" id is the
// magic openocd's `rtt setup` searches for.
ControlBlock g_rtt __attribute__((used)) = {
    {'S', 'E', 'G', 'G', 'E', 'R', ' ', 'R', 'T', 'T', 0, 0, 0, 0, 0, 0},
    1,
    1,
    {{"Terminal", g_up_buffer, UP_SIZE, 0, 0, 0}},
    {{"Terminal", g_down_buffer, sizeof(g_down_buffer), 0, 0, 0}},
};

} // namespace

namespace rtt {

size_t write(const uint8_t *data, size_t len) {
  Buffer &b = g_rtt.up[0];
  unsigned wr = b.write_off;
  unsigned rd = b.read_off;
  for (size_t i = 0; i < len; i++) {
    unsigned next = wr + 1;
    if (next >= b.size) {
      next = 0;
    }
    // Overwrite the oldest byte when full: push read_off forward so the ring
    // always holds the most recent b.size bytes. A snapshot RAM dump (no live
    // host draining read_off) then reflects the latest telemetry, not just the
    // first burst after boot. A live reader that keeps up never hits this.
    if (next == rd) {
      rd = (rd + 1 >= b.size) ? 0 : rd + 1;
    }
    b.buffer[wr] = static_cast<char>(data[i]);
    wr = next;
  }
  // Publish the buffer writes before the offsets the host polls.
  __asm__ volatile("dmb" ::: "memory");
  b.read_off = rd;
  b.write_off = wr;
  return len;
}

size_t write(uint8_t c) { return write(&c, 1); }

} // namespace rtt

#endif
