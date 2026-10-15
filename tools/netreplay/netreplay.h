// What tuushin-netreplay and its eBPF program share: the layout of their maps.
#pragma once

#include <linux/types.h>

// The replay's one entry of the `config` map.
struct netreplay_config {
  __u64 start_ns;  // CLOCK_MONOTONIC time of the first point
  __u64 bin_ns;    // spacing of the points
  __u32 points;    // number of points; the trace loops after the last
  __u16 port;      // replay only UDP from or to this port; 0 for any
  __u16 pad;
};

// One point of the `points` map.
struct netreplay_point {
  __u64 delay_ns;  // the one-way delay of a datagram sent at this point
  __u32 lost;      // nonzero: datagrams sent from here to the next point are lost
  __u32 pad;
};

// Indices of the `stats` map, a per-CPU count each.
enum {
  NETREPLAY_DELAYED,  // UDP datagrams held for the trace's delay
  NETREPLAY_DROPPED,  // UDP datagrams dropped where the trace lost one
  NETREPLAY_PASSED,   // everything else, sent as it came
  NETREPLAY_STATS,
};
