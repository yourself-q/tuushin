#pragma once

#include <string>
#include <vector>

#include "tuushin/time.hpp"

namespace tuushin::net {

// One packet of a recorded one-way path.
struct TraceEntry {
  Nanos send_offset = 0;  // send time relative to the start of the recording
  Nanos delay = 0;        // one-way delay; negative means the packet was lost
  bool lost() const { return delay < 0; }
};

// A trace resampled onto points `bin` apart from its first send time, as
// tuushin-netreplay replays it on a Linux interface. A point's delay is
// interpolated linearly between the received packets sent around it (a
// lost packet has none), and the datagrams sent from one point to the next
// are lost if the packet sent nearest the point was: each packet stands
// for the path until the next, as in TraceChannel.
struct TraceGrid {
  Nanos bin = 0;
  std::vector<Nanos> delay;
  std::vector<bool> lost;

  // The delay of a datagram sent `t` >= 0 after the first point, the grid
  // looping, or -1 if it is lost: interpolated in integers between the
  // point before and the point after, exactly as the replay's eBPF program
  // computes it. Between points whose own datagrams arrive in order, so do
  // the datagrams sent between them. The last point leads to the first, so
  // where the trace ends more than a bin's worth of delay above where it
  // starts, the datagrams of that one bin can overtake each other once per
  // loop.
  Nanos delay_at(Nanos t) const;
  double loss_rate() const;
};

// A recorded one-way network path, replayable by TraceChannel.
//
// File format (CSV, UTF-8):
//   # free-form comment lines start with '#'
//   send_ns,delay_ns
//   0,12873311
//   1000000,-1          <- lost packet
// Entries are sorted by send_ns.
struct Trace {
  std::vector<TraceEntry> entries;
  std::vector<std::string> comments;

  // Time from the first to the last entry, plus one mean packet interval so
  // a looped replay keeps the original spacing.
  Nanos duration() const;
  double loss_rate() const;
  // The median time between consecutive sends; 0 with fewer than two
  // entries.
  Nanos median_interval() const;
  // The trace on points `bin` apart over duration(), at least one. Throws
  // std::invalid_argument for a bin <= 0 or a trace with no received packet.
  TraceGrid grid(Nanos bin) const;

  // Throws std::runtime_error on I/O or format errors.
  static Trace load_csv(const std::string& path);
  void save_csv(const std::string& path) const;
};

}  // namespace tuushin::net
