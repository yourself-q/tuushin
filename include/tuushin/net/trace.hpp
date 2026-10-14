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

  // Throws std::runtime_error on I/O or format errors.
  static Trace load_csv(const std::string& path);
  void save_csv(const std::string& path) const;
};

}  // namespace tuushin::net
