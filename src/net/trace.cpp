#include "tuushin/net/trace.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace tuushin::net {

Nanos Trace::duration() const {
  if (entries.size() < 2) return entries.empty() ? 0 : 1;
  const Nanos span = entries.back().send_offset - entries.front().send_offset;
  const Nanos interval = span / static_cast<Nanos>(entries.size() - 1);
  return span + std::max<Nanos>(interval, 1);
}

double Trace::loss_rate() const {
  if (entries.empty()) return 0.0;
  const auto lost = std::count_if(entries.begin(), entries.end(),
                                  [](const TraceEntry& e) { return e.lost(); });
  return static_cast<double>(lost) / static_cast<double>(entries.size());
}

Nanos Trace::median_interval() const {
  if (entries.size() < 2) return 0;
  std::vector<Nanos> gaps;
  gaps.reserve(entries.size() - 1);
  for (std::size_t i = 1; i < entries.size(); ++i) {
    gaps.push_back(entries[i].send_offset - entries[i - 1].send_offset);
  }
  std::nth_element(gaps.begin(), gaps.begin() + gaps.size() / 2, gaps.end());
  return gaps[gaps.size() / 2];
}

TraceGrid Trace::grid(Nanos bin) const {
  if (bin <= 0) throw std::invalid_argument("trace grid: bin must be positive");
  std::vector<const TraceEntry*> received;
  for (const auto& e : entries) {
    if (!e.lost()) received.push_back(&e);
  }
  if (received.empty()) throw std::invalid_argument("trace grid: no packet was received");
  TraceGrid g;
  g.bin = bin;
  const std::size_t n = static_cast<std::size_t>(
      std::max<Nanos>(1, (duration() + bin / 2) / bin));
  g.delay.resize(n);
  g.lost.resize(n);
  const Nanos first = entries.front().send_offset;
  std::size_t r = 0;  // the first received packet sent after the point
  std::size_t e = 0;  // the packet sent nearest the point
  for (std::size_t k = 0; k < n; ++k) {
    const Nanos at = first + static_cast<Nanos>(k) * bin;
    while (r < received.size() && received[r]->send_offset <= at) ++r;
    double d;
    if (r == 0) {
      d = static_cast<double>(received.front()->delay);
    } else if (r == received.size()) {
      d = static_cast<double>(received.back()->delay);
    } else {
      const TraceEntry& a = *received[r - 1];
      const TraceEntry& b = *received[r];
      const double f = static_cast<double>(at - a.send_offset) /
                       static_cast<double>(b.send_offset - a.send_offset);
      d = static_cast<double>(a.delay) + f * static_cast<double>(b.delay - a.delay);
    }
    g.delay[k] = std::max<Nanos>(0, std::llround(d));
    while (e + 1 < entries.size() &&
           std::llabs(entries[e + 1].send_offset - at) <= std::llabs(entries[e].send_offset - at)) {
      ++e;
    }
    g.lost[k] = entries[e].lost();
  }
  return g;
}

Nanos TraceGrid::delay_at(Nanos t) const {
  const auto n = static_cast<Nanos>(delay.size());
  if (n == 0 || bin <= 0) return 0;
  // Unsigned, as in the eBPF program.
  const auto ut = static_cast<std::uint64_t>(t) % static_cast<std::uint64_t>(bin * n);
  const auto ubin = static_cast<std::uint64_t>(bin);
  const auto i = static_cast<std::size_t>(ut / ubin);
  const std::size_t j = i + 1 < delay.size() ? i + 1 : 0;
  const std::uint64_t frac = ut - i * ubin;
  if (lost[i]) return -1;
  auto d = static_cast<std::uint64_t>(delay[i]);
  const auto next = static_cast<std::uint64_t>(delay[j]);
  if (next >= d) {
    d += (next - d) * frac / ubin;
  } else {
    d -= (d - next) * frac / ubin;
  }
  return static_cast<Nanos>(d);
}

double TraceGrid::loss_rate() const {
  if (lost.empty()) return 0.0;
  return static_cast<double>(std::count(lost.begin(), lost.end(), true)) /
         static_cast<double>(lost.size());
}

namespace {

Nanos parse_int(const std::string& s, const std::string& path, std::size_t line_no) {
  if (s.empty()) return -1;  // an empty delay field also means "lost"
  char* end = nullptr;
  const long long v = std::strtoll(s.c_str(), &end, 10);
  if (end == s.c_str() || *end != '\0') {
    throw std::runtime_error(path + ":" + std::to_string(line_no) + ": not an integer: '" + s + "'");
  }
  return static_cast<Nanos>(v);
}

std::string trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t\r");
  if (b == std::string::npos) return {};
  const auto e = s.find_last_not_of(" \t\r");
  return s.substr(b, e - b + 1);
}

}  // namespace

Trace Trace::load_csv(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open trace: " + path);
  Trace t;
  std::string line;
  std::size_t line_no = 0;
  bool header_seen = false;
  while (std::getline(in, line)) {
    ++line_no;
    line = trim(line);
    if (line.empty()) continue;
    if (line[0] == '#') {
      t.comments.push_back(trim(line.substr(1)));
      continue;
    }
    if (!header_seen) {
      if (line != "send_ns,delay_ns") {
        throw std::runtime_error(path + ": expected header 'send_ns,delay_ns'");
      }
      header_seen = true;
      continue;
    }
    const auto comma = line.find(',');
    if (comma == std::string::npos) {
      throw std::runtime_error(path + ":" + std::to_string(line_no) + ": missing ','");
    }
    TraceEntry e;
    e.send_offset = parse_int(trim(line.substr(0, comma)), path, line_no);
    e.delay = parse_int(trim(line.substr(comma + 1)), path, line_no);
    t.entries.push_back(e);
  }
  if (!header_seen) throw std::runtime_error(path + ": empty trace");
  std::stable_sort(t.entries.begin(), t.entries.end(),
                   [](const TraceEntry& a, const TraceEntry& b) { return a.send_offset < b.send_offset; });
  return t;
}

void Trace::save_csv(const std::string& path) const {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write trace: " + path);
  for (const auto& c : comments) out << "# " << c << '\n';
  out << "send_ns,delay_ns\n";
  for (const auto& e : entries) {
    out << e.send_offset << ',' << (e.lost() ? -1 : e.delay) << '\n';
  }
  if (!out) throw std::runtime_error("write failed: " + path);
}

}  // namespace tuushin::net
