#include "tuushin/net/trace.hpp"

#include <algorithm>
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
