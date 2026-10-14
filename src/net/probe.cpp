#include "tuushin/net/probe.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>

namespace tuushin::net {

// ---------------------------------------------------------------- reflector

bool ProbeReflector::Session::test(std::uint32_t seq) const {
  if (seq == 0 || seq > highest || highest - seq >= kWindow) return false;
  const std::uint32_t slot = seq % kWindow;
  return (seen[slot / 64] >> (slot % 64)) & 1u;
}

void ProbeReflector::Session::mark(std::uint32_t seq) {
  if (seq == 0) return;
  if (seq > highest) {
    // The slots between the old and the new highest held seqs a full window
    // older; forget them.
    if (seq - highest >= kWindow) {
      std::fill(seen.begin(), seen.end(), 0);
    } else {
      for (std::uint32_t k = highest + 1; k < seq; ++k) {
        const std::uint32_t slot = k % kWindow;
        seen[slot / 64] &= ~(std::uint64_t{1} << (slot % 64));
      }
    }
    highest = seq;
  } else if (highest - seq >= kWindow) {
    return;  // older than the window: nothing to remember it in
  }
  const std::uint32_t slot = seq % kWindow;
  seen[slot / 64] |= std::uint64_t{1} << (slot % 64);
}

void ProbeReflector::expire(Nanos now) {
  if (now - last_expiry_ < kNanosPerSecond) return;
  last_expiry_ = now;
  for (auto it = sessions_.begin(); it != sessions_.end();) {
    if (now - it->second.last_rx > idle_timeout_) {
      it = sessions_.erase(it);
    } else {
      ++it;
    }
  }
}

std::optional<ProbeReflector::Echo> ProbeReflector::on_probe(const wire::Header& probe,
                                                             Nanos rx_time, Nanos tx_time) {
  expire(rx_time);
  auto it = sessions_.find(probe.session);
  if (it == sessions_.end()) {
    if (sessions_.size() >= max_sessions_) return std::nullopt;
    it = sessions_.emplace(probe.session, Session{}).first;
  }
  Session& s = it->second;
  s.last_rx = rx_time;
  s.mark(probe.seq);

  Echo e;
  e.header.type = wire::PacketType::kProbeEcho;
  e.header.session = probe.session;
  e.header.seq = ++s.echo_seq;
  e.header.send_time = tx_time;
  e.header.echo_seq = probe.seq;
  e.header.echo_hold = wire::hold_field(tx_time - rx_time);
  for (std::uint32_t i = 0; i < 64; ++i) {
    if (probe.seq <= i + 1) break;
    if (s.test(probe.seq - 1 - i)) e.receipts |= std::uint64_t{1} << i;
  }
  e.payload = probe.payload_len;
  return e;
}

// ------------------------------------------------------------------ client

wire::Header ProbeClient::next_probe(Nanos t1) {
  ProbeRecord r;
  r.t1 = t1;
  records_.push_back(r);
  wire::Header h;
  h.type = wire::PacketType::kProbe;
  h.session = session_;
  h.seq = static_cast<std::uint32_t>(records_.size());
  h.send_time = t1;
  return h;
}

bool ProbeClient::on_datagram(const std::uint8_t* data, std::size_t size, Nanos t4) {
  const auto h = wire::decode_header(data, size);
  if (!h || h->type != wire::PacketType::kProbeEcho || h->session != session_ || h->echo_seq == 0 ||
      h->echo_seq > records_.size()) {
    ++stray_;
    return false;
  }
  ++echoes_;
  const std::uint32_t k = h->echo_seq;
  ProbeRecord& r = records_[k - 1];
  if (r.echoed) {
    ++r.duplicates;
  } else {
    r.echoed = true;
    r.t3 = h->send_time;
    r.t2 = h->send_time - static_cast<Nanos>(h->echo_hold);
    r.t4 = t4;
    r.reflector_seq = h->seq;
  }
  r.reflector_received = true;
  r.covered = true;
  if (const auto map = wire::decode_receipts(data, size)) {
    for (std::uint32_t i = 0; i < 64 && k > i + 1; ++i) {
      ProbeRecord& older = records_[k - 2 - i];
      older.covered = true;
      if ((*map >> i) & 1u) older.reflector_received = true;
    }
  }
  return true;
}

// ---------------------------------------------------------------- analysis

namespace {

double quantile(const std::vector<double>& sorted, double p) {
  // Nearest rank.
  const std::size_t n = sorted.size();
  std::size_t idx = static_cast<std::size_t>(std::ceil(p * static_cast<double>(n)));
  idx = idx == 0 ? 0 : idx - 1;
  return sorted[std::min(idx, n - 1)];
}

std::size_t longest_run(const std::vector<bool>& lost) {
  std::size_t best = 0, run = 0;
  for (bool l : lost) {
    run = l ? run + 1 : 0;
    best = std::max(best, run);
  }
  return best;
}

std::string format(const char* fmt, double a, double b = 0.0) {
  char buf[160];
  std::snprintf(buf, sizeof buf, fmt, a, b);
  return buf;
}

}  // namespace

DelayStats delay_stats(std::vector<double> delays, std::vector<std::pair<Nanos, double>> arrivals) {
  DelayStats s;
  s.count = delays.size();
  if (delays.empty()) return s;
  std::sort(delays.begin(), delays.end());
  s.min = delays.front();
  s.max = delays.back();
  s.p50 = quantile(delays, 0.50);
  s.p90 = quantile(delays, 0.90);
  s.p99 = quantile(delays, 0.99);
  s.p999 = quantile(delays, 0.999);
  std::sort(arrivals.begin(), arrivals.end());
  double j = 0.0;
  for (std::size_t i = 1; i < arrivals.size(); ++i) {
    const double d = std::fabs(arrivals[i].second - arrivals[i - 1].second);
    j += (d - j) / 16.0;
  }
  s.rfc3550_jitter = j;
  return s;
}

ProbeReport analyze_probes(const ProbeClient& client, Nanos clock_window) {
  ProbeReport rep;
  const auto& recs = client.records();
  rep.probes = recs.size();
  if (recs.empty()) return rep;

  std::vector<TimestampSample> samples;
  for (const auto& r : recs) {
    if (r.echoed) samples.push_back({r.t1, r.t2, r.t3, r.t4});
    rep.duplicates += static_cast<std::size_t>(r.duplicates);
  }
  rep.clock = fit_clock(samples, clock_window);
  const ClockModel& clk = rep.clock;

  // Forward delays: measured where echoed, interpolated in t1 where only a
  // receipt map says the probe arrived.
  const std::size_t n = recs.size();
  std::vector<double> fwd(n, 0.0);
  std::vector<bool> fwd_known(n, false);
  std::vector<double> holds;
  for (std::size_t k = 0; k < n; ++k) {
    const auto& r = recs[k];
    if (!r.echoed) continue;
    fwd[k] = static_cast<double>(r.t2 - r.t1) - clk.offset_at(r.t1);
    fwd_known[k] = true;
    holds.push_back(static_cast<double>(r.t3 - r.t2));
  }
  std::sort(holds.begin(), holds.end());
  const double hold_median = holds.empty() ? 0.0 : quantile(holds, 0.5);
  {
    std::size_t prev = n;  // index of the last known delay before k
    std::size_t next = 0;  // index of the first known delay at or after k
    for (std::size_t k = 0; k < n; ++k) {
      if (fwd_known[k]) {
        prev = k;
        continue;
      }
      if (!recs[k].reflector_received) continue;
      if (next <= k) {
        next = k + 1;
        while (next < n && !fwd_known[next]) ++next;
      }
      const bool has_prev = prev < n, has_next = next < n;
      if (has_prev && has_next) {
        const double ta = static_cast<double>(recs[prev].t1);
        const double tb = static_cast<double>(recs[next].t1);
        const double w = tb > ta ? (static_cast<double>(recs[k].t1) - ta) / (tb - ta) : 0.5;
        fwd[k] = fwd[prev] + w * (fwd[next] - fwd[prev]);
      } else if (has_prev) {
        fwd[k] = fwd[prev];
      } else if (has_next) {
        fwd[k] = fwd[next];
      }
      ++rep.interpolated;
    }
  }

  const Nanos t0 = recs.front().t1;
  std::vector<double> fwd_measured, bwd_measured, rtts;
  std::vector<std::pair<Nanos, double>> fwd_arrivals, bwd_arrivals;
  std::vector<bool> fwd_lost(n, false);
  struct Backward {
    Nanos send_offset;
    Nanos delay;
  };
  std::vector<Backward> bwd;
  bwd.reserve(n);
  // (reflector seq, client arrival) of every echo: the reflector's numbering
  // is its exact send order, which estimated send times are not when two
  // probes arrive at nearly the same moment.
  std::vector<std::pair<std::uint32_t, Nanos>> echo_order;
  Nanos max_t2 = INT64_MIN;
  for (std::size_t k = 0; k < n; ++k) {
    const auto& r = recs[k];
    rep.fwd.sent++;
    if (!r.reflector_received) {
      fwd_lost[k] = true;
      rep.fwd.lost++;
      if (!r.covered) rep.ambiguous++;
      rep.fwd_trace.entries.push_back({r.t1 - t0, -1});
      continue;
    }
    // Negative estimates (clock-fit noise on a sub-millisecond path) would
    // read as losses in a trace.
    rep.fwd_trace.entries.push_back({r.t1 - t0, std::max<Nanos>(0, std::llround(fwd[k]))});
    rep.bwd.sent++;
    if (r.echoed) {
      // Jitter from clock-corrected transits: raw ones carry the skew, which
      // adds skew * interval to every difference.
      fwd_measured.push_back(fwd[k]);
      fwd_arrivals.emplace_back(r.t2, fwd[k]);
      if (r.t2 < max_t2) rep.fwd.reordered++;
      max_t2 = std::max(max_t2, r.t2);
      const double theta = clk.offset_at(r.t4);
      const double b = static_cast<double>(r.t4 - r.t3) + theta;
      bwd_measured.push_back(b);
      bwd_arrivals.emplace_back(r.t4, b);
      echo_order.emplace_back(r.reflector_seq, r.t4);
      rtts.push_back(static_cast<double>((r.t4 - r.t1) - (r.t3 - r.t2)));
      const Nanos send_client = std::llround(static_cast<double>(r.t3) - theta);
      bwd.push_back({send_client - t0, std::max<Nanos>(0, std::llround(b))});
    } else {
      rep.bwd.lost++;
      const Nanos send_est = r.t1 - t0 + std::llround(fwd[k] + hold_median);
      bwd.push_back({send_est, -1});
    }
  }
  std::stable_sort(bwd.begin(), bwd.end(),
                   [](const Backward& a, const Backward& b) { return a.send_offset < b.send_offset; });
  std::vector<bool> bwd_lost;
  bwd_lost.reserve(bwd.size());
  for (const auto& b : bwd) {
    rep.bwd_trace.entries.push_back({b.send_offset, b.delay});
    bwd_lost.push_back(b.delay < 0);
  }
  std::sort(echo_order.begin(), echo_order.end());
  Nanos max_t4 = INT64_MIN;
  for (const auto& e : echo_order) {
    if (e.second < max_t4) rep.bwd.reordered++;
    max_t4 = std::max(max_t4, e.second);
  }
  if (!rep.bwd_trace.entries.empty()) {
    // Traces start at offset 0 by convention.
    const Nanos first = rep.bwd_trace.entries.front().send_offset;
    for (auto& e : rep.bwd_trace.entries) e.send_offset -= first;
  }
  rep.fwd.longest_burst = longest_run(fwd_lost);
  rep.bwd.longest_burst = longest_run(bwd_lost);
  rep.fwd.delay = delay_stats(std::move(fwd_measured), std::move(fwd_arrivals));
  rep.bwd.delay = delay_stats(std::move(bwd_measured), std::move(bwd_arrivals));
  rep.rtt = delay_stats(std::move(rtts), {});

  for (Trace* t : {&rep.fwd_trace, &rep.bwd_trace}) {
    t->comments.push_back(format("clock skew %.3f ppm, offset %.3f ms at the first probe",
                                 clk.skew * 1e6, clk.offset_at(t0) / 1e6));
    t->comments.push_back(
        "one-way delays assume equal minimum delay in both directions; an asymmetry a moves a/2 "
        "from one trace to the other");
  }
  rep.fwd_trace.comments.push_back(format("%.0f of the delays interpolated (echo lost)",
                                          static_cast<double>(rep.interpolated)));
  rep.fwd_trace.comments.push_back(format("%.0f losses ambiguous in direction, counted here",
                                          static_cast<double>(rep.ambiguous)));
  return rep;
}

}  // namespace tuushin::net
