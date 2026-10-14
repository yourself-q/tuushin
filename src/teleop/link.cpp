#include "tuushin/teleop/link.hpp"

#include <algorithm>

#include "tuushin/wire/packet.hpp"

namespace tuushin::teleop {

// ---------------------------------------------------------------- SeqWindow

bool SeqWindow::test(std::uint32_t seq) const {
  const std::uint32_t i = seq % kWindow;
  return (bits_[i / 64] >> (i % 64)) & 1u;
}

void SeqWindow::set(std::uint32_t seq) {
  const std::uint32_t i = seq % kWindow;
  bits_[i / 64] |= std::uint64_t{1} << (i % 64);
}

SeqWindow::Verdict SeqWindow::accept(std::uint32_t seq) {
  if (seq > highest_) {
    // Clear the slots the window slides over, which held seqs kWindow older.
    const std::uint32_t advance = seq - highest_;
    if (advance >= kWindow) {
      std::fill(bits_.begin(), bits_.end(), 0);
    } else {
      for (std::uint32_t k = 1; k <= advance; ++k) {
        const std::uint32_t i = (highest_ + k) % kWindow;
        bits_[i / 64] &= ~(std::uint64_t{1} << (i % 64));
      }
    }
    highest_ = seq;
    set(seq);
    ++distinct_;
    return Verdict::kNew;
  }
  if (highest_ - seq >= kWindow) return Verdict::kStale;
  if (test(seq)) return Verdict::kDuplicate;
  set(seq);
  ++distinct_;
  return Verdict::kLate;
}

// ---------------------------------------------------------------- HapticLink

HapticLink::HapticLink(net::UdpSocket& socket, std::uint32_t session, const net::Endpoint& peer,
                       Passivity scheme)
    : socket_(socket), session_(session), peer_(peer), scheme_(scheme) {}

void HapticLink::send(Nanos now, const Sample& s, double monitor, bool final) {
  wire::Header h;
  h.type = wire::PacketType::kHaptic;
  h.session = session_;
  h.seq = next_seq_++;
  h.flags = static_cast<std::uint16_t>(
      (final ? wire::kFlagFinal : 0) |
      ((static_cast<unsigned>(scheme_) << wire::kSchemeShift) & wire::kSchemeMask));
  h.send_time = now;
  h.echo_seq = window_.highest();
  h.echo_hold = window_.highest() == 0 ? 0 : wire::hold_field(now - highest_rx_time_);
  wire::HapticPayload p;
  p.dof = 1;
  p.energies = 2;
  p.value[0] = s.value;
  p.aux[0] = s.aux;
  p.energy[0] = s.energy;
  p.energy[1] = monitor;
  wire::encode_haptic(h, p, buf_);
  history_[h.seq % kHistory] = Sent{h.seq, now, false};
  ++counters_.sent;

  if (!emulation_) {
    transmit(buf_);
    return;
  }
  const auto transit = emulation_->transit(now, buf_.size());
  if (!transit) {
    ++counters_.emulated_lost;
    return;
  }
  pending_.push(Pending{now + *transit, pending_order_++, buf_});
}

void HapticLink::transmit(const std::vector<std::uint8_t>& datagram) {
  if (!socket_.send_to(peer_, datagram.data(), datagram.size())) ++counters_.send_failed;
}

void HapticLink::pump(Nanos now) {
  while (!pending_.empty() && pending_.top().release <= now) {
    transmit(pending_.top().bytes);
    pending_.pop();
  }
}

std::optional<Nanos> HapticLink::next_release() const {
  if (pending_.empty()) return std::nullopt;
  return pending_.top().release;
}

std::optional<HapticLink::Received> HapticLink::on_datagram(const std::uint8_t* data,
                                                            std::size_t size,
                                                            const net::Endpoint& from,
                                                            Nanos rx_time) {
  const auto h = wire::decode_header(data, size);
  if (!h || h->type != wire::PacketType::kHaptic || h->session != session_ || from != peer_ ||
      ((h->flags & wire::kSchemeMask) >> wire::kSchemeShift) != static_cast<unsigned>(scheme_)) {
    ++counters_.foreign;
    return std::nullopt;
  }
  const auto p = wire::decode_haptic(data, size);
  if (!p || p->dof != 1 || p->energies < 2) {
    ++counters_.foreign;
    return std::nullopt;
  }
  const auto verdict = window_.accept(h->seq);
  if (verdict == SeqWindow::Verdict::kDuplicate || verdict == SeqWindow::Verdict::kStale) {
    ++counters_.duplicates;
    return std::nullopt;
  }
  if (verdict == SeqWindow::Verdict::kNew) highest_rx_time_ = rx_time;
  if (verdict == SeqWindow::Verdict::kLate) ++counters_.late;

  // The echo: the peer's newest sample from us when it sent this one, and
  // how long it held it. One exchange per sample of ours, from its first
  // echo, which is the one with the least waiting in it.
  if (h->echo_seq != 0) {
    Sent& s = history_[h->echo_seq % kHistory];
    if (s.seq == h->echo_seq && !s.echoed) {
      s.echoed = true;
      net::TimestampSample x;
      x.t1 = s.t;
      x.t3 = h->send_time;
      x.t2 = h->send_time - static_cast<Nanos>(h->echo_hold);
      x.t4 = rx_time;
      rtts_.push_back(static_cast<double>(x.rtt()));
      if (exchanges_.size() < exchange_limit_) exchanges_.push_back(x);
    }
  }

  Received r;
  r.seq = h->seq;
  r.sample = Sample{p->value[0], p->aux[0], p->energy[0]};
  r.monitor = p->energy[1];
  r.send_time = h->send_time;
  r.rx_time = rx_time;
  r.final = (h->flags & wire::kFlagFinal) != 0;
  r.verdict = verdict;
  return r;
}

HapticLink::Counters HapticLink::counters() const {
  Counters c = counters_;
  c.received = window_.distinct();
  c.missing = window_.missing();
  return c;
}

}  // namespace tuushin::teleop
