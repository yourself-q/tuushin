// Replays a recorded one-way path on an interface's egress (tc clsact,
// direct action), for tuushin-netreplay: each UDP datagram is held for the
// delay the trace recorded at the moment it is sent, or dropped where the
// trace lost its packet.
//
// The delay is set as the datagram's earliest departure time, skb->tstamp,
// which the fq qdisc that tuushin-netreplay makes the interface's root
// honours: fq keeps the datagram until then, in departure order within a
// flow, as it does for the kernel's own TCP pacing. Nothing is copied and
// nothing runs per datagram but this program. The same mechanism was used
// for delay emulation by Becker et al. (arXiv 2208.05862) and for replaying
// satellite traces by Tian et al. (arXiv 2408.15581).
//
// The trace is a grid of points bin_ns apart from start_ns, looped, and the
// lookup is by the time a datagram is sent, not by its number, so traffic
// at any rate meets the path as it was at that moment. Between two points
// the delay is interpolated linearly. A datagram sent at t then leaves at
// t + d(t), which, d being linear between points whose own datagrams were
// received in order, puts the datagrams between them in order as well;
// holding each point's delay as a step would reorder a datagram sent late
// in a bin behind the next bin's first whenever the delay falls.

#include <linux/bpf.h>
#include <linux/if_ether.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/pkt_cls.h>
#include <linux/udp.h>

#include <bpf/bpf_endian.h>
#include <bpf/bpf_helpers.h>

#include "netreplay.h"

struct {
  __uint(type, BPF_MAP_TYPE_ARRAY);
  __uint(max_entries, 1);
  __type(key, __u32);
  __type(value, struct netreplay_config);
} config SEC(".maps");

// Resized to the trace's points by the loader before the program loads.
struct {
  __uint(type, BPF_MAP_TYPE_ARRAY);
  __uint(max_entries, 1);
  __type(key, __u32);
  __type(value, struct netreplay_point);
} points SEC(".maps");

struct {
  __uint(type, BPF_MAP_TYPE_PERCPU_ARRAY);
  __uint(max_entries, NETREPLAY_STATS);
  __type(key, __u32);
  __type(value, __u64);
} stats SEC(".maps");

// Ethernet, the longest IPv4 header, UDP.
#define HEADERS (14 + 60 + 8)

static __always_inline int count(__u32 what, int action) {
  __u64* n = bpf_map_lookup_elem(&stats, &what);
  if (n) *n += 1;
  return action;
}

// 1 if the frame is a UDP datagram from or to `port` (any port for 0), 0 if
// not. A later IPv4 fragment carries no ports: it matches only for port 0.
static __always_inline int is_udp(struct __sk_buff* skb, __u16 port) {
  if (skb->data_end - skb->data < HEADERS && skb->len > skb->data_end - skb->data) {
    bpf_skb_pull_data(skb, skb->len < HEADERS ? skb->len : HEADERS);
  }
  void* data = (void*)(long)skb->data;
  void* end = (void*)(long)skb->data_end;
  struct ethhdr* eth = data;
  if ((void*)(eth + 1) > end) return 0;
  struct udphdr* udp;
  if (eth->h_proto == bpf_htons(ETH_P_IP)) {
    struct iphdr* ip = (void*)(eth + 1);
    if ((void*)(ip + 1) > end || ip->protocol != IPPROTO_UDP) return 0;
    if (ip->frag_off & bpf_htons(0x1fff)) return port == 0;
    if (ip->ihl < 5) return 0;
    udp = (void*)ip + ip->ihl * 4;
  } else if (eth->h_proto == bpf_htons(ETH_P_IPV6)) {
    struct ipv6hdr* ip6 = (void*)(eth + 1);
    if ((void*)(ip6 + 1) > end || ip6->nexthdr != IPPROTO_UDP) return 0;
    udp = (void*)(ip6 + 1);
  } else {
    return 0;
  }
  if ((void*)(udp + 1) > end) return 0;
  return port == 0 || udp->source == bpf_htons(port) || udp->dest == bpf_htons(port);
}

SEC("tc")
int netreplay(struct __sk_buff* skb) {
  __u32 zero = 0;
  struct netreplay_config* cfg = bpf_map_lookup_elem(&config, &zero);
  if (!cfg || cfg->points == 0 || cfg->bin_ns == 0) return count(NETREPLAY_PASSED, TC_ACT_OK);
  if (!is_udp(skb, cfg->port)) return count(NETREPLAY_PASSED, TC_ACT_OK);
  const __u64 now = bpf_ktime_get_ns();
  if (now < cfg->start_ns) return count(NETREPLAY_PASSED, TC_ACT_OK);

  const __u64 t = (now - cfg->start_ns) % (cfg->bin_ns * cfg->points);
  __u32 i = t / cfg->bin_ns;
  __u32 j = i + 1 < cfg->points ? i + 1 : 0;
  const __u64 frac = t - (__u64)i * cfg->bin_ns;
  struct netreplay_point* a = bpf_map_lookup_elem(&points, &i);
  struct netreplay_point* b = bpf_map_lookup_elem(&points, &j);
  if (!a || !b) return count(NETREPLAY_PASSED, TC_ACT_OK);
  if (a->lost) return count(NETREPLAY_DROPPED, TC_ACT_SHOT);

  __u64 d = a->delay_ns;
  if (b->delay_ns >= d) {
    d += (b->delay_ns - d) * frac / cfg->bin_ns;
  } else {
    d -= (d - b->delay_ns) * frac / cfg->bin_ns;
  }
  // From now, not from a time already set: a datagram from a local UDP
  // socket carries none, and a forwarded one may carry its receive time on
  // another clock.
  skb->tstamp = now + d;
  return count(NETREPLAY_DELAYED, TC_ACT_OK);
}

// Calls no GPL-only helper, so it loads under the repository's license.
char LICENSE[] SEC("license") = "MIT";
