#pragma once

#include <memory>
#include <string>

#include "tuushin/net/tcp.hpp"

namespace tuushin::net {

// How a TLS connection checks the server it reaches.
struct TlsSettings {
  // The name the server's certificate must carry: a DNS name, sent as SNI
  // too, or an IP address; empty for the address connected to.
  std::string host;
  // A file of PEM certificates to trust instead of the system's, for a
  // server with a certificate of its own CA (a lab's, say).
  std::string ca_file;
};

// Whether this build has TLS (OpenSSL was found when it was configured).
bool tls_supported();

// The client end of TLS connections to one server, loading the
// certificates to trust once for all of them (a relay reconnects).
class TlsConnector {
 public:
  // Throws std::runtime_error if this build has no TLS or the CA file
  // cannot be read.
  explicit TlsConnector(const TlsSettings& settings);

  // TLS 1.2 or later over a TcpStream (tcp.hpp) to `to`, offering
  // "stun.turn" by ALPN (RFC 7443). The stream is kConnecting through the
  // TCP connect and the handshake, and closes, with the reason, if the
  // server's certificate does not check out. Throws std::system_error as
  // TcpStream does.
  std::unique_ptr<Stream> open(const Endpoint& to, const Endpoint& local, int dscp) const;

 private:
  struct Context;
  std::shared_ptr<const Context> context_;
};

}  // namespace tuushin::net
