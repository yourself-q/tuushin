// TLS for a build without OpenSSL: none.

#include <stdexcept>

#include "tuushin/net/tls.hpp"

namespace tuushin::net {

struct TlsConnector::Context {};

bool tls_supported() { return false; }

TlsConnector::TlsConnector(const TlsSettings&) {
  throw std::runtime_error("this build has no TLS (OpenSSL was not found when it was configured)");
}

std::unique_ptr<Stream> TlsConnector::open(const Endpoint&, const Endpoint&, int) const {
  return nullptr;
}

}  // namespace tuushin::net
