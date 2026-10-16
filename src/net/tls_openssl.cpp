// TLS over OpenSSL (Apache License 2.0).

#include <arpa/inet.h>
#include <sys/socket.h>

#include <openssl/err.h>
#include <openssl/ssl.h>
#include <openssl/x509v3.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "tuushin/net/tls.hpp"

namespace tuushin::net {

namespace {

#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;  // TcpStream sets SO_NOSIGPIPE instead
#endif

// The reason for the last error on this thread, without the codes.
std::string last_error(const char* fallback) {
  const unsigned long code = ERR_peek_last_error();
  const char* reason = code != 0 ? ERR_reason_error_string(code) : nullptr;
  return reason != nullptr ? reason : fallback;
}

// A BIO on the socket that sends with MSG_NOSIGNAL: OpenSSL's own socket
// BIO write()s, which raises SIGPIPE on Linux once the server has reset
// the connection.
int fd_of(BIO* b) { return static_cast<int>(reinterpret_cast<std::intptr_t>(BIO_get_data(b))); }

int bio_write(BIO* b, const char* data, int size) {
  BIO_clear_retry_flags(b);
  const ssize_t n = ::send(fd_of(b), data, static_cast<std::size_t>(size), kSendFlags);
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) BIO_set_retry_write(b);
  return static_cast<int>(n);
}

int bio_read(BIO* b, char* data, int size) {
  BIO_clear_retry_flags(b);
  const ssize_t n = ::recv(fd_of(b), data, static_cast<std::size_t>(size), 0);
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) BIO_set_retry_read(b);
  return static_cast<int>(n);
}

long bio_ctrl(BIO*, int cmd, long, void*) { return cmd == BIO_CTRL_FLUSH ? 1 : 0; }

int bio_create(BIO* b) {
  BIO_set_init(b, 1);
  return 1;
}

const BIO_METHOD* socket_method() {
  static BIO_METHOD* const method = [] {
    BIO_METHOD* m = BIO_meth_new(BIO_get_new_index() | BIO_TYPE_SOURCE_SINK, "tuushin socket");
    if (m == nullptr) return m;
    BIO_meth_set_write(m, bio_write);
    BIO_meth_set_read(m, bio_read);
    BIO_meth_set_ctrl(m, bio_ctrl);
    BIO_meth_set_create(m, bio_create);
    return m;
  }();
  return method;
}

// "stun.turn", length-prefixed as ALPN lists are (RFC 7443, 3).
constexpr unsigned char kAlpn[] = {9, 's', 't', 'u', 'n', '.', 't', 'u', 'r', 'n'};

bool is_ip_literal(const std::string& host) {
  std::uint8_t buffer[16];
  return ::inet_pton(AF_INET, host.c_str(), buffer) == 1 ||
         ::inet_pton(AF_INET6, host.c_str(), buffer) == 1;
}

}  // namespace

struct TlsConnector::Context {
  SSL_CTX* ctx = nullptr;
  std::string host;
  ~Context() { SSL_CTX_free(ctx); }
};

namespace {

// TLS on a TcpStream's socket: the TCP connect, then the handshake, then
// records. Bytes written wait in out_ while the socket takes no more, as
// TcpStream's do; SSL_write is retried with what waits, which starts with
// what it was first given (SSL_MODE_ACCEPT_MOVING_WRITE_BUFFER).
class TlsStream : public Stream {
 public:
  TlsStream(SSL* ssl, const Endpoint& to, const Endpoint& local, int dscp)
      : tcp_(to, local, dscp), ssl_(ssl) {
    if (tcp_.state() == State::kClosed) close_with(tcp_.error());
  }
  ~TlsStream() override {
    // A close_notify, if the socket takes it now; the server learns of the
    // close from the FIN in any case.
    if (state_ == State::kOpen) {
      ERR_clear_error();
      SSL_shutdown(ssl_);
    }
    SSL_free(ssl_);
    ERR_clear_error();
  }
  TlsStream(const TlsStream&) = delete;
  TlsStream& operator=(const TlsStream&) = delete;

  State state() override {
    if (state_ != State::kConnecting) return state_;
    if (!attached_) {
      const State tcp = tcp_.state();
      if (tcp == State::kClosed) close_with(tcp_.error());
      if (tcp != State::kOpen) return state_;
      BIO* bio = BIO_new(socket_method());
      if (bio == nullptr) {
        close_with("cannot make a BIO");
        return state_;
      }
      BIO_set_data(bio, reinterpret_cast<void*>(static_cast<std::intptr_t>(tcp_.native_handle())));
      SSL_set_bio(ssl_, bio, bio);
      attached_ = true;
    }
    ERR_clear_error();
    const int r = SSL_connect(ssl_);
    if (r == 1) {
      state_ = State::kOpen;
      want_write_ = false;
      flush();
    } else {
      on_error(r, "handshake");
    }
    return state_;
  }

  const std::string& error() const override { return error_; }

  bool write(const std::uint8_t* data, std::size_t size) override {
    if (state_ == State::kClosed) return false;
    if (out_.size() - head_ + size > kMaxBacklog) return false;
    std::size_t sent = 0;
    if (state_ == State::kOpen && head_ == out_.size()) {
      const long n = write_some(data, size);
      if (n < 0) return false;
      sent = static_cast<std::size_t>(n);
    }
    if (sent < size) {
      if (head_ == out_.size()) {
        out_.clear();
        head_ = 0;
      }
      out_.insert(out_.end(), data + sent, data + size);
    }
    return true;
  }

  void flush() override {
    if (state_ != State::kOpen) return;
    while (head_ < out_.size()) {
      const long n = write_some(out_.data() + head_, out_.size() - head_);
      if (n <= 0) return;
      head_ += static_cast<std::size_t>(n);
    }
    out_.clear();
    head_ = 0;
  }

  long read(std::uint8_t* buffer, std::size_t capacity) override {
    if (state_ == State::kConnecting && state() != State::kOpen) {
      return state_ == State::kClosed ? -1 : 0;
    }
    if (state_ == State::kClosed) return -1;
    ERR_clear_error();
    const int n = SSL_read(ssl_, buffer, static_cast<int>(std::min<std::size_t>(capacity, INT_MAX)));
    if (n > 0) {
      want_write_ = false;
      return n;
    }
    on_error(n, "recv");
    return state_ == State::kClosed ? -1 : 0;
  }

  bool has_buffered() const override { return state_ == State::kOpen && SSL_pending(ssl_) > 0; }

  int native_handle() const override { return tcp_.native_handle(); }
  bool wants_write() const override {
    if (state_ == State::kConnecting) return !attached_ || want_write_;
    return state_ == State::kOpen && (head_ < out_.size() || want_write_);
  }

 private:
  // What SSL_write took (a record at a time), 0 if the socket takes no
  // more now, -1 once closed.
  long write_some(const std::uint8_t* data, std::size_t size) {
    ERR_clear_error();
    const int n = SSL_write(ssl_, data, static_cast<int>(std::min<std::size_t>(size, INT_MAX)));
    if (n > 0) {
      want_write_ = false;
      return n;
    }
    on_error(n, "send");
    return state_ == State::kClosed ? -1 : 0;
  }

  // After a call that returned `r`: waits for the socket, or closes with
  // the reason.
  void on_error(int r, const char* what) {
    const int e = SSL_get_error(ssl_, r);
    if (e == SSL_ERROR_WANT_READ) {
      want_write_ = false;
      return;
    }
    if (e == SSL_ERROR_WANT_WRITE) {
      want_write_ = true;
      return;
    }
    if (e == SSL_ERROR_ZERO_RETURN) {
      close_with("closed by the other end");
      return;
    }
    if (e == SSL_ERROR_SYSCALL && ERR_peek_last_error() == 0) {
      close_with(errno != 0 ? std::string(what) + ": " + std::strerror(errno)
                            : std::string("closed by the other end"));
      return;
    }
    const long verify = SSL_get_verify_result(ssl_);
    if (verify != X509_V_OK) {
      close_with(std::string("the server's certificate does not check out: ") +
                 X509_verify_cert_error_string(verify));
      return;
    }
    close_with(std::string(what) + ": " + last_error("TLS error"));
  }

  void close_with(const std::string& why) {
    if (state_ == State::kClosed) return;
    state_ = State::kClosed;
    error_ = why;
    out_.clear();
    head_ = 0;
    want_write_ = false;
  }

  TcpStream tcp_;
  SSL* ssl_;
  State state_ = State::kConnecting;
  bool attached_ = false;    // the socket BIO is set: the handshake has begun
  bool want_write_ = false;  // the last call is to be retried once the socket takes more
  std::string error_;
  std::vector<std::uint8_t> out_;
  std::size_t head_ = 0;  // out_[head_..] waits
};

}  // namespace

bool tls_supported() { return true; }

TlsConnector::TlsConnector(const TlsSettings& settings) {
  auto context = std::make_shared<Context>();
  ERR_clear_error();
  context->ctx = SSL_CTX_new(TLS_client_method());
  if (context->ctx == nullptr) throw std::runtime_error("TLS: " + last_error("cannot make a context"));
  SSL_CTX* ctx = context->ctx;
  SSL_CTX_set_min_proto_version(ctx, TLS1_2_VERSION);
  SSL_CTX_set_mode(ctx, SSL_MODE_ENABLE_PARTIAL_WRITE | SSL_MODE_ACCEPT_MOVING_WRITE_BUFFER);
  SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, nullptr);
  if (!settings.ca_file.empty()) {
    // The system's reason, where OpenSSL's would be "BIO lib".
    if (std::FILE* f = std::fopen(settings.ca_file.c_str(), "r")) {
      std::fclose(f);
    } else {
      throw std::runtime_error("TLS: cannot read " + settings.ca_file + ": " + std::strerror(errno));
    }
    if (SSL_CTX_load_verify_locations(ctx, settings.ca_file.c_str(), nullptr) != 1) {
      throw std::runtime_error("TLS: cannot load the certificates in " + settings.ca_file + ": " +
                               last_error("unreadable"));
    }
  } else if (SSL_CTX_set_default_verify_paths(ctx) != 1) {
    throw std::runtime_error("TLS: cannot load the system's certificates: " + last_error("unreadable"));
  }
  ERR_clear_error();
  context->host = settings.host;
  context_ = std::move(context);
}

std::unique_ptr<Stream> TlsConnector::open(const Endpoint& to, const Endpoint& local, int dscp) const {
  ERR_clear_error();
  SSL* ssl = SSL_new(context_->ctx);
  if (ssl == nullptr) throw std::runtime_error("TLS: " + last_error("cannot make a connection"));
  // The name or address the certificate is to carry. SNI carries names
  // only (RFC 6066, 3).
  const std::string& host = context_->host;
  bool named = false;
  if (host.empty()) {
    const auto address = to.address();
    named = X509_VERIFY_PARAM_set1_ip(SSL_get0_param(ssl), address.data(), address.size()) == 1;
  } else if (is_ip_literal(host)) {
    named = X509_VERIFY_PARAM_set1_ip_asc(SSL_get0_param(ssl), host.c_str()) == 1;
  } else {
    named = SSL_set_tlsext_host_name(ssl, host.c_str()) == 1 && SSL_set1_host(ssl, host.c_str()) == 1;
  }
  // (SSL_set_alpn_protos returns 0 on success.)
  if (!named || SSL_set_alpn_protos(ssl, kAlpn, sizeof kAlpn) != 0) {
    SSL_free(ssl);
    throw std::runtime_error("TLS: cannot set up the connection to " + to.to_string());
  }
  try {
    return std::make_unique<TlsStream>(ssl, to, local, dscp);
  } catch (...) {
    // The stream did not take the SSL: its constructor threw in TcpStream's.
    SSL_free(ssl);
    throw;
  }
}

}  // namespace tuushin::net
