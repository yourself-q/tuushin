#pragma once

#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <openssl/err.h>
#include <openssl/pem.h>
#include <openssl/ssl.h>
#include <openssl/x509v3.h>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "tuushin/net/udp.hpp"

namespace test {

namespace net = tuushin::net;

// A TLS front for tests on IPv4 loopback: it takes TLS connections and
// passes what they carry, in the clear, to a TCP server behind it (a
// SimTurn, sim_turn.hpp), as a TLS-terminating proxy in front of a TURN
// server does; and closes each side when the other closes. Its certificate
// is self-signed, made for it with the subjectAltName entries given, and
// written to a PEM file that a client can trust (ca_file()).
class TlsFront {
 public:
  // `names` as in a configuration file: "DNS:turn.test,IP:127.0.0.1".
  TlsFront(const net::Endpoint& backend, const std::string& names) : backend_(backend) {
    // The server end's writes go through OpenSSL's socket BIO, which
    // write()s: on Linux, a client that has gone raises SIGPIPE.
    std::signal(SIGPIPE, SIG_IGN);
    make_credentials(names);
    endpoint_ = *net::Endpoint::resolve("127.0.0.1", 0);
    listener_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    socklen_t len = net::Endpoint::kCapacity;
    if (listener_ < 0 ||
        ::bind(listener_, static_cast<const sockaddr*>(endpoint_.data()),
               static_cast<socklen_t>(endpoint_.size())) != 0 ||
        ::listen(listener_, 8) != 0 ||
        ::getsockname(listener_, static_cast<sockaddr*>(endpoint_.data()), &len) != 0) {
      throw std::runtime_error("TlsFront: cannot listen");
    }
    endpoint_.set_size(static_cast<std::uint32_t>(len));
    ::fcntl(listener_, F_SETFL, ::fcntl(listener_, F_GETFL, 0) | O_NONBLOCK);
    thread_ = std::thread([this] { run(); });
  }

  ~TlsFront() {
    stop_.store(true);
    thread_.join();
    drop_connections();
    ::close(listener_);
    SSL_CTX_free(ctx_);
    std::remove(ca_file_.c_str());
  }

  TlsFront(const TlsFront&) = delete;
  TlsFront& operator=(const TlsFront&) = delete;

  const net::Endpoint& endpoint() const { return endpoint_; }
  const std::string& ca_file() const { return ca_file_; }

  // Handshakes that completed, and that failed (the client refused the
  // certificate, say).
  int handshakes() const { return handshakes_.load(); }
  int failed_handshakes() const { return failed_.load(); }
  // The protocols the last client offered by ALPN, as "stun.turn,h2".
  std::string offered() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return offered_;
  }

  // Closes every connection, with a close_notify to the client.
  void drop_connections() {
    std::vector<std::unique_ptr<Conn>> conns;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      conns.swap(conns_);
    }
    for (auto& c : conns) {
      c->stop.store(true);
      c->thread.join();
    }
  }

 private:
  struct Conn {
    int fd = -1;
    std::atomic<bool> stop{false};
    std::thread thread;
  };

  void make_credentials(const std::string& names) {
    EVP_PKEY* key = EVP_PKEY_Q_keygen(nullptr, nullptr, "EC", "P-256");
    X509* cert = X509_new();
    if (key == nullptr || cert == nullptr) throw std::runtime_error("TlsFront: no key");
    X509_set_version(cert, 2);
    ASN1_INTEGER_set(X509_get_serialNumber(cert), 1);
    X509_gmtime_adj(X509_getm_notBefore(cert), -3600);
    X509_gmtime_adj(X509_getm_notAfter(cert), 24 * 3600);
    X509_set_pubkey(cert, key);
    X509_NAME* name = X509_get_subject_name(cert);
    X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC,
                               reinterpret_cast<const unsigned char*>("tuushin test"), -1, -1, 0);
    X509_set_issuer_name(cert, name);
    X509V3_CTX v3;
    X509V3_set_ctx_nodb(&v3);
    X509V3_set_ctx(&v3, cert, cert, nullptr, nullptr, 0);
    X509_EXTENSION* san = X509V3_EXT_conf_nid(nullptr, &v3, NID_subject_alt_name, names.c_str());
    if (san == nullptr) throw std::runtime_error("TlsFront: bad names " + names);
    X509_add_ext(cert, san, -1);
    X509_EXTENSION_free(san);
    X509_sign(cert, key, EVP_sha256());

    ctx_ = SSL_CTX_new(TLS_server_method());
    SSL_CTX_use_certificate(ctx_, cert);
    SSL_CTX_use_PrivateKey(ctx_, key);
    SSL_CTX_set_alpn_select_cb(ctx_, &TlsFront::select_alpn, this);

    const char* tmp = std::getenv("TMPDIR");
    std::string path = std::string(tmp != nullptr && *tmp != '\0' ? tmp : "/tmp") +
                       "/tuushin-test-ca-XXXXXX";
    const int fd = ::mkstemp(path.data());
    FILE* f = fd >= 0 ? ::fdopen(fd, "w") : nullptr;
    if (f == nullptr || PEM_write_X509(f, cert) != 1) throw std::runtime_error("TlsFront: no CA file");
    std::fclose(f);
    ca_file_ = path;
    X509_free(cert);
    EVP_PKEY_free(key);
  }

  static int select_alpn(SSL*, const unsigned char** out, unsigned char* outlen,
                         const unsigned char* in, unsigned int inlen, void* arg) {
    auto* self = static_cast<TlsFront*>(arg);
    std::string offered;
    int chosen = -1;
    for (unsigned int i = 0; i < inlen && i + 1 + in[i] <= inlen; i += 1 + in[i]) {
      const std::string p(reinterpret_cast<const char*>(in + i + 1), in[i]);
      if (p == "stun.turn" && chosen < 0) chosen = static_cast<int>(i);
      offered += (offered.empty() ? "" : ",") + p;
    }
    {
      std::lock_guard<std::mutex> lock(self->mutex_);
      self->offered_ = offered;
    }
    if (chosen < 0) return SSL_TLSEXT_ERR_NOACK;
    *out = in + chosen + 1;
    *outlen = in[chosen];
    return SSL_TLSEXT_ERR_OK;
  }

  void run() {
    while (!stop_.load()) {
      pollfd p{listener_, POLLIN, 0};
      if (::poll(&p, 1, 10) <= 0) continue;
      const int fd = ::accept(listener_, nullptr, nullptr);
      if (fd < 0) continue;
      // Blocking (accepted sockets inherit O_NONBLOCK on BSD), but never
      // for long, so that a connection notices it is to stop.
      ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
      auto conn = std::make_unique<Conn>();
      conn->fd = fd;
      Conn* c = conn.get();
      conn->thread = std::thread([this, c] { serve(*c); });
      std::lock_guard<std::mutex> lock(mutex_);
      conns_.push_back(std::move(conn));
    }
  }

  static void set_timeouts(int fd) {
    const timeval t{0, 100 * 1000};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &t, sizeof t);
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &t, sizeof t);
  }

  // Sends it all, unless told to stop or the connection fails.
  static bool send_all(const Conn& c, int fd, const unsigned char* data, std::size_t size) {
    while (size > 0 && !c.stop.load()) {
      const ssize_t n = ::send(fd, data, size, 0);
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
      if (n <= 0) return false;
      data += n;
      size -= static_cast<std::size_t>(n);
    }
    return size == 0;
  }

  void serve(Conn& c) {
    set_timeouts(c.fd);
    SSL* ssl = SSL_new(ctx_);
    SSL_set_fd(ssl, c.fd);
    int back = -1;
    int r = 0;
    // A handshake that times out (SO_RCVTIMEO) is tried again until told
    // to stop.
    while ((r = SSL_accept(ssl)) != 1 && !c.stop.load() &&
           (SSL_get_error(ssl, r) == SSL_ERROR_WANT_READ ||
            SSL_get_error(ssl, r) == SSL_ERROR_WANT_WRITE)) {
    }
    if (r != 1) {
      if (!c.stop.load()) failed_.fetch_add(1);
      ERR_clear_error();
    } else {
      handshakes_.fetch_add(1);
      back = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
      if (back >= 0 && ::connect(back, static_cast<const sockaddr*>(backend_.data()),
                                 static_cast<socklen_t>(backend_.size())) == 0) {
        set_timeouts(back);
        pass(c, ssl, back);
      }
      // A close_notify, where the client is still there to take it.
      SSL_shutdown(ssl);
    }
    SSL_free(ssl);
    if (back >= 0) ::close(back);
    ::close(c.fd);
  }

  void pass(Conn& c, SSL* ssl, int back) {
    unsigned char buf[16384];
    while (!c.stop.load()) {
      pollfd p[2] = {{c.fd, POLLIN, 0}, {back, POLLIN, 0}};
      if (SSL_pending(ssl) == 0 && ::poll(p, 2, 10) <= 0) continue;
      if (SSL_pending(ssl) > 0 || (p[0].revents & (POLLIN | POLLHUP | POLLERR))) {
        const int n = SSL_read(ssl, buf, sizeof buf);
        if (n <= 0) {
          const int e = SSL_get_error(ssl, n);
          if (e == SSL_ERROR_WANT_READ || e == SSL_ERROR_WANT_WRITE) continue;
          return;
        }
        if (!send_all(c, back, buf, static_cast<std::size_t>(n))) return;
      }
      if (p[1].revents & (POLLIN | POLLHUP | POLLERR)) {
        const ssize_t n = ::recv(back, buf, sizeof buf, 0);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
        if (n <= 0) return;
        if (SSL_write(ssl, buf, static_cast<int>(n)) != n) return;
      }
    }
  }

  net::Endpoint backend_;
  net::Endpoint endpoint_;
  int listener_ = -1;
  SSL_CTX* ctx_ = nullptr;
  std::string ca_file_;
  std::atomic<bool> stop_{false};
  std::atomic<int> handshakes_{0};
  std::atomic<int> failed_{0};
  mutable std::mutex mutex_;
  std::string offered_;
  std::vector<std::unique_ptr<Conn>> conns_;
  std::thread thread_;
};

}  // namespace test
