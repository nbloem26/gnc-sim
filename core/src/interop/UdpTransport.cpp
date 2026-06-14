// gnc-sim — interop: POSIX UDP transport for the DIS leg (issue #47), opt-in via
// GNCSIM_INTEROP_UDP.
//
// Entirely outside the pure core and outside the CI test path. Compiled only when the build defines
// GNCSIM_INTEROP_UDP (so the default build pulls in no socket headers and binds no port).
#if defined(GNCSIM_INTEROP_UDP)

#include "gncsim/interop/UdpTransport.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

namespace gncsim::interop {

UdpTransport::UdpTransport(const std::string& remote_addr, std::uint16_t port, bool bind_local)
    : port_(port) {
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) throw std::runtime_error("UdpTransport: socket() failed");

  const int yes = 1;
  ::setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
  ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  in_addr addr{};
  if (::inet_pton(AF_INET, remote_addr.c_str(), &addr) != 1) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("UdpTransport: bad remote address " + remote_addr);
  }
  remote_be_ = addr.s_addr;

  if (bind_local) {
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(port_);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("UdpTransport: bind() failed");
    }
    // Non-blocking so receive() can poll.
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  }
}

UdpTransport::~UdpTransport() {
  if (fd_ >= 0) ::close(fd_);
}

void UdpTransport::send(const std::vector<std::uint8_t>& datagram) {
  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_addr.s_addr = remote_be_;
  dest.sin_port = htons(port_);
  ::sendto(fd_, datagram.data(), datagram.size(), 0, reinterpret_cast<sockaddr*>(&dest),
           sizeof(dest));
}

bool UdpTransport::receive(std::vector<std::uint8_t>& out) {
  std::uint8_t buf[2048];
  const ssize_t n = ::recv(fd_, buf, sizeof(buf), 0);
  if (n <= 0) return false;
  out.assign(buf, buf + n);
  return true;
}

}  // namespace gncsim::interop

#endif  // GNCSIM_INTEROP_UDP
