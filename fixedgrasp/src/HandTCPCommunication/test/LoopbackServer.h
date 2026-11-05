#ifndef LINKERHAND_TEST_LOOPBACK_SERVER_H
#define LINKERHAND_TEST_LOOPBACK_SERVER_H
#include <arpa/inet.h>
#include <cerrno>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace test_tcp {
class Socket {
public:
    explicit Socket(int fd = -1) : fd(fd) {}
    ~Socket() { if (fd >= 0) ::close(fd); }
    Socket(const Socket &) = delete;
    Socket &operator=(const Socket &) = delete;
    int fd;
};
class Server {
public:
    Server() : socket(::socket(AF_INET, SOCK_STREAM, 0)) {
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (socket.fd < 0 || ::bind(socket.fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) ||
            ::listen(socket.fd, 8)) throw std::runtime_error("Cannot create loopback server");
        socklen_t size = sizeof(address);
        if (::getsockname(socket.fd, reinterpret_cast<sockaddr *>(&address), &size))
            throw std::runtime_error("Cannot obtain loopback port");
        port = ntohs(address.sin_port);
    }
    int accept() {
        pollfd pfd{socket.fd, POLLIN, 0};
        if (::poll(&pfd, 1, 3000) != 1) throw std::runtime_error("Accept deadline exceeded");
        const int fd = ::accept(socket.fd, nullptr, nullptr);
        if (fd < 0) throw std::runtime_error("Accept failed");
        timeval timeout{2, 0};
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        return fd;
    }
    Socket socket;
    int port;
};
inline bool receive(int fd, void *buffer, size_t size) {
    char *bytes = static_cast<char *>(buffer);
    for (size_t offset = 0; offset < size;) {
        const ssize_t count = ::recv(fd, bytes + offset, size - offset, 0);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) return false;
        offset += static_cast<size_t>(count);
    }
    return true;
}
inline bool send(int fd, const void *buffer, size_t size) {
    const char *bytes = static_cast<const char *>(buffer);
    for (size_t offset = 0; offset < size;) {
        const ssize_t count = ::send(fd, bytes + offset, size - offset, MSG_NOSIGNAL);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) return false;
        offset += static_cast<size_t>(count);
    }
    return true;
}
inline void sleep(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
} // namespace test_tcp
#endif
