#include "TcpStream.h"

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace AIMOcommunicate {

TcpStream::~TcpStream() { close(); }

void TcpStream::close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
}

TcpStream::Result TcpStream::wait(short events, Deadline deadline) {
    for (;;) {
        const auto remaining = deadline - Clock::now();
        if (remaining <= Clock::duration::zero()) return Result::Timeout;
        // Round up so poll does not expire before the monotonic deadline.
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count();
        const int timeout = static_cast<int>(ms < 60000 ? ms + 1 : 60000);
        pollfd pfd{fd_, events, 0};
        const int ret = ::poll(&pfd, 1, timeout);
        if (ret < 0) {
            if (errno == EINTR) continue;
            return Result::Error;
        }
        if (ret == 0) continue;
        // A final readable fragment may accompany POLLHUP.
        if (pfd.revents & events) return Result::Complete;
        if (pfd.revents & POLLNVAL) return Result::Error;
        if (pfd.revents & (POLLHUP | POLLERR)) return Result::Disconnected;
    }
}

TcpStream::Result TcpStream::connectTo(const std::string &ipv4, int port, int timeout_ms) {
    close();
    if (port < 1 || port > 65535 || timeout_ms < 1) return Result::Error;
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, ipv4.c_str(), &address.sin_addr) != 1) return Result::Error;
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) return Result::Error;
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        close();
        return Result::Error;
    }
#ifdef SO_NOSIGPIPE
    const int enabled = 1;
    if (::setsockopt(fd_, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled)) < 0) {
        close();
        return Result::Error;
    }
#endif
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    if (::connect(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0)
        return Result::Complete;
    if (errno != EINPROGRESS && errno != EINTR) {
        close();
        return Result::Disconnected;
    }
    Result result = wait(POLLOUT, deadline);
    if (result == Result::Complete) {
        int error = 0;
        socklen_t length = sizeof(error);
        if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &length) < 0 || error != 0)
            result = Result::Disconnected;
    }
    if (result != Result::Complete) close();
    return result;
}

TcpStream::Result TcpStream::writeAll(const void *data, size_t size, Deadline deadline) {
    if (fd_ < 0) return Result::Disconnected;
    const char *bytes = static_cast<const char *>(data);
    size_t offset = 0;
    while (offset < size) {
        if (Clock::now() >= deadline) return Result::Timeout;
#ifdef MSG_NOSIGNAL
        const int flags = MSG_NOSIGNAL;
#else
        const int flags = 0;
#endif
        const ssize_t count = ::send(fd_, bytes + offset, size - offset, flags);
        if (count > 0) { offset += static_cast<size_t>(count); continue; }
        if (count == 0) return Result::Disconnected;
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            const auto result = wait(POLLOUT, deadline);
            if (result != Result::Complete) return result;
            continue;
        }
        return Result::Disconnected;
    }
    return Result::Complete;
}

TcpStream::Result TcpStream::readAll(void *data, size_t size, Deadline deadline) {
    if (fd_ < 0) return Result::Disconnected;
    char *bytes = static_cast<char *>(data);
    size_t offset = 0;
    while (offset < size) {
        if (Clock::now() >= deadline) return Result::Timeout;
        const ssize_t count = ::recv(fd_, bytes + offset, size - offset, 0);
        if (count > 0) { offset += static_cast<size_t>(count); continue; }
        if (count == 0) return Result::Disconnected;
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            const auto result = wait(POLLIN, deadline);
            if (result != Result::Complete) return result;
            continue;
        }
        return Result::Disconnected;
    }
    return Result::Complete;
}

} // namespace AIMOcommunicate
