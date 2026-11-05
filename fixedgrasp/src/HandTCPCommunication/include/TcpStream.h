#ifndef LINKERHAND_TCP_STREAM_H
#define LINKERHAND_TCP_STREAM_H

#include <chrono>
#include <cstddef>
#include <string>

namespace AIMOcommunicate {

// Fixed-size protocol frames over a nonblocking, exclusively owned socket.
// The caller must discard the connection after any incomplete transfer.
class TcpStream {
public:
    enum class Result { Complete, Timeout, Disconnected, Error };
    using Clock = std::chrono::steady_clock;
    using Deadline = Clock::time_point;

    TcpStream() = default;
    ~TcpStream();
    TcpStream(const TcpStream &) = delete;
    TcpStream &operator=(const TcpStream &) = delete;

    Result connectTo(const std::string &ipv4, int port, int timeout_ms);
    Result writeAll(const void *data, size_t size, Deadline deadline);
    Result readAll(void *data, size_t size, Deadline deadline);
    void close();
    bool connected() const { return fd_ >= 0; }

private:
    Result wait(short events, Deadline deadline);
    friend struct TcpStreamTestAccess;
    int fd_ = -1;
};

} // namespace AIMOcommunicate
#endif
