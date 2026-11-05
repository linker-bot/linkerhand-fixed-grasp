#include "TcpStream.h"
#include "LoopbackServer.h"
#include <gtest/gtest.h>
#include <future>
#include <vector>
#include <csignal>
#include <pthread.h>
#include <fcntl.h>
#include <memory>

namespace AIMOcommunicate {
struct TcpStreamTestAccess { static int fd(TcpStream &s) { return s.fd_; } };
using Result = TcpStream::Result;
static TcpStream::Deadline deadline(int ms = 2000) {
    return TcpStream::Clock::now() + std::chrono::milliseconds(ms);
}
TEST(TcpStream, ReassemblesFragmentedResponse) {
    test_tcp::Server server;
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket fd(server.accept());
        char request[4]; EXPECT_TRUE(test_tcp::receive(fd.fd, request, sizeof(request)));
        EXPECT_EQ(std::string("ping"), std::string(request, 4));
        for (char byte : std::string("response")) {
            EXPECT_TRUE(test_tcp::send(fd.fd, &byte, 1)); test_tcp::sleep(2);
        }
    });
    TcpStream stream; ASSERT_EQ(Result::Complete, stream.connectTo("127.0.0.1", server.port, 200));
    char response[8]{};
    EXPECT_EQ(Result::Complete, stream.writeAll("ping", 4, deadline()));
    EXPECT_EQ(Result::Complete, stream.readAll(response, sizeof(response), deadline()));
    EXPECT_EQ(std::string("response"), std::string(response, 8)); peer.get();
}
TEST(TcpStream, ReadsCoalescedFramesWithoutConsumingNextFrame) {
    test_tcp::Server server;
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket fd(server.accept()); EXPECT_TRUE(test_tcp::send(fd.fd, "onetwo", 6));
    });
    TcpStream stream; ASSERT_EQ(Result::Complete, stream.connectTo("127.0.0.1", server.port, 200));
    char response[3];
    EXPECT_EQ(Result::Complete, stream.readAll(response, 3, deadline()));
    EXPECT_EQ(std::string("one"), std::string(response, 3));
    EXPECT_EQ(Result::Complete, stream.readAll(response, 3, deadline()));
    EXPECT_EQ(std::string("two"), std::string(response, 3)); peer.get();
}
TEST(TcpStream, CompletesShortWritesUnderBackpressure) {
    test_tcp::Server server;
    const std::vector<char> request(1024 * 1024, 'x');
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket fd(server.accept()); test_tcp::sleep(80);
        std::vector<char> received(request.size());
        EXPECT_TRUE(test_tcp::receive(fd.fd, received.data(), received.size()));
        EXPECT_EQ(request, received);
    });
    TcpStream stream; ASSERT_EQ(Result::Complete, stream.connectTo("127.0.0.1", server.port, 200));
    int size = 4096;
    ASSERT_EQ(0, ::setsockopt(TcpStreamTestAccess::fd(stream), SOL_SOCKET, SO_SNDBUF, &size, sizeof(size)));
    EXPECT_EQ(Result::Complete, stream.writeAll(request.data(), request.size(), deadline())); peer.get();
}
TEST(TcpStream, WriteTimeoutBoundsAnUnresponsivePeer) {
    test_tcp::Server server;
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket fd(server.accept()); test_tcp::sleep(200);
    });
    TcpStream stream; ASSERT_EQ(Result::Complete, stream.connectTo("127.0.0.1", server.port, 200));
    int size = 4096;
    ASSERT_EQ(0, ::setsockopt(TcpStreamTestAccess::fd(stream), SOL_SOCKET, SO_SNDBUF, &size, sizeof(size)));
    std::vector<char> request(8 * 1024 * 1024, 'x');
    const auto start = TcpStream::Clock::now();
    EXPECT_EQ(Result::Timeout, stream.writeAll(request.data(), request.size(), deadline(50)));
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(TcpStream::Clock::now() - start).count();
    EXPECT_GE(elapsed, 45); EXPECT_LT(elapsed, 180); peer.get();
}
TEST(TcpStream, ReadTimeoutDoesNotRestartForEachFragment) {
    test_tcp::Server server;
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket fd(server.accept());
        for (int i = 0; i < 8; ++i) { test_tcp::send(fd.fd, "x", 1); test_tcp::sleep(20); }
    });
    TcpStream stream; ASSERT_EQ(Result::Complete, stream.connectTo("127.0.0.1", server.port, 200));
    char response[8]; const auto start = TcpStream::Clock::now();
    EXPECT_EQ(Result::Timeout, stream.readAll(response, 8, deadline(65)));
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(TcpStream::Clock::now() - start).count();
    EXPECT_GE(elapsed, 60); EXPECT_LT(elapsed, 145); peer.get();
}
TEST(TcpStream, RequestAndResponseShareOneDeadline) {
    test_tcp::Server server;
    const std::vector<char> request(128 * 1024, 'x');
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket fd(server.accept()); test_tcp::sleep(80);
        std::vector<char> received(request.size());
        ASSERT_TRUE(test_tcp::receive(fd.fd, received.data(), received.size()));
        test_tcp::sleep(240); test_tcp::send(fd.fd, "x", 1);
    });
    TcpStream stream; ASSERT_EQ(Result::Complete, stream.connectTo("127.0.0.1", server.port, 200));
    int size = 4096;
    ASSERT_EQ(0, ::setsockopt(TcpStreamTestAccess::fd(stream), SOL_SOCKET, SO_SNDBUF, &size, sizeof(size)));
    const auto until = deadline(280);
    EXPECT_EQ(Result::Complete, stream.writeAll(request.data(), request.size(), until));
    char response;
    EXPECT_EQ(Result::Timeout, stream.readAll(&response, 1, until)); peer.get();
}
TEST(TcpStream, DetectsEofDuringFrame) {
    test_tcp::Server server;
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket fd(server.accept()); test_tcp::send(fd.fd, "abc", 3);
    });
    TcpStream stream; ASSERT_EQ(Result::Complete, stream.connectTo("127.0.0.1", server.port, 200));
    char response[8]{}; EXPECT_EQ(Result::Disconnected, stream.readAll(response, 8, deadline())); peer.get();
}
TEST(TcpStream, PeerResetDoesNotTerminateProcessWithSigpipe) {
    test_tcp::Server server;
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket fd(server.accept()); linger reset{1, 0};
        EXPECT_EQ(0, ::setsockopt(fd.fd, SOL_SOCKET, SO_LINGER, &reset, sizeof(reset)));
    });
    TcpStream stream; ASSERT_EQ(Result::Complete, stream.connectTo("127.0.0.1", server.port, 200)); peer.get();
    char response;
    EXPECT_EQ(Result::Disconnected, stream.readAll(&response, 1, deadline()));
    EXPECT_EQ(Result::Disconnected, stream.writeAll("x", 1, deadline()));
}
static void signalHandler(int) {}
TEST(TcpStream, RetriesInterruptedPollWithoutExtendingDeadline) {
    struct sigaction action{}, previous{}; action.sa_handler = signalHandler;
    sigemptyset(&action.sa_mask); ASSERT_EQ(0, sigaction(SIGUSR1, &action, &previous));
    test_tcp::Server server; const auto target = pthread_self();
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket fd(server.accept());
        for (int i = 0; i < 8; ++i) { test_tcp::sleep(10); pthread_kill(target, SIGUSR1); }
    });
    TcpStream stream; ASSERT_EQ(Result::Complete, stream.connectTo("127.0.0.1", server.port, 200));
    char response; const auto start = TcpStream::Clock::now();
    EXPECT_EQ(Result::Timeout, stream.readAll(&response, 1, deadline(50)));
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(TcpStream::Clock::now() - start).count(), 130);
    peer.get(); sigaction(SIGUSR1, &previous, nullptr);
}
TEST(TcpStream, ConnectTimeoutBoundsAFullAcceptQueue) {
    test_tcp::Server server;
    std::vector<std::unique_ptr<test_tcp::Socket>> pending;
    for (int i = 0; i < 24; ++i) {
        pending.emplace_back(new test_tcp::Socket(::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)));
        sockaddr_in address{}; address.sin_family = AF_INET;
        address.sin_port = htons(server.port); address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        ::connect(pending.back()->fd, reinterpret_cast<sockaddr *>(&address), sizeof(address));
    }
    TcpStream stream; const auto start = TcpStream::Clock::now();
    EXPECT_EQ(Result::Timeout, stream.connectTo("127.0.0.1", server.port, 50));
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(TcpStream::Clock::now() - start).count(), 180);
    EXPECT_FALSE(stream.connected());
}
TEST(TcpStream, RejectsInvalidEndpointsAndReportsConnectionRefused) {
    TcpStream stream;
    EXPECT_EQ(Result::Error, stream.connectTo("not-an-ip", 1, 20));
    EXPECT_EQ(Result::Error, stream.connectTo("127.0.0.1", 0, 20));
    EXPECT_EQ(Result::Error, stream.connectTo("127.0.0.1", 1, 0));
    int unused_port;
    { test_tcp::Server server; unused_port = server.port; }
    EXPECT_EQ(Result::Disconnected, stream.connectTo("127.0.0.1", unused_port, 50));
    EXPECT_FALSE(stream.connected());
}
} // namespace AIMOcommunicate

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
