#include "TCPCommunication_Hand.h"
#include "LoopbackServer.h"
#include <gtest/gtest.h>
#include <future>

namespace AIMOcommunicate {
struct TcpTestAccess {
    static void configure(TCPCommunication_Hand &h, int port) {
        h.m_server_hand_ip = "127.0.0.1";
        h.m_server_hand_port = port;
        h.hand_connect_timeout_ms = 50;
        h.hand_io_timeout_ms = 60;
        h.hand_reconnect_delay_ms = 80;
        h.Hand_Freedom = 16;
        h.configuration_valid = true;
        h.target_poses.resize(5);
        h.pub_right_hand = ros::NodeHandle().advertise<sensor_msgs::JointState>("/tcp_test_hand_states", 1);
    }
    static void enable(TCPCommunication_Hand &h) { h.handEnable = true; }
    static void exchange(TCPCommunication_Hand &h) { h.Hand_TCPSendRecvMessage(ros::WallTimerEvent()); }
    static bool connected(TCPCommunication_Hand &h) { return h.m_hand_isConnected; }
    static bool valid(TCPCommunication_Hand &h) { return h.hand_feedback_valid; }
    static int step(TCPCommunication_Hand &h) { return h.show_step; }
    static TcpStream::Deadline retry(TCPCommunication_Hand &h) { return h.next_hand_connect; }
};
class FakeTcpHand : public TCPCommunication_Hand {
public:
    FakeTcpHand(int port) : TCPCommunication_Hand(false) { TcpTestAccess::configure(*this, port); }
    int moves = 0;
    bool armmove_j(geometry_msgs::Pose) override { ++moves; return true; }
};
using Access = TcpTestAccess;
TEST(TcpRecovery, DropsPartialFrameAndReconnectsWithoutResumingGraspSequence) {
    test_tcp::Server server;
    auto peer = std::async(std::launch::async, [&] {
        {
            test_tcp::Socket fd(server.accept());
            Hand_SendStruct command{};
            ASSERT_TRUE(test_tcp::receive(fd.fd, &command, sizeof(command)));
            Hand_RecvStruct reply{}; reply.Get_CurrentPoint_RightHand.Joint1 = 11;
            ASSERT_TRUE(test_tcp::send(fd.fd, &reply, sizeof(reply)));
            ASSERT_TRUE(test_tcp::receive(fd.fd, &command, sizeof(command)));
            reply.Get_CurrentPoint_RightHand.Joint1 = 99;
            ASSERT_TRUE(test_tcp::send(fd.fd, &reply, sizeof(reply) / 2));
        }
        {
            test_tcp::Socket fd(server.accept());
            Hand_SendStruct command{};
            ASSERT_TRUE(test_tcp::receive(fd.fd, &command, sizeof(command)));
            EXPECT_EQ(77, command.Set_Point_RightHand.Joint1);
            Hand_RecvStruct reply{}; reply.Get_CurrentPoint_RightHand.Joint1 = 22;
            const char *bytes = reinterpret_cast<const char *>(&reply);
            ASSERT_TRUE(test_tcp::send(fd.fd, bytes, 3)); test_tcp::sleep(5);
            ASSERT_TRUE(test_tcp::send(fd.fd, bytes + 3, sizeof(reply) - 3));
        }
    });
    FakeTcpHand hand(server.port);
    hand.grep_run(); EXPECT_EQ(0, hand.moves);
    Access::exchange(hand);
    ASSERT_TRUE(Access::valid(hand)); EXPECT_EQ(11, hand.HandClient_Recv.Get_CurrentPoint_RightHand.Joint1);
    Access::exchange(hand);
    EXPECT_FALSE(Access::connected(hand)); EXPECT_FALSE(Access::valid(hand));
    EXPECT_EQ(11, hand.HandClient_Recv.Get_CurrentPoint_RightHand.Joint1);
    EXPECT_EQ(9, Access::step(hand));
    const auto retry = Access::retry(hand);
    Access::exchange(hand); EXPECT_EQ(retry, Access::retry(hand)); EXPECT_FALSE(Access::connected(hand));
    hand.HandClient_Send.Set_Point_RightHand.Joint1 = 77;
    test_tcp::sleep(100); Access::exchange(hand);
    EXPECT_TRUE(Access::connected(hand)); EXPECT_TRUE(Access::valid(hand));
    EXPECT_EQ(22, hand.HandClient_Recv.Get_CurrentPoint_RightHand.Joint1);
    hand.grep_run(); EXPECT_EQ(0, hand.moves); EXPECT_EQ(9, Access::step(hand)); peer.get();
}
TEST(TcpRecovery, TimeoutRejectsDelayedResponseFromOldConnection) {
    test_tcp::Server server;
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket old(server.accept());
        Hand_SendStruct command{}; ASSERT_TRUE(test_tcp::receive(old.fd, &command, sizeof(command)));
        Hand_RecvStruct delayed{}; delayed.Get_CurrentPoint_RightHand.Joint1 = 99;
        ASSERT_TRUE(test_tcp::send(old.fd, &delayed, 2));
        test_tcp::sleep(100);
        test_tcp::send(old.fd, reinterpret_cast<const char *>(&delayed) + 2, sizeof(delayed) - 2);
        test_tcp::Socket fresh(server.accept());
        ASSERT_TRUE(test_tcp::receive(fresh.fd, &command, sizeof(command)));
        Hand_RecvStruct reply{}; reply.Get_CurrentPoint_RightHand.Joint1 = 42;
        ASSERT_TRUE(test_tcp::send(fresh.fd, &reply, sizeof(reply)));
    });
    FakeTcpHand hand(server.port);
    const auto previous = hand.HandClient_Recv.Get_CurrentPoint_RightHand.Joint1;
    Access::exchange(hand);
    EXPECT_FALSE(Access::valid(hand)); EXPECT_FALSE(Access::connected(hand));
    EXPECT_EQ(previous, hand.HandClient_Recv.Get_CurrentPoint_RightHand.Joint1);
    test_tcp::sleep(100); Access::exchange(hand);
    EXPECT_TRUE(Access::valid(hand)); EXPECT_EQ(42, hand.HandClient_Recv.Get_CurrentPoint_RightHand.Joint1);
    EXPECT_EQ(9, Access::step(hand)); peer.get();
}
TEST(TcpRecovery, FailedReconnectHonorsRetryDelay) {
    int unused_port;
    { test_tcp::Server server; unused_port = server.port; }
    FakeTcpHand hand(unused_port);
    Access::exchange(hand); EXPECT_FALSE(Access::connected(hand));
    const auto retry = Access::retry(hand);
    ASSERT_GT(retry, TcpStream::Clock::now());
    Access::exchange(hand); EXPECT_EQ(retry, Access::retry(hand));
    EXPECT_FALSE(Access::valid(hand));
}
TEST(TcpRecovery, SuppressesStatePublicationUntilCompleteFeedback) {
    test_tcp::Server server;
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket fd(server.accept()); Hand_SendStruct command{};
        ASSERT_TRUE(test_tcp::receive(fd.fd, &command, sizeof(command)));
        Hand_RecvStruct reply{}; reply.Get_CurrentPoint_RightHand.Joint1 = 12;
        ASSERT_TRUE(test_tcp::send(fd.fd, &reply, sizeof(reply)));
        ASSERT_TRUE(test_tcp::receive(fd.fd, &command, sizeof(command)));
    });
    FakeTcpHand hand(server.port);
    int received = 0;
    ros::NodeHandle node;
    auto subscriber = node.subscribe<sensor_msgs::JointState>("/tcp_test_hand_states", 10,
        boost::function<void(const sensor_msgs::JointState::ConstPtr &)>(
            [&](const sensor_msgs::JointState::ConstPtr &) { ++received; }));
    const auto until = ros::WallTime::now() + ros::WallDuration(3);
    while (subscriber.getNumPublishers() == 0 && ros::WallTime::now() < until) {
        ros::spinOnce(); test_tcp::sleep(10);
    }
    ASSERT_GT(subscriber.getNumPublishers(), 0u);
    hand.RightHand_pubFunction(); ros::spinOnce(); EXPECT_EQ(0, received);
    Access::exchange(hand); hand.RightHand_pubFunction();
    while (received == 0 && ros::WallTime::now() < until) { ros::spinOnce(); test_tcp::sleep(10); }
    ASSERT_EQ(1, received);
    Access::exchange(hand); EXPECT_FALSE(Access::valid(hand));
    hand.RightHand_pubFunction(); test_tcp::sleep(30); ros::spinOnce(); EXPECT_EQ(1, received); peer.get();
}
TEST(TcpRecovery, WallTimerExchangesWhileSimulationTimeIsPaused) {
    test_tcp::Server server;
    auto peer = std::async(std::launch::async, [&] {
        test_tcp::Socket fd(server.accept()); Hand_SendStruct command{};
        ASSERT_TRUE(test_tcp::receive(fd.fd, &command, sizeof(command)));
        Hand_RecvStruct reply{}; reply.Get_CurrentPoint_RightHand.Joint1 = 43;
        ASSERT_TRUE(test_tcp::send(fd.fd, &reply, sizeof(reply)));
        test_tcp::sleep(100);
    });
    FakeTcpHand hand(server.port); Access::enable(hand);
    ros::NodeHandle private_node("~"); private_node.setParam("hand_port", server.port);
    ASSERT_TRUE(hand.Hand_TCPCommunication_connect(ros::NodeHandle()));
    ASSERT_TRUE(ros::Time::isSimTime()); ASSERT_TRUE(ros::Time::now().isZero());
    const auto until = ros::WallTime::now() + ros::WallDuration(2);
    ros::WallRate loop_rate(200);
    while (!Access::valid(hand) && ros::WallTime::now() < until) {
        ros::spinOnce(); loop_rate.sleep();
    }
    EXPECT_TRUE(Access::valid(hand)); EXPECT_TRUE(ros::Time::now().isZero());
    EXPECT_EQ(43, hand.HandClient_Recv.Get_CurrentPoint_RightHand.Joint1);
    private_node.deleteParam("hand_port"); peer.get();
}
TEST(TcpRecovery, RejectsInvalidTimeoutConfigurationBeforeConnecting) {
    FakeTcpHand hand(20008); Access::enable(hand);
    ros::NodeHandle private_node("~");
    private_node.setParam("hand_io_timeout_ms", 0);
    EXPECT_FALSE(hand.Hand_TCPCommunication_connect(ros::NodeHandle()));
    EXPECT_FALSE(Access::connected(hand));
    private_node.deleteParam("hand_io_timeout_ms");
}
} // namespace AIMOcommunicate
int main(int argc, char **argv) {
    ros::init(argc, argv, "tcp_recovery_regression");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
