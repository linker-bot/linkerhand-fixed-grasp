
#include "TCPCommunication_Hand.h"

void exit_sig_handler(int signum)
{
    fprintf(stderr, "Ctrl-C caught, exit process...\n");
    ROS_WARN("Ctrl-C caught, exit process...\n");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    signal(SIGINT, exit_sig_handler);

    ros::init(argc, argv, "HandCtrl_node");
    ros::NodeHandle node;

    AIMOcommunicate::TCPCommunication_Hand handCtrl;
    if (!handCtrl.Hand_TCPCommunication_connect(node)) return 1;
    ros::Rate loop_rate(30); // 设置发布频率（以每秒100次的速率发布数据）
    if (!handCtrl.InitArm()) return 1;

    while (ros::ok())
    {
        handCtrl.RightHand_pubFunction();
        handCtrl.LeftHand_pubFunction();
        handCtrl.RightGlove_pubFunction();
        handCtrl.LeftGlove_pubFunction();
        // handCtrl.show_left();
        handCtrl.grep_run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("HandCtrl_node over");
    return 0;
}
