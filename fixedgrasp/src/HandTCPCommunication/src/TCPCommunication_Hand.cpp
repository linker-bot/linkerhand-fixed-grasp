
/*
  通讯处理类：Json处理数据 机械手
*/
#include "TCPCommunication_Hand.h"

void MCallback(CallbackData data) {}

namespace AIMOcommunicate
{
    TCPCommunication_Hand::TCPCommunication_Hand(bool initialize_hardware)
    {
        if (!initialize_hardware) return;
        Hand_TCPCommunication_Json_Launch_Deal();
        if (handEnable == true)
        {
            Hand_InitDataSend();
            GetHandEyeCalibrationResultX_EyeInHand();
        }
        grep_init();
    }
    TCPCommunication_Hand::~TCPCommunication_Hand()
    {
        Hand_TCPCommunication_stop();
        Realseaemsocket();
        delete m_pApi;
    }
    void TCPCommunication_Hand::grep_init()
    {
    }
    void TCPCommunication_Hand::grep_run()
    {
        if (!configuration_valid || !m_hand_isConnected || !hand_feedback_valid) return;
        if (target_poses.size() < 5)
        {
            ROS_ERROR_THROTTLE(5, "Five configured arm poses are required");
            return;
        }
        show_count++;
        if (show_count >= show_count_obj)
        {
            show_count = 0;
            switch (show_step)
            {
            case 0: // 到达识别位置
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;
                if (!armmove_j(target_poses[0]))
                {
                    ROS_ERROR("Initial arm move failed; grasp sequence stopped");
                    show_step = 9;
                    break;
                }
                if_complite = false;
                get_Visual_pub.publish(Visual_msg);
                show_step = 1;
                show_count_obj = 50;
                break;
            }
            case 1: // 分析识别结果，决定本次抓取目标，策略为：按顺序检索任务表中的物品及其次数，考察以下指标按优先级抓取，如果次数超过3次或者被标记为成功，则继续考察
            {
                if (if_complite)
                {
                    for (auto it = obj_list.begin(); it != obj_list.end();) // 遍历列表，删除次数超限的物品
                    {
                        if (it->count >= 3)
                        {
                            it = obj_list.erase(it);
                        }
                        else
                        {
                            ++it;
                        }
                    }
                    if (obj_list.empty()) // 列表为空则完成，去执行收尾动作
                    {
                        show_step = 8;
                        break;
                    }
                    else
                    {
                        Move_pose.x = obj_list[0].x;
                        Move_pose.y = obj_list[0].y;
                        Move_pose.z = obj_list[0].z;
                        Move_pose.rx = obj_list[0].rx;
                        Move_pose.ry = obj_list[0].ry;
                        Move_pose.rz = obj_list[0].rz;

                        pass_obj.ID = obj_list[0].ID;
                    }
                    obj_list[0].count += 1;
                    show_step++;
                    // sleep(5);
                    show_count_obj = 50;
                }
                break;
            }
            case 2: // 机械臂到达引导位置
            {
                if (pass_obj.ID < 1 ||
                    static_cast<size_t>(pass_obj.ID) > tools_farmeName.size() ||
                    !Change_Tool_Frame(pass_obj.ID))
                {
                    ROS_ERROR("Tool frame change failed; grasp sequence stopped");
                    show_step = 9;
                    break;
                }
                show_step++;
                show_count_obj = 1;
                break;
            }
            case 3: // 机械臂到达抓取位置
            {
                if (!if_send)
                {
                    if (armmovej_p(Move_pose))
                    {
                        show_count_obj = 50;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        ROS_ERROR("Arm move failed; grasp sequence stopped");
                        show_step = 9;
                    }
                }
                break;
            }
            case 4: // 抓取
            {
                Hand_CrawlPoses(pass_obj.ID);
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 5: // 机械臂到达托盘引导位置
            {
                if (pass_obj.ID == 3 || pass_obj.ID == 2 || pass_obj.ID == 4) // 放到B托盘
                {
                    if (!if_send)
                    {
                        if (armmove_j(target_poses[1]))
                        {
                            show_count_obj = 10;
                            if_send = false;
                            show_step++;
                        }
                        else
                        {
                            ROS_ERROR("Arm move failed; grasp sequence stopped");
                            show_step = 9;
                        }
                    }
                }
                else // 放到A托盘
                {
                    if (!if_send)
                    {
                        if (armmove_j(target_poses[3]))
                        {
                            show_count_obj = 10;
                            if_send = false;
                            show_step++;
                        }
                        else
                        {
                            ROS_ERROR("Arm move failed; grasp sequence stopped");
                            show_step = 9;
                        }
                    }
                }
                break;
            }
            case 6: // 到达放置位置
            {
                if (pass_obj.ID == 3 || pass_obj.ID == 2 || pass_obj.ID == 4) // 放到B托盘
                {
                    if (!if_send)
                    {
                        if (armmove_j(target_poses[2]))
                        {
                            show_count_obj = 10;
                            if_send = false;
                            show_step++;
                        }
                        else
                        {
                            ROS_ERROR("Arm move failed; grasp sequence stopped");
                            show_step = 9;
                        }
                    }
                }
                else // 放到A托盘
                {
                    if (!if_send)
                    {
                        if (armmove_j(target_poses[4]))
                        {
                            show_count_obj = 10;
                            if_send = false;
                            show_step++;
                        }
                        else
                        {
                            ROS_ERROR("Arm move failed; grasp sequence stopped");
                            show_step = 9;
                        }
                    }
                }
                break;
            }
            case 7: // 放物品
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;
                if (!obj_list.empty()) obj_list.erase(obj_list.begin());
                show_step = obj_list.empty() ? 8 : 1;
                show_count_obj = 10;
                break;
            }
            case 8: // 回到完成状态
            {
                show_step++;
                show_count_obj = 10;
                break;
            }
            }
        }
    }
    void TCPCommunication_Hand::show_left()
    {
        show_count++;
        if (show_count >= show_count_obj)
        {
            show_count = 0;
            // std::cout << "hello" << if_complite << std::endl;
            switch (show_step)
            {
            case 0: // 手张开，到拍照位置
            {
                if (if_complite)
                {
                    // // 转换
                    Pose camera_point;
                    camera_point.position.x = object_data[0].cen_x;
                    camera_point.position.y = object_data[0].cen_y;
                    camera_point.position.z = object_data[0].cen_z;
                    camera_point.euler.rx = 0;
                    camera_point.euler.ry = 0;
                    camera_point.euler.rz = 0;
                    std::cout
                        << "camera pose" << " x = " << object_data[0].cen_x << " y = " << object_data[0].cen_y << " z = " << object_data[0].cen_z << std::endl;
                    Pose robot;
                    robot.position.x = -0.382;
                    robot.position.y = 0;
                    robot.position.z = 0.496;
                    robot.euler.rx = 3.142;
                    robot.euler.ry = 1.571;
                    robot.euler.rz = 0;
                    Pose robot_zh;
                    // robot_zh = coorTrans.ReadX_CloudPointToBaseOXYZ_EyeInHand(camera_point, robot, Calibration_Result_EyeInHand);

                    robot_zh = coorTrans.GetGraspPose(camera_point, robot, Calibration_Result_EyeInHand);
                    std::cout
                        << "Calcaulate: " << " x = " << robot_zh.position.x << " y = " << robot_zh.position.y << " z = " << robot_zh.position.z
                        << " rx = " << robot_zh.euler.rx << " ry = " << robot_zh.euler.ry << " rz = " << robot_zh.euler.rz << std::endl;
                    // if_complite = false;
                }

                // HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                // HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                // HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                // HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                // HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                // HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                // HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                // HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                // HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                // HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                // HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                // HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                // HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                // HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                // HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                // HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;

                // show_count_obj = 50;

                // if (!if_send)
                // {
                //     if (armmove_j(target_poses[0])) // 到开始位置
                //     {
                //         if_complite = false;
                //         arm_reached_pub.publish(msg);
                //         show_count_obj = 1;
                //         if_send = false;
                //         show_step = 41;
                //     }
                //     else
                //     {
                //         if_send = true;
                //     }
                // }
                break;
            }
            // 杯子
            case 1: // 杯子手初始姿态
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 100;
                // show_step++;
                show_count_obj = 10;
                break;
            }
            case 2: // 到引导位置
            {
                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }
                break;
            }
            case 3: // 到抓取位置
            {
                if (!if_send)
                {
                    if (armmovej_p(movej_p_pose))
                    {
                        if_complite = false;
                        show_count_obj = 1;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }

                break;
            }
            case 4: // 抓杯子
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 120;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 100;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 100;
                show_step++;
                show_count_obj = 40;
                break;
            }
            case 5: // 到引导位置
            {
                show_step++;
                show_count_obj = 40;
                break;
            }
            case 6: // 到B托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[1]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 7: // 到B托盘位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[2]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 8: // 放下物品
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;
                show_step++;
                show_count_obj = 20;
                break;
            }
            case 9: // 到B托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[1])) // 到开始位置
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = true;
                        show_step++;
                    }
                    if_send = true;
                }
                break;
            }
            case 10: // 到拍照位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[0])) // 到开始位置
                    {
                        if_complite = false;
                        arm_reached_pub.publish(msg);
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            // 丝绸
            case 11: // 丝绸手初始姿态
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 150;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 12: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 13: // 到抓取位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 14: // 抓丝绸
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 110;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 110;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 110;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 30;
                break;
            }
            case 15: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 16: // 到B托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[1]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 17: // 到B托盘位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[2]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 18: // 放下物品
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 19: // 到B托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[1])) // 到开始位置
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = true;
                        show_step++;
                    }
                    if_send = true;
                }
                break;
            }
            case 20: // 到拍照位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[0])) // 到开始位置
                    {
                        if_complite = false;
                        arm_reached_pub.publish(msg);
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            // 改锥
            case 21: // 改锥手初始姿态
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 150;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 22: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 23: // 到抓取位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 24: // 抓改锥
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 110;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 110;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 20;
                break;
            }
            case 25: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 26: // 到A托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[3]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 27: // 到A托盘位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[4]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 28: // 放下物品
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 29: // 到A托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[3])) // 到开始位置
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = true;
                        show_step++;
                    }
                    if_send = true;
                }
                break;
            }
            case 30: // 到拍照位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[0])) // 到开始位置
                    {
                        if_complite = false;
                        arm_reached_pub.publish(msg);
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            // 金属球
            case 31: // 金属球手初始姿态
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 200;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 200;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 200;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 32: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 33: // 到抓取位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 34: // 抓金属球
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 100;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 20;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 55;
                break;
            }
            case 35: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 36: // 到A托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[3]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 37: // 到A托盘位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[4]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 38: // 放下物品
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 39: // 到A托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[3])) // 到开始位置
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = true;
                        show_step++;
                    }
                    if_send = true;
                }
                break;
            }
            case 40: // 到拍照位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[0])) // 到开始位置
                    {
                        if_complite = false;
                        arm_reached_pub.publish(msg);
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            // 电池
            case 41: // 手初始姿态
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 200;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 200;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 200;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 150;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 42: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 43: // 到抓取位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 44: // 抓
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 40;
                break;
            }
            case 45: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 46: // 到A托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[3]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 47: // 到A托盘位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[4]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 48: // 放下物品
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 49: // 到A托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[3])) // 到开始位置
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = true;
                        show_step++;
                    }
                    if_send = true;
                }
                break;
            }
            case 50: // 到拍照位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[0])) // 到开始位置
                    {
                        if_complite = false;
                        arm_reached_pub.publish(msg);
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            // 鸡蛋
            case 51: // 手初始姿态
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 200;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 52: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 53: // 到抓取位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 54: // 抓
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 150;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 100;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 200;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 30;
                break;
            }
            case 55: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 56: // 到B托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[1]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 57: // 到B托盘位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[2]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 58: // 放下物品
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 59: // 到B托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[1])) // 到开始位置
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = true;
                        show_step++;
                    }
                    if_send = true;
                }
                break;
            }
            case 60: // 到拍照位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[0])) // 到开始位置
                    {
                        if_complite = false;
                        arm_reached_pub.publish(msg);
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            // 螺栓
            case 61: // 螺栓手初始位置
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 150;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 150;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 150;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 150;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 80;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 50;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 160;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 62: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 63: // 到抓取位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 64: // 抓螺栓
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 130;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 115;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 130;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 90;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 80;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 30;
                break;
            }
            case 65: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 66: // 到A托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[3]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 67: // 到A托盘位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[4]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 68: // 放下物品
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 69: // 到A托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[3])) // 到开始位置
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = true;
                        show_step++;
                    }
                    if_send = true;
                }
                break;
            }
            case 70: // 到拍照位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[0])) // 到开始位置
                    {
                        if_complite = false;
                        arm_reached_pub.publish(msg);
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            // 药丸
            case 71: // 药丸手初始姿态
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 180;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 0;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 0;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 115;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 90;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 0;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 0;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 72: // 到引导位置
            {
                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }
                break;
            }
            case 73: // 到抓取位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 74: // 抓胶囊
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 0;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 0;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 115;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 90;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 0;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 0;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 75: // 到引导位置
            {
                if (!if_send)
                {
                    if_complite = false;
                    pose_pub.publish(target_poses[current_index]);
                    show_count_obj = 1;
                    if_send = true;
                }

                if (if_complite)
                {
                    if_complite = false;
                    current_index++;
                    show_step++;
                    show_count_obj = 1;
                    if_send = false;
                }

                break;
            }
            case 76: // 到B托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[1]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 77: // 到B托盘位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[2]))
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }
            case 78: // 放下物品
            {
                HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;
                show_step++;
                show_count_obj = 10;
                break;
            }
            case 79: // 到B托盘引导位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[1])) // 到开始位置
                    {
                        if_complite = false;
                        show_count_obj = 10;
                        if_send = true;
                        show_step++;
                    }
                    if_send = true;
                }
                break;
            }
            case 80: // 到拍照位置
            {
                if (!if_send)
                {
                    if (armmove_j(target_poses[0])) // 到开始位置
                    {
                        if_complite = false;
                        arm_reached_pub.publish(msg);
                        show_count_obj = 10;
                        if_send = false;
                        show_step++;
                    }
                    else
                    {
                        if_send = true;
                    }
                }
                break;
            }

                // 码枪
                // case 73: // 手初始姿态
                // {
                //     HandClient_Send.Set_Point_LeftHand.Joint3 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint4 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint5 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

                //     HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_tip = 250;

                //     HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                //     HandClient_Send.Set_Point_LeftHand.Joint2_tip = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_tip = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint4_tip = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint5_tip = 250;
                //     show_step++;
                //     show_count_obj = 10;
                //     break;
                // }
                // case 74: // 到引导位置
                // {
                //     if (!if_send)
                //     {
                //         if_complite = false;
                //         pose_pub.publish(target_poses[current_index]);
                //         show_count_obj = 1;
                //         if_send = true;
                //     }

                //     if (if_complite)
                //     {
                //         if_complite = false;
                //         current_index++;
                //         show_step++;
                //         show_count_obj = 1;
                //         if_send = false;
                //     }

                //     break;
                // }
                // case 75: // 到抓取位置
                // {
                //     if (!if_send)
                //     {
                //         if_complite = false;
                //         pose_pub.publish(target_poses[current_index]);
                //         show_count_obj = 1;
                //         if_send = true;
                //     }

                //     if (if_complite)
                //     {
                //         if_complite = false;
                //         current_index++;
                //         show_step++;
                //         show_count_obj = 1;
                //         if_send = false;
                //     }

                //     break;
                // }
                // case 76: // 抓
                // {
                //     HandClient_Send.Set_Point_LeftHand.Joint3 = 120;
                //     HandClient_Send.Set_Point_LeftHand.Joint4 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint5 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

                //     HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_tip = 100;

                //     HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                //     HandClient_Send.Set_Point_LeftHand.Joint2_tip = 90;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_tip = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint4_tip = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                //     show_step++;
                //     show_count_obj = 50;
                //     break;
                // }
                // case 77: // 到引导位置
                // {
                //     if (!if_send)
                //     {
                //         if_complite = false;
                //         pose_pub.publish(target_poses[current_index]);
                //         show_count_obj = 1;
                //         if_send = true;
                //     }

                //     if (if_complite)
                //     {
                //         if_complite = false;
                //         current_index++;
                //         show_step++;
                //         show_count_obj = 1;
                //         if_send = false;
                //     }

                //     break;
                // }
                // case 78: // 到扫码位置
                // {
                //     if (!if_send)
                //     {
                //         if_complite = false;
                //         pose_pub.publish(target_poses[current_index]);
                //         show_count_obj = 1;
                //         if_send = true;
                //     }

                //     if (if_complite)
                //     {
                //         if_complite = false;
                //         current_index++;
                //         show_step++;
                //         show_count_obj = 1;
                //         if_send = false;
                //     }

                //     break;
                // }
                // case 79: // 扫码
                // {
                //     HandClient_Send.Set_Point_LeftHand.Joint3 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint4 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint5 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

                //     HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_tip = 100;

                //     HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                //     HandClient_Send.Set_Point_LeftHand.Joint2_tip = 50;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_tip = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint4_tip = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                //     show_step++;
                //     show_count_obj = 50;
                //     break;
                // }
                // case 80:
                // {
                //     HandClient_Send.Set_Point_LeftHand.Joint3 = 120;
                //     HandClient_Send.Set_Point_LeftHand.Joint4 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint5 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

                //     HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_tip = 100;

                //     HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                //     HandClient_Send.Set_Point_LeftHand.Joint2_tip = 50;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_tip = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint4_tip = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                //     show_step++;
                //     show_count_obj = 50;
                //     break;
                // }
                // case 81: // 扫码
                // {
                //     HandClient_Send.Set_Point_LeftHand.Joint3 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint4 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint5 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

                //     HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_tip = 100;

                //     HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                //     HandClient_Send.Set_Point_LeftHand.Joint2_tip = 50;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_tip = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint4_tip = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                //     show_step++;
                //     show_count_obj = 50;
                //     break;
                // }
                // case 82:
                // {
                //     HandClient_Send.Set_Point_LeftHand.Joint3 = 120;
                //     HandClient_Send.Set_Point_LeftHand.Joint4 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint5 = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

                //     HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint2 = 250;
                //     HandClient_Send.Set_Point_LeftHand.Joint1_tip = 100;

                //     HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
                //     HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

                //     HandClient_Send.Set_Point_LeftHand.Joint2_tip = 50;
                //     HandClient_Send.Set_Point_LeftHand.Joint3_tip = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint4_tip = 0;
                //     HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
                //     show_step++;
                //     show_count_obj = 50;
                //     break;
                // }
            }
        }
    }
    void TCPCommunication_Hand::show_right()
    {
        show_count++;
        if (show_count >= show_count_obj)
        {
            show_count = 0;
            switch (show_step)
            {
            case 0:
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 250;
                HandClient_Send.Set_Point_RightHand.Joint4 = 250;
                HandClient_Send.Set_Point_RightHand.Joint5 = 250;
                HandClient_Send.Set_Point_RightHand.Joint6 = 250;

                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                HandClient_Send.Set_Point_RightHand.Joint7 = 128;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_RightHand.Joint8 = 128;
                HandClient_Send.Set_Point_RightHand.Joint9 = 128;

                HandClient_Send.Set_Point_RightHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;
                show_step++;
                show_count_obj = 100;
                break;
            }
            case 1: // 收小指与无名指
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint5 = 0;
                HandClient_Send.Set_Point_RightHand.Joint6 = 0;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 0;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 0;

                show_step++;
                show_count_obj = 10;
                break;
            }
            case 2: // 将拇指搭到小指与无名指上面
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 180;
                HandClient_Send.Set_Point_RightHand.Joint2 = 200;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 0;

                show_step++;
                show_count_obj = 30;
                break;
            }
            case 3: // 食指和中指向一侧倾斜
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 200;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 200;

                show_step++;
                show_count_obj = 10;
                break;
            }
            case 4: // 另一侧
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 50;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 50;

                show_step++;
                show_count_obj = 10;
                break;
            }
            case 5: // 两支回中
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 128;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 128;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 6: // 食指和中指做Y
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 50;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 200;

                show_step++;
                show_count_obj = 10;
                break;
            }
            case 7: // 收Y
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 128;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 128;

                show_step++;
                show_count_obj = 10;
                break;
            }
            case 8: // 重复一遍
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 50;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 200;

                show_step++;
                show_count_obj = 10;
                break;
            }
            case 9:
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 128;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 128;

                show_step++;
                show_count_obj = 10;
                break;
            }
            case 10: // 中指和食指弯曲伸直交替两遍
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 100;
                HandClient_Send.Set_Point_RightHand.Joint4 = 100;
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 100;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 11:
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 250;
                HandClient_Send.Set_Point_RightHand.Joint4 = 250;
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 12:
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 100;
                HandClient_Send.Set_Point_RightHand.Joint4 = 100;
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 100;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 100;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 13:
            {
                // HandClient_Send.Set_Point_RightHand.Joint3 = 250;
                // HandClient_Send.Set_Point_RightHand.Joint4 = 250;
                // HandClient_Send.Set_Point_RightHand.Joint2_tip = 250;
                // HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint3 = 250;
                HandClient_Send.Set_Point_RightHand.Joint4 = 250;
                HandClient_Send.Set_Point_RightHand.Joint5 = 250;
                HandClient_Send.Set_Point_RightHand.Joint6 = 250;
                HandClient_Send.Set_Point_RightHand.Joint7 = 128;
                HandClient_Send.Set_Point_RightHand.Joint8 = 128;
                HandClient_Send.Set_Point_RightHand.Joint9 = 128;

                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;

                show_step++;
                show_count_obj = 30;
                break;
            }
            case 14: // 蜷曲拇指
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 150;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 0;

                show_step++;
                show_count_obj = 40;
                break;
            }
            case 15: // 拇指收于掌内
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 5;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 0;

                show_step++;
                show_count_obj = 10;
                break;
            }
            case 16: // 收4指
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 100;
                HandClient_Send.Set_Point_RightHand.Joint4 = 100;
                HandClient_Send.Set_Point_RightHand.Joint5 = 100;
                HandClient_Send.Set_Point_RightHand.Joint6 = 100;
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 10;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 10;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 10;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 10;

                show_step++;
                show_count_obj = 30;
                break;
            }
            case 17: // 依次放开4指和拇指
            {
                HandClient_Send.Set_Point_RightHand.Joint6 = 250;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 18:
            {
                HandClient_Send.Set_Point_RightHand.Joint5 = 250;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 19:
            {
                HandClient_Send.Set_Point_RightHand.Joint4 = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 20:
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 250;
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 21:
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 22: // 并拢拇指
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 10;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 23: // 反转拇指指掌心
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 0;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 10;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 40;
                break;
            }
            case 24: // 分两步回到初始位置
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 0;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 30;
                break;
            }
            case 25:
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 50;
                break;
            }
            case 26:
            {
                HandClient_Send.Set_Point_RightHand.Joint7 = 200;
                HandClient_Send.Set_Point_RightHand.Joint8 = 200;
                HandClient_Send.Set_Point_RightHand.Joint9 = 200;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 200;

                show_step++;
                show_count_obj = 10;
                break;
            }
            case 27:
            {
                HandClient_Send.Set_Point_RightHand.Joint7 = 80;
                HandClient_Send.Set_Point_RightHand.Joint8 = 80;
                HandClient_Send.Set_Point_RightHand.Joint9 = 80;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 80;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 28:
            {
                HandClient_Send.Set_Point_RightHand.Joint7 = 128;
                HandClient_Send.Set_Point_RightHand.Joint8 = 128;
                HandClient_Send.Set_Point_RightHand.Joint9 = 128;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 128;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 29: // 依次蜷曲4小指
            {
                // HandClient_Send.Set_Point_RightHand.Joint3 = 0;
                // HandClient_Send.Set_Point_RightHand.Joint4 = 0;
                // HandClient_Send.Set_Point_RightHand.Joint5 = 0;
                // HandClient_Send.Set_Point_RightHand.Joint6 = 0;
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 0;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;

                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 30: // 蜷曲4指
            {
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 0;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 0;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;

                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 31: // 蜷曲4指
            {
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 0;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 0;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 0;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;

                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 32: // 蜷曲4指
            {
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 0;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 0;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 0;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 0;

                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 33: // 依次蜷曲4小指
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 0;
                HandClient_Send.Set_Point_RightHand.Joint4 = 250;
                HandClient_Send.Set_Point_RightHand.Joint5 = 250;
                HandClient_Send.Set_Point_RightHand.Joint6 = 250;

                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 34: // 依次蜷曲4小指
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 0;
                HandClient_Send.Set_Point_RightHand.Joint4 = 0;
                HandClient_Send.Set_Point_RightHand.Joint5 = 250;
                HandClient_Send.Set_Point_RightHand.Joint6 = 250;

                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 35: // 依次蜷曲4小指
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 0;
                HandClient_Send.Set_Point_RightHand.Joint4 = 0;
                HandClient_Send.Set_Point_RightHand.Joint5 = 0;
                HandClient_Send.Set_Point_RightHand.Joint6 = 250;

                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 36: // 依次蜷曲4小指
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 0;
                HandClient_Send.Set_Point_RightHand.Joint4 = 0;
                HandClient_Send.Set_Point_RightHand.Joint5 = 0;
                HandClient_Send.Set_Point_RightHand.Joint6 = 0;

                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 37: // 蜷曲拇指
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 0;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 200;

                show_step++;
                show_count_obj = 40;
                break;
            }
            case 38: // 打开食指和小指
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 40;
                break;
            }
            case 39: // 打开食指和小指
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 250;
                HandClient_Send.Set_Point_RightHand.Joint6 = 250;
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;

                show_step++;
                show_count_obj = 30;
                break;
            }
            case 40: // 将拇指搭上666
            {

                HandClient_Send.Set_Point_RightHand.Joint1 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 200;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 100;

                show_step++;
                show_count_obj = 40;
                break;
            }
            case 41: // 左右动手指
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 80;
                HandClient_Send.Set_Point_RightHand.Joint9 = 200;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 42: // 左右动手指
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 200;
                HandClient_Send.Set_Point_RightHand.Joint9 = 80;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 43: // 左右动手指
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 80;
                HandClient_Send.Set_Point_RightHand.Joint9 = 200;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 44: // 左右动手指
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 200;
                HandClient_Send.Set_Point_RightHand.Joint9 = 80;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 45: // 左右动手指
            {

                HandClient_Send.Set_Point_RightHand.Joint7 = 128;
                HandClient_Send.Set_Point_RightHand.Joint9 = 128;

                show_step++;
                show_count_obj = 15;
                break;
            }
            case 46:
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint3 = 250;
                HandClient_Send.Set_Point_RightHand.Joint4 = 250;
                HandClient_Send.Set_Point_RightHand.Joint5 = 250;
                HandClient_Send.Set_Point_RightHand.Joint6 = 250;
                HandClient_Send.Set_Point_RightHand.Joint7 = 128;
                HandClient_Send.Set_Point_RightHand.Joint8 = 128;
                HandClient_Send.Set_Point_RightHand.Joint9 = 128;

                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;
                show_step++;
                show_count_obj = 50;
                break;
            }
            case 47: // 拇指和食指捏
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 20;
                HandClient_Send.Set_Point_RightHand.Joint4 = 250;
                HandClient_Send.Set_Point_RightHand.Joint5 = 250;
                HandClient_Send.Set_Point_RightHand.Joint6 = 250;

                HandClient_Send.Set_Point_RightHand.Joint1 = 70;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 40;
                break;
            }
            case 48:
            {
                HandClient_Send.Set_Point_RightHand.Joint3 = 250;
                HandClient_Send.Set_Point_RightHand.Joint4 = 20;
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 49: // 拇指和中指捏
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 70;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 170;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 50:
            {
                HandClient_Send.Set_Point_RightHand.Joint4 = 250;
                HandClient_Send.Set_Point_RightHand.Joint5 = 20;
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 51: // 拇指和无名指捏
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 70;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 130;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 52:
            {
                HandClient_Send.Set_Point_RightHand.Joint5 = 250;
                HandClient_Send.Set_Point_RightHand.Joint6 = 20;
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 53: // 拇指和小指捏
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 70;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 90;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 54: // 拇指和小指掐
            {
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 50;

                HandClient_Send.Set_Point_RightHand.Joint6 = 130;
                HandClient_Send.Set_Point_RightHand.Joint1 = 180;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 90;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 100;

                show_step++;
                show_count_obj = 30;
                break;
            }
            case 55:
            {
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 50;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;

                HandClient_Send.Set_Point_RightHand.Joint6 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 130;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 56: // 拇指和无名指掐
            {
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 50;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;

                HandClient_Send.Set_Point_RightHand.Joint5 = 130;
                HandClient_Send.Set_Point_RightHand.Joint1 = 180;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 130;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 100;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 57:
            {
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 50;

                HandClient_Send.Set_Point_RightHand.Joint5 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 180;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 58: // 拇指和中指掐
            {
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 35;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;

                HandClient_Send.Set_Point_RightHand.Joint4 = 110;
                HandClient_Send.Set_Point_RightHand.Joint1 = 180;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 180;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 100;

                show_step++;
                show_count_obj = 25;
                break;
            }
            case 59:
            {
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 50;

                HandClient_Send.Set_Point_RightHand.Joint4 = 250;
                HandClient_Send.Set_Point_RightHand.Joint3 = 130;
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;

                show_step++;
                show_count_obj = 20;
                break;
            }
            case 60: // 拇指和食指掐
            {
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 35;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;

                HandClient_Send.Set_Point_RightHand.Joint3 = 150;
                HandClient_Send.Set_Point_RightHand.Joint1 = 180;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 220;
                HandClient_Send.Set_Point_RightHand.Joint2 = 100;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 100;

                show_step++;
                show_count_obj = 30;
                break;
            }
            case 61:
            {
                HandClient_Send.Set_Point_RightHand.Joint1 = 250;
                HandClient_Send.Set_Point_RightHand.Joint1_rotation = 250;
                HandClient_Send.Set_Point_RightHand.Joint3 = 250;
                HandClient_Send.Set_Point_RightHand.Joint4 = 250;
                HandClient_Send.Set_Point_RightHand.Joint5 = 250;
                HandClient_Send.Set_Point_RightHand.Joint6 = 250;
                HandClient_Send.Set_Point_RightHand.Joint7 = 128;
                HandClient_Send.Set_Point_RightHand.Joint8 = 128;
                HandClient_Send.Set_Point_RightHand.Joint9 = 128;

                HandClient_Send.Set_Point_RightHand.Joint2 = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_raw = 128;
                HandClient_Send.Set_Point_RightHand.Joint1_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint2_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint3_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint4_tip = 250;
                HandClient_Send.Set_Point_RightHand.Joint5_tip = 250;
                show_step++;
                show_count_obj = 50;
                break;
            }
            }
        }
    }
    void TCPCommunication_Hand::Hand_TCPCommunication_stop(void)
    {
        m_hand_timer.stop();
        hand_stream.close();
        m_hand_isConnected = false;
        hand_feedback_valid = false;
    }
    void TCPCommunication_Hand::Hand_TCPCommunication_Json_Launch_Deal(void)
    {
        m_server_hand_ip = "192.168.1.40";
        m_hand_tcp_nh.getParam("arm/hand_ip", m_server_hand_ip);
        m_server_hand_port = 20008;

        m_hand_tcp_nh.param<bool>("arm/handEnable", handEnable, false);
        m_hand_tcp_nh.param<int>("arm/Hand_Freedom", Hand_Freedom, 9);
        m_hand_tcp_nh.param<int>("arm/JointVelocity", JointVelocity, 10);
        m_hand_tcp_nh.param<int>("arm/JointEffort", JointEffort, 100);
        m_hand_tcp_nh.param<float>("arm/pose_X", movej_p_pose.x, -500);
        m_hand_tcp_nh.param<float>("arm/pose_Y", movej_p_pose.y, 10);
        m_hand_tcp_nh.param<float>("arm/pose_Z", movej_p_pose.z, 200);
        m_hand_tcp_nh.param<float>("arm/pose_RX", movej_p_pose.rx, 0.1);
        m_hand_tcp_nh.param<float>("arm/pose_RY", movej_p_pose.ry, 0.1);
        m_hand_tcp_nh.param<float>("arm/pose_RZ", movej_p_pose.rz, 0.1);
    }

    void TCPCommunication_Hand::Hand_InitDataSend(void)
    {
        HandClient_Send.Set_Force_RightHand.Joint1 = JointEffort;
        HandClient_Send.Set_Force_RightHand.Joint2 = JointEffort;
        HandClient_Send.Set_Force_RightHand.Joint3 = JointEffort;
        HandClient_Send.Set_Force_RightHand.Joint4 = JointEffort;
        HandClient_Send.Set_Force_RightHand.Joint5 = JointEffort;

        HandClient_Send.Set_Force_LeftHand.Joint1 = JointEffort;
        HandClient_Send.Set_Force_LeftHand.Joint2 = JointEffort;
        HandClient_Send.Set_Force_LeftHand.Joint3 = JointEffort;
        HandClient_Send.Set_Force_LeftHand.Joint4 = JointEffort;
        HandClient_Send.Set_Force_LeftHand.Joint5 = JointEffort;

        HandClient_Send.Set_Speed_RightHand.Joint1 = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint2 = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint3 = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint4 = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint5 = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint6 = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint7 = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint8 = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint9 = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint1_rotation = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint3_raw = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint1_tip = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint2_tip = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint3_tip = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint4_tip = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint5_tip = JointVelocity;

        HandClient_Send.Set_Speed_LeftHand.Joint1 = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint2 = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint3 = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint4 = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint5 = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint6 = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint7 = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint8 = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint9 = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint1_rotation = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint3_raw = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint1_tip = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint2_tip = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint3_tip = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint4_tip = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint5_tip = JointVelocity;

        HandClient_Send.Set_Point_RightHand.Joint1 = 80;
        HandClient_Send.Set_Point_RightHand.Joint2 = 128;
        HandClient_Send.Set_Point_RightHand.Joint3 = 200;
        HandClient_Send.Set_Point_RightHand.Joint4 = 200;
        HandClient_Send.Set_Point_RightHand.Joint5 = 200;
        HandClient_Send.Set_Point_RightHand.Joint6 = 200;
        HandClient_Send.Set_Point_RightHand.Joint7 = 128;
        HandClient_Send.Set_Point_RightHand.Joint8 = 128;
        HandClient_Send.Set_Point_RightHand.Joint9 = 128;
        HandClient_Send.Set_Point_RightHand.Joint1_rotation = 0;
        HandClient_Send.Set_Point_RightHand.Joint3_raw = 128;
        HandClient_Send.Set_Point_RightHand.Joint1_tip = 0;
        HandClient_Send.Set_Point_RightHand.Joint2_tip = 0;
        HandClient_Send.Set_Point_RightHand.Joint3_tip = 0;
        HandClient_Send.Set_Point_RightHand.Joint4_tip = 0;
        HandClient_Send.Set_Point_RightHand.Joint5_tip = 0;

        HandClient_Send.Set_Point_LeftHand.Joint1 = 80;
        HandClient_Send.Set_Point_LeftHand.Joint2 = 128;
        HandClient_Send.Set_Point_LeftHand.Joint3 = 200;
        HandClient_Send.Set_Point_LeftHand.Joint4 = 200;
        HandClient_Send.Set_Point_LeftHand.Joint5 = 200;
        HandClient_Send.Set_Point_LeftHand.Joint6 = 200;
        HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
        HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
        HandClient_Send.Set_Point_LeftHand.Joint9 = 128;
        HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 0;
        HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
        HandClient_Send.Set_Point_LeftHand.Joint1_tip = 0;
        HandClient_Send.Set_Point_LeftHand.Joint2_tip = 0;
        HandClient_Send.Set_Point_LeftHand.Joint3_tip = 0;
        HandClient_Send.Set_Point_LeftHand.Joint4_tip = 0;
        HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
    }
    void TCPCommunication_Hand::Hand_CrawlPoses(int tool_id)
    {
        if (tool_id == 1) // 丝绸
        {
            HandClient_Send.Set_Point_LeftHand.Joint3 = 110;
            HandClient_Send.Set_Point_LeftHand.Joint4 = 110;
            HandClient_Send.Set_Point_LeftHand.Joint5 = 110;
            HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

            HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
            HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
            HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

            HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
            HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

            HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
        }
        else if (tool_id == 2) // 鸡蛋
        {
            HandClient_Send.Set_Point_LeftHand.Joint3 = 100;
            HandClient_Send.Set_Point_LeftHand.Joint4 = 150;
            HandClient_Send.Set_Point_LeftHand.Joint5 = 100;
            HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

            HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
            HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
            HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

            HandClient_Send.Set_Point_LeftHand.Joint7 = 50;
            HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
            HandClient_Send.Set_Point_LeftHand.Joint8 = 200;
            HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

            HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
        }
        else if (tool_id == 3) // 电池
        {
            HandClient_Send.Set_Point_LeftHand.Joint3 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint4 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint5 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

            HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
            HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
            HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

            HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
            HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

            HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
        }
        else if (tool_id == 4) // 胶囊
        {
            HandClient_Send.Set_Point_LeftHand.Joint3 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint4 = 0;
            HandClient_Send.Set_Point_LeftHand.Joint5 = 0;
            HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

            HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
            HandClient_Send.Set_Point_LeftHand.Joint2 = 115;
            HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

            HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
            HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

            HandClient_Send.Set_Point_LeftHand.Joint2_tip = 90;
            HandClient_Send.Set_Point_LeftHand.Joint3_tip = 0;
            HandClient_Send.Set_Point_LeftHand.Joint4_tip = 0;
            HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
        }
        else if (tool_id == 5) // 球
        {
            HandClient_Send.Set_Point_LeftHand.Joint3 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint4 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint5 = 90;
            HandClient_Send.Set_Point_LeftHand.Joint6 = 0;

            HandClient_Send.Set_Point_LeftHand.Joint1 = 140;
            HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
            HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
            HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

            HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
            HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

            HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint3_tip = 110;
            HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
        }
        else if (tool_id == 6) // 螺丝刀
        {
            HandClient_Send.Set_Point_LeftHand.Joint3 = 110;
            HandClient_Send.Set_Point_LeftHand.Joint4 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint5 = 110;
            HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

            HandClient_Send.Set_Point_LeftHand.Joint1 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
            HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
            HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

            HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
            HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

            HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
        }
        else if (tool_id == 7) // 塑料瓶
        {
            HandClient_Send.Set_Point_LeftHand.Joint3 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint4 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint5 = 120;
            HandClient_Send.Set_Point_LeftHand.Joint6 = 120;

            HandClient_Send.Set_Point_LeftHand.Joint1 = 250;
            HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
            HandClient_Send.Set_Point_LeftHand.Joint2 = 50;
            HandClient_Send.Set_Point_LeftHand.Joint1_tip = 100;

            HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
            HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

            HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint5_tip = 100;
        }
        else if (tool_id == 8) // 螺栓
        {
            HandClient_Send.Set_Point_LeftHand.Joint3 = 130;
            HandClient_Send.Set_Point_LeftHand.Joint4 = 115;
            HandClient_Send.Set_Point_LeftHand.Joint5 = 130;
            HandClient_Send.Set_Point_LeftHand.Joint6 = 250;

            HandClient_Send.Set_Point_LeftHand.Joint1 = 90;
            HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 220;
            HandClient_Send.Set_Point_LeftHand.Joint2 = 80;
            HandClient_Send.Set_Point_LeftHand.Joint1_tip = 180;

            HandClient_Send.Set_Point_LeftHand.Joint7 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint3_raw = 128;
            HandClient_Send.Set_Point_LeftHand.Joint8 = 128;
            HandClient_Send.Set_Point_LeftHand.Joint9 = 128;

            HandClient_Send.Set_Point_LeftHand.Joint2_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint3_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint4_tip = 100;
            HandClient_Send.Set_Point_LeftHand.Joint5_tip = 0;
        }
    }
    bool TCPCommunication_Hand::Hand_TCPCommunication_connect(const ros::NodeHandle &ros_TCP_nh)
    {
        if (!handEnable) return false;
        m_hand_tcp_nh = ros_TCP_nh;
        ros::NodeHandle private_node("~");
        private_node.param("hand_port", m_server_hand_port, 20008);
        private_node.param("hand_connect_timeout_ms", hand_connect_timeout_ms, 250);
        private_node.param("hand_io_timeout_ms", hand_io_timeout_ms, 100);
        private_node.param("hand_reconnect_delay_ms", hand_reconnect_delay_ms, 1000);
        if (m_server_hand_port < 1 || m_server_hand_port > 65535 ||
            hand_connect_timeout_ms < 1 || hand_connect_timeout_ms > 60000 ||
            hand_io_timeout_ms < 1 || hand_io_timeout_ms > 60000 ||
            hand_reconnect_delay_ms < 1 || hand_reconnect_delay_ms > 60000)
        {
            ROS_ERROR("Invalid hand TCP port or timeout configuration");
            return false;
        }
        if (!initTargetPoses()) return false;
        if (hand_stream.connectTo(m_server_hand_ip, m_server_hand_port,
                                  hand_connect_timeout_ms) != TcpStream::Result::Complete)
        {
            ROS_ERROR("Hand TCP connection failed");
            return false;
        }
        m_hand_isConnected = true;
        hand_feedback_valid = false;
        m_hand_timer = ros_TCP_nh.createWallTimer(ros::WallDuration(0.02), &TCPCommunication_Hand::Hand_TCPSendRecvMessage, this);

        sub_right_hand = m_hand_tcp_nh.subscribe("/right_hand_control", 10, &TCPCommunication_Hand::RightHandCtrl_subCallback, this);
        sub_left_hand = m_hand_tcp_nh.subscribe("/left_hand_control", 10, &TCPCommunication_Hand::LeftHandCtrl_subCallback, this);
        pub_right_hand = m_hand_tcp_nh.advertise<sensor_msgs::JointState>("/right_hand_states", 10);
        pub_left_hand = m_hand_tcp_nh.advertise<sensor_msgs::JointState>("/left_hand_states", 10);
        pub_right_glove = m_hand_tcp_nh.advertise<sensor_msgs::JointState>("/right_glove_states", 10);
        pub_left_glove = m_hand_tcp_nh.advertise<sensor_msgs::JointState>("/left_glove_states", 10);

        // result_sub = m_hand_tcp_nh.subscribe("/arm_motion_result", 10, &TCPCommunication_Hand::motionResultCallback, this);

        // pose_pub = m_hand_tcp_nh.advertise<geometry_msgs::Pose>("/arm_target_pose", 10);
        connectVisual(m_hand_tcp_nh);
        //  初始化目标位姿队列
        current_index = 0;
        return true;
    }

    ///////////////////////////////////////////////////////////////////////////
    void TCPCommunication_Hand::connectVisual(ros::NodeHandle node)
    {
        get_Visual_pub = node.advertise<std_msgs::String>("/aimo/robot_control/OrderDetect0bj", 10);
        Visual_msg.data = "detect_objects";
        detection_sub = node.subscribe("/aimo/robot_ai/orderDetectobj_result", 10,
                                      &TCPCommunication_Hand::orderDetectCallback, this);
    }

    bool TCPCommunication_Hand::initTargetPoses()
    {
        ControllerConfiguration candidate;
        std::string error;
        if (!loadControllerConfiguration(ros::NodeHandle("~"), ros::package::getPath("HandTCPCommunication"),
                                         candidate, error))
        {
            ROS_ERROR("%s", error.c_str());
            configuration_valid = false;
            target_poses.clear();
            Tools_Tcp.clear();
            tools_farmeName.clear();
            fixed_grasp_targets.clear();
            obj_list.clear();
            if_complite = false;
            show_step = 9;
            return false;
        }
        target_poses = candidate.joint_poses;
        Tools_Tcp = candidate.tool_offsets;
        tools_farmeName = candidate.tool_frames;
        fixed_grasp_targets = candidate.fixed_targets;
        arm_configuration = std::move(candidate);
        configuration_valid = true;
        return true;
    }
    //////////////////////天慧视觉topic解析
    void TCPCommunication_Hand::GetVisualData_subCallback(const sensor_msgs::PointCloud2ConstPtr &GridData)
    {
        if (!if_complite)
        {
            object_data.clear();
            //  获取字段信息
            const std::vector<sensor_msgs::PointField> &fields = GridData->fields;

            // 获取数据指针
            const uint8_t *data_ptr = &GridData->data[0];
            int id;
            ObjectData obj_data;
            float cls, x1, y1, z1, x2, y2, z2, a, b, c, d;
            size_t point_step = GridData->point_step; // 每个点的步长，包含所有字段的大小

            // // 遍历每个点的数据
            for (size_t i = 0; i < GridData->width * GridData->height; ++i)
            {
                // 获取当前点的起始地址
                const uint8_t *point_ptr = data_ptr + i * point_step;
                // 从当前点的内存位置解析数据
                memcpy(&cls, point_ptr + getFieldOffset(fields, "cls"), sizeof(float));
                obj_data.id = static_cast<int>(cls);
                memcpy(&obj_data.cen_x, point_ptr + getFieldOffset(fields, "x1"), sizeof(float));
                memcpy(&obj_data.cen_y, point_ptr + getFieldOffset(fields, "y1"), sizeof(float));
                memcpy(&obj_data.cen_z, point_ptr + getFieldOffset(fields, "z1"), sizeof(float));
                memcpy(&obj_data.vec_x, point_ptr + getFieldOffset(fields, "x2"), sizeof(float));
                memcpy(&obj_data.vec_y, point_ptr + getFieldOffset(fields, "y2"), sizeof(float));
                memcpy(&obj_data.vec_z, point_ptr + getFieldOffset(fields, "z2"), sizeof(float));
                memcpy(&obj_data.a, point_ptr + getFieldOffset(fields, "a"), sizeof(float));
                memcpy(&obj_data.b, point_ptr + getFieldOffset(fields, "b"), sizeof(float));
                memcpy(&obj_data.c, point_ptr + getFieldOffset(fields, "c"), sizeof(float));
                memcpy(&obj_data.d, point_ptr + getFieldOffset(fields, "d"), sizeof(float));

                object_data.push_back(obj_data);

                // // 处理数据，例如打印出来
                ROS_INFO("cls: %d, x1: %f, y1: %f, z1: %f, x2: %f, y2: %f, z2: %f, a: %f, b: %f, c: %f, d: %f",
                         object_data[id - 1].id, object_data[id - 1].cen_x, object_data[id - 1].cen_y, object_data[id - 1].cen_z, object_data[id - 1].vec_x, object_data[id - 1].vec_y, object_data[id - 1].vec_z, object_data[id - 1].a, object_data[id - 1].b, object_data[id - 1].c, object_data[id - 1].d);
            }
            Pose robot;
            robot.position.x = -0.382;
            robot.position.y = 0;
            robot.position.z = 0.496;
            robot.euler.rx = 3.142;
            robot.euler.ry = 1.571;
            robot.euler.rz = 0;

            for (size_t i = 0; i < object_data.size(); i++)
            {
                Pose camera_point;
                camera_point.position.x = object_data[i].cen_x;
                camera_point.position.y = object_data[i].cen_y;
                camera_point.position.z = object_data[i].cen_z;
                camera_point.euler.rx = 0;
                camera_point.euler.ry = 0;
                camera_point.euler.rz = 0;

                Pose robot_zh;
                robot_zh = coorTrans.GetGraspPose(camera_point, robot, Calibration_Result_EyeInHand);
                pass_obj.rz = Getry_pose(object_data[i].vec_x, object_data[i].vec_y, object_data[i].vec_z);
                Grap_Obj pass_obj;
                pass_obj.ID = object_data[i].id;
                pass_obj.x = robot_zh.position.x;
                pass_obj.y = robot_zh.position.y;
                pass_obj.z = robot_zh.position.z;

                // 如果 obj_list 为空，直接将物品添加到 obj_list 中
                if (obj_list.empty())
                {
                    obj_list.push_back(pass_obj);
                }
                else
                {
                    bool found = false;
                    // 遍历 obj_list 中的元素
                    for (auto it = obj_list.begin(); it != obj_list.end();)
                    {
                        if (it->ID == pass_obj.ID)
                        {
                            // 如果 obj_list 中已经有该 ID，更新 obj_list 中的元素
                            it->x = pass_obj.x;
                            it->y = pass_obj.y;
                            it->z = pass_obj.z;
                            it->rz = pass_obj.rz;
                            found = true;
                            ++it; // 继续遍历下一个元素
                        }
                        else
                        {
                            // 如果 obj_list 中的元素不在 object_data 中，删除
                            bool to_remove = true;
                            for (size_t j = 0; j < object_data.size(); j++)
                            {
                                if (object_data[j].id == it->ID)
                                {
                                    to_remove = false;
                                    break;
                                }
                            }

                            if (to_remove)
                            {
                                it = obj_list.erase(it); // 删除元素
                            }
                            else
                            {
                                ++it; // 保持遍历
                            }
                        }
                    }

                    // 如果 obj_list 中没有该 ID，添加到 obj_list 中
                    if (!found)
                    {
                        obj_list.push_back(pass_obj);
                    }
                }
            }

            for (auto it = obj_list.begin(); it != obj_list.end();)
            {
                std::cout << "obj_list:ID: " << it->ID << "x = " << it->x << "y = " << it->y << "z = " << it->z << std::endl;
                ++it;
            }

            if_complite = true;
        }
    }
    void TCPCommunication_Hand::orderDetectCallback(const HandTCPCommunication::OrderDetectResult::ConstPtr& msg)
    {
        if (msg->bboxes.size() != msg->labels.size() * 4)
        {
            ROS_ERROR("Each detection label requires four bounding box coordinates");
            return;
        }
        for (size_t i = 0; i < msg->labels.size(); ++i)
        {
            const size_t offset = i * 4;
            const auto &boxes = msg->bboxes;
            if (!std::isfinite(boxes[offset]) || !std::isfinite(boxes[offset + 1]) ||
                !std::isfinite(boxes[offset + 2]) || !std::isfinite(boxes[offset + 3]) ||
                boxes[offset] > boxes[offset + 2] || boxes[offset + 1] > boxes[offset + 3])
            {
                ROS_ERROR("Invalid detection bounding box");
                return;
            }
        }
        // 遍历处理每个物品
        vector<pair<float, float>> centers;  // 存储中心点坐标
        
        for(size_t i = 0; i < msg->labels.size(); ++i)
        {
            // 获取当前物品的坐标索引
            const size_t idx = i * 4;
            
            // 提取坐标值
            const float x1 = msg->bboxes[idx];      // 左上x
            const float y1 = msg->bboxes[idx+1];    // 左上y
            const float x2 = msg->bboxes[idx+2];    // 右下x
            const float y2 = msg->bboxes[idx+3];    // 右下y

            // 计算中心点
            const float center_x = (x1 + x2) / 2.0f;
            const float center_y = (y1 + y2) / 2.0f;
            
            // 存入容器
            centers.emplace_back(center_x, center_y);

            // 打印结果
            ROS_INFO("Item [%zu] '%s' center at (%.2f, %.2f)",
                    i, 
                    msg->labels[i].c_str(),
                    center_x,
                    center_y);
        }
        if (show_step != 1 || if_complite) return;
        if (fixed_grasp_targets.empty())
        {
            ROS_WARN_THROTTLE(5, "Configure fixed_grasp_targets before using 2D detections for fixed-position grasping");
            return;
        }
        std::vector<Grap_Obj> tasks;
        for (const auto &label : msg->labels)
        {
            auto target = fixed_grasp_targets.find(label);
            if (target == fixed_grasp_targets.end()) continue;
            bool duplicate = false;
            for (const auto &task : tasks) duplicate |= task.ID == target->second.ID;
            if (!duplicate) tasks.push_back(target->second);
        }
        obj_list.swap(tasks);
        if_complite = true;
    }
    // 获取字段在数据中的偏移量
    int TCPCommunication_Hand::getFieldOffset(const std::vector<sensor_msgs::PointField> &fields, const std::string &field_name)
    {
        for (const auto &field : fields)
        {
            if (field.name == field_name)
            {
                return field.offset;
            }
        }
        ROS_ERROR("Field %s not found", field_name.c_str());
        return -1;
    }
    void TCPCommunication_Hand::motionResultCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if_complite = msg->data;
    }
    void TCPCommunication_Hand::RightHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        if (msg->position.size() == 16)
        {
            std::cout << "RightHandCtrl_subCallback " << std::endl;
            // 关节角度
            HandClient_Send.Set_Point_RightHand.Joint1 = msg->position[0];
            HandClient_Send.Set_Point_RightHand.Joint2 = msg->position[1];
            HandClient_Send.Set_Point_RightHand.Joint3 = msg->position[2];
            HandClient_Send.Set_Point_RightHand.Joint4 = msg->position[3];
            HandClient_Send.Set_Point_RightHand.Joint5 = msg->position[4];
            HandClient_Send.Set_Point_RightHand.Joint6 = msg->position[5];
            HandClient_Send.Set_Point_RightHand.Joint7 = msg->position[6];
            HandClient_Send.Set_Point_RightHand.Joint8 = msg->position[7];
            HandClient_Send.Set_Point_RightHand.Joint9 = msg->position[8];

            HandClient_Send.Set_Point_RightHand.Joint1_rotation = msg->position[9];

            HandClient_Send.Set_Point_RightHand.Joint3_raw = msg->position[10];
            HandClient_Send.Set_Point_RightHand.Joint1_tip = msg->position[11];
            HandClient_Send.Set_Point_RightHand.Joint2_tip = msg->position[12];
            HandClient_Send.Set_Point_RightHand.Joint3_tip = msg->position[13];
            HandClient_Send.Set_Point_RightHand.Joint4_tip = msg->position[14];
            HandClient_Send.Set_Point_RightHand.Joint5_tip = msg->position[15];
        }
        else
        {
            ROS_WARN("RightHandCtrl_subCallback: joint state size is not 16!");
        }

        // 速度值
        if (msg->velocity.size() < 16 && msg->velocity.size() > 0)
        {
            HandClient_Send.Set_Speed_RightHand.Joint1 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint2 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint3 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint4 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint5 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint6 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint7 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint8 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint9 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];

            HandClient_Send.Set_Speed_RightHand.Joint1_rotation = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];

            HandClient_Send.Set_Speed_RightHand.Joint3_raw = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint1_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint2_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint3_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint4_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint5_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
        }
        else if (msg->velocity.size() >= 16)
        {
            HandClient_Send.Set_Speed_RightHand.Joint1 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint2 = (msg->velocity[1] == 0) ? JointVelocity : msg->velocity[1];
            HandClient_Send.Set_Speed_RightHand.Joint3 = (msg->velocity[2] == 0) ? JointVelocity : msg->velocity[2];
            HandClient_Send.Set_Speed_RightHand.Joint4 = (msg->velocity[3] == 0) ? JointVelocity : msg->velocity[3];
            HandClient_Send.Set_Speed_RightHand.Joint5 = (msg->velocity[4] == 0) ? JointVelocity : msg->velocity[4];
            HandClient_Send.Set_Speed_RightHand.Joint6 = (msg->velocity[5] == 0) ? JointVelocity : msg->velocity[5];
            HandClient_Send.Set_Speed_RightHand.Joint7 = (msg->velocity[6] == 0) ? JointVelocity : msg->velocity[6];
            HandClient_Send.Set_Speed_RightHand.Joint8 = (msg->velocity[7] == 0) ? JointVelocity : msg->velocity[7];
            HandClient_Send.Set_Speed_RightHand.Joint9 = (msg->velocity[8] == 0) ? JointVelocity : msg->velocity[8];

            HandClient_Send.Set_Speed_RightHand.Joint1_rotation = (msg->velocity[9] == 0) ? JointVelocity : msg->velocity[9];

            HandClient_Send.Set_Speed_RightHand.Joint3_raw = (msg->velocity[10] == 0) ? JointVelocity : msg->velocity[10];
            HandClient_Send.Set_Speed_RightHand.Joint1_tip = (msg->velocity[11] == 0) ? JointVelocity : msg->velocity[11];
            HandClient_Send.Set_Speed_RightHand.Joint2_tip = (msg->velocity[12] == 0) ? JointVelocity : msg->velocity[12];
            HandClient_Send.Set_Speed_RightHand.Joint3_tip = (msg->velocity[13] == 0) ? JointVelocity : msg->velocity[13];
            HandClient_Send.Set_Speed_RightHand.Joint4_tip = (msg->velocity[14] == 0) ? JointVelocity : msg->velocity[14];
            HandClient_Send.Set_Speed_RightHand.Joint5_tip = (msg->velocity[15] == 0) ? JointVelocity : msg->velocity[15];
        }
        else
        {
            ROS_WARN("RightHandCtrl_subCallback: velocity state size is warn!");
        }

        // 力度
        if (msg->effort.size() < 5 && msg->effort.size() > 0)
        {
            HandClient_Send.Set_Force_RightHand.Joint1 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_RightHand.Joint2 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_RightHand.Joint3 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_RightHand.Joint4 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_RightHand.Joint5 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
        }
        else if (msg->effort.size() >= 5)
        {
            HandClient_Send.Set_Force_RightHand.Joint1 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_RightHand.Joint2 = (msg->effort[1] == 0) ? JointEffort : msg->effort[1];
            HandClient_Send.Set_Force_RightHand.Joint3 = (msg->effort[2] == 0) ? JointEffort : msg->effort[2];
            HandClient_Send.Set_Force_RightHand.Joint4 = (msg->effort[3] == 0) ? JointEffort : msg->effort[3];
            HandClient_Send.Set_Force_RightHand.Joint5 = (msg->effort[4] == 0) ? JointEffort : msg->effort[4];
        }
        else
        {
            ROS_WARN("RightHandCtrl_subCallback: effort state size is warn!");
        }
    }
    void TCPCommunication_Hand::LeftHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        if (msg->position.size() == 16)
        {
            std::cout << "LeftHandCtrl_subCallback " << std::endl;
            HandClient_Send.Set_Point_LeftHand.Joint1 = msg->position[0];
            HandClient_Send.Set_Point_LeftHand.Joint2 = msg->position[1];
            HandClient_Send.Set_Point_LeftHand.Joint3 = msg->position[2];
            HandClient_Send.Set_Point_LeftHand.Joint4 = msg->position[3];
            HandClient_Send.Set_Point_LeftHand.Joint5 = msg->position[4];
            HandClient_Send.Set_Point_LeftHand.Joint6 = msg->position[5];
            HandClient_Send.Set_Point_LeftHand.Joint7 = msg->position[6];
            HandClient_Send.Set_Point_LeftHand.Joint8 = msg->position[7];
            HandClient_Send.Set_Point_LeftHand.Joint9 = msg->position[8];

            HandClient_Send.Set_Point_LeftHand.Joint1_rotation = msg->position[9];

            HandClient_Send.Set_Point_LeftHand.Joint3_raw = msg->position[10];
            HandClient_Send.Set_Point_LeftHand.Joint1_tip = msg->position[11];
            HandClient_Send.Set_Point_LeftHand.Joint2_tip = msg->position[12];
            HandClient_Send.Set_Point_LeftHand.Joint3_tip = msg->position[13];
            HandClient_Send.Set_Point_LeftHand.Joint4_tip = msg->position[14];
            HandClient_Send.Set_Point_LeftHand.Joint5_tip = msg->position[15];
        }
        else
        {
            ROS_WARN("LeftHandCtrl_subCallback: joint state size is not 16!");
        }

        // 速度值
        if (msg->velocity.size() < 16 && msg->velocity.size() > 0)
        {
            HandClient_Send.Set_Speed_LeftHand.Joint1 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint2 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint3 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint4 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint5 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint6 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint7 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint8 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint9 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];

            HandClient_Send.Set_Speed_LeftHand.Joint1_rotation = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];

            HandClient_Send.Set_Speed_LeftHand.Joint3_raw = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint1_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint2_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint3_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint4_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint5_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
        }
        else if (msg->velocity.size() >= 16)
        {
            HandClient_Send.Set_Speed_LeftHand.Joint1 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint2 = (msg->velocity[1] == 0) ? JointVelocity : msg->velocity[1];
            HandClient_Send.Set_Speed_LeftHand.Joint3 = (msg->velocity[2] == 0) ? JointVelocity : msg->velocity[2];
            HandClient_Send.Set_Speed_LeftHand.Joint4 = (msg->velocity[3] == 0) ? JointVelocity : msg->velocity[3];
            HandClient_Send.Set_Speed_LeftHand.Joint5 = (msg->velocity[4] == 0) ? JointVelocity : msg->velocity[4];
            HandClient_Send.Set_Speed_LeftHand.Joint6 = (msg->velocity[5] == 0) ? JointVelocity : msg->velocity[5];
            HandClient_Send.Set_Speed_LeftHand.Joint7 = (msg->velocity[6] == 0) ? JointVelocity : msg->velocity[6];
            HandClient_Send.Set_Speed_LeftHand.Joint8 = (msg->velocity[7] == 0) ? JointVelocity : msg->velocity[7];
            HandClient_Send.Set_Speed_LeftHand.Joint9 = (msg->velocity[8] == 0) ? JointVelocity : msg->velocity[8];

            HandClient_Send.Set_Speed_LeftHand.Joint1_rotation = (msg->velocity[9] == 0) ? JointVelocity : msg->velocity[9];

            HandClient_Send.Set_Speed_LeftHand.Joint3_raw = (msg->velocity[10] == 0) ? JointVelocity : msg->velocity[10];
            HandClient_Send.Set_Speed_LeftHand.Joint1_tip = (msg->velocity[11] == 0) ? JointVelocity : msg->velocity[11];
            HandClient_Send.Set_Speed_LeftHand.Joint2_tip = (msg->velocity[12] == 0) ? JointVelocity : msg->velocity[12];
            HandClient_Send.Set_Speed_LeftHand.Joint3_tip = (msg->velocity[13] == 0) ? JointVelocity : msg->velocity[13];
            HandClient_Send.Set_Speed_LeftHand.Joint4_tip = (msg->velocity[14] == 0) ? JointVelocity : msg->velocity[14];
            HandClient_Send.Set_Speed_LeftHand.Joint5_tip = (msg->velocity[15] == 0) ? JointVelocity : msg->velocity[15];
        }
        else
        {
            ROS_WARN("RightHandCtrl_subCallback: velocity state size is warn!");
        }

        // 力度
        if (msg->effort.size() < 5 && msg->effort.size() > 0)
        {
            HandClient_Send.Set_Force_LeftHand.Joint1 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_LeftHand.Joint2 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_LeftHand.Joint3 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_LeftHand.Joint4 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_LeftHand.Joint5 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
        }
        else if (msg->effort.size() >= 5)
        {
            HandClient_Send.Set_Force_LeftHand.Joint1 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_LeftHand.Joint2 = (msg->effort[1] == 0) ? JointEffort : msg->effort[1];
            HandClient_Send.Set_Force_LeftHand.Joint3 = (msg->effort[2] == 0) ? JointEffort : msg->effort[2];
            HandClient_Send.Set_Force_LeftHand.Joint4 = (msg->effort[3] == 0) ? JointEffort : msg->effort[3];
            HandClient_Send.Set_Force_LeftHand.Joint5 = (msg->effort[4] == 0) ? JointEffort : msg->effort[4];
        }
        else
        {
            ROS_WARN("RightHandCtrl_subCallback: effort state size is warn!");
        }
    }

    void TCPCommunication_Hand::RightHand_pubFunction(void)
    {
        if (!hand_feedback_valid) return;
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "right_hand";
        joint_state.name.resize(16);
        joint_state.position.resize(16);
        joint_state.effort.resize(5);

        joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9",
                            "joint10", "joint11", "joint12", "joint13", "joint14", "joint15", "joint16"};

        joint_state.position[0] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint1;
        joint_state.position[1] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint2;
        joint_state.position[2] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint3;
        joint_state.position[3] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint4;
        joint_state.position[4] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint5;
        joint_state.position[5] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint6;
        joint_state.position[6] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint7;
        joint_state.position[7] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint8;
        joint_state.position[8] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint9;
        joint_state.position[9] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint1_rotation;
        joint_state.position[10] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint3_raw;
        joint_state.position[11] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint1_tip;
        joint_state.position[12] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint2_tip;
        joint_state.position[13] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint3_tip;
        joint_state.position[14] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint4_tip;
        joint_state.position[15] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint5_tip;

        joint_state.effort[0] = HandClient_Recv.Get_CurrentForce_RightHand.Joint1;
        joint_state.effort[1] = HandClient_Recv.Get_CurrentForce_RightHand.Joint2;
        joint_state.effort[2] = HandClient_Recv.Get_CurrentForce_RightHand.Joint3;
        joint_state.effort[3] = HandClient_Recv.Get_CurrentForce_RightHand.Joint4;
        joint_state.effort[4] = HandClient_Recv.Get_CurrentForce_RightHand.Joint5;

        pub_right_hand.publish(joint_state);
    }
    void TCPCommunication_Hand::LeftHand_pubFunction(void)
    {
        if (!hand_feedback_valid) return;
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "left_hand";
        joint_state.name.resize(16);
        joint_state.position.resize(16);
        joint_state.effort.resize(5);

        joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9",
                            "joint10", "joint11", "joint12", "joint13", "joint14", "joint15", "joint16"};

        joint_state.position[0] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint1;
        joint_state.position[1] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint2;
        joint_state.position[2] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint3;
        joint_state.position[3] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint4;
        joint_state.position[4] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint5;
        joint_state.position[5] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint6;
        joint_state.position[6] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint7;
        joint_state.position[7] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint8;
        joint_state.position[8] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint9;
        joint_state.position[9] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint1_rotation;
        joint_state.position[10] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint3_raw;
        joint_state.position[11] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint1_tip;
        joint_state.position[12] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint2_tip;
        joint_state.position[13] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint3_tip;
        joint_state.position[14] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint4_tip;
        joint_state.position[15] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint5_tip;

        joint_state.effort[0] = HandClient_Recv.Get_CurrentForce_LeftHand.Joint1;
        joint_state.effort[1] = HandClient_Recv.Get_CurrentForce_LeftHand.Joint2;
        joint_state.effort[2] = HandClient_Recv.Get_CurrentForce_LeftHand.Joint3;
        joint_state.effort[3] = HandClient_Recv.Get_CurrentForce_LeftHand.Joint4;
        joint_state.effort[4] = HandClient_Recv.Get_CurrentForce_LeftHand.Joint5;

        pub_left_hand.publish(joint_state);
    }

    void TCPCommunication_Hand::RightGlove_pubFunction(void)
    {
        if (!hand_feedback_valid) return;
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "right_glove";
        joint_state.name.resize(16);
        joint_state.position.resize(16);

        joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9",
                            "joint10", "joint11", "joint12", "joint13", "joint14", "joint15", "joint16"};

        joint_state.position[0] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint1;
        joint_state.position[1] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint2;
        joint_state.position[2] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint3;
        joint_state.position[3] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint4;
        joint_state.position[4] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint5;
        joint_state.position[5] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint6;
        joint_state.position[6] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint7;
        joint_state.position[7] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint8;
        joint_state.position[8] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint9;
        joint_state.position[9] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint1_rotation;
        joint_state.position[10] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint3_raw;
        joint_state.position[11] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint1_tip;
        joint_state.position[12] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint2_tip;
        joint_state.position[13] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint3_tip;
        joint_state.position[14] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint4_tip;
        joint_state.position[15] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint5_tip;

        pub_right_glove.publish(joint_state);
    }
    void TCPCommunication_Hand::LeftGlove_pubFunction(void)
    {
        if (!hand_feedback_valid) return;
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "left_glove";
        joint_state.name.resize(16);
        joint_state.position.resize(16);

        joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9",
                            "joint10", "joint11", "joint12", "joint13", "joint14", "joint15", "joint16"};

        joint_state.position[0] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint1;
        joint_state.position[1] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint2;
        joint_state.position[2] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint3;
        joint_state.position[3] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint4;
        joint_state.position[4] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint5;
        joint_state.position[5] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint6;
        joint_state.position[6] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint7;
        joint_state.position[7] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint8;
        joint_state.position[8] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint9;
        joint_state.position[9] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint1_rotation;
        joint_state.position[10] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint3_raw;
        joint_state.position[11] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint1_tip;
        joint_state.position[12] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint2_tip;
        joint_state.position[13] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint3_tip;
        joint_state.position[14] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint4_tip;
        joint_state.position[15] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint5_tip;

        pub_left_glove.publish(joint_state);
    }

    ///////////////////////////////////////////////////////////////////////////
    void TCPCommunication_Hand::Hand_TCPSendRecvMessage(const ros::WallTimerEvent &)
    {
        if (!m_hand_isConnected)
        {
            if (TcpStream::Clock::now() < next_hand_connect) return;
            if (hand_stream.connectTo(m_server_hand_ip, m_server_hand_port,
                                      hand_connect_timeout_ms) != TcpStream::Result::Complete)
            {
                next_hand_connect = TcpStream::Clock::now() +
                    std::chrono::milliseconds(hand_reconnect_delay_ms);
                ROS_WARN_THROTTLE(5, "Hand TCP reconnect failed");
                return;
            }
            m_hand_isConnected = true;
            ROS_INFO("Hand TCP connection restored; grasp sequence remains stopped");
        }
        if (Hand_DataDeal_Struct() != 0)
        {
            // Never reuse a stream after an incomplete request or response.
            hand_stream.close();
            m_hand_isConnected = false;
            hand_feedback_valid = false;
            show_step = 9;
            next_hand_connect = TcpStream::Clock::now() +
                std::chrono::milliseconds(hand_reconnect_delay_ms);
        }
    }

    int TCPCommunication_Hand::Hand_DataDeal_Struct()
    {
        Hand_SendStructDataDeal(HandClient_Send, TCP_SendStruct_Data);
        const auto deadline = TcpStream::Clock::now() +
            std::chrono::milliseconds(hand_io_timeout_ms);
        auto result = hand_stream.writeAll(&TCP_SendStruct_Data, sizeof(TCP_SendStruct_Data), deadline);
        Hand_RecvStruct response{};
        if (result == TcpStream::Result::Complete)
            result = hand_stream.readAll(&response, sizeof(response), deadline);
        if (result != TcpStream::Result::Complete)
        {
            ROS_WARN_THROTTLE(5, "Hand TCP exchange failed (%s); discarding connection",
                result == TcpStream::Result::Timeout ? "timeout" : "connection error");
            return -1;
        }
        TCP_RecvStruct_Data = response;
        Hand_RecvStructDataDeal(response, HandClient_Recv);
        hand_feedback_valid = true;
        return 0;
    }
    void TCPCommunication_Hand::Hand_SendStructDataDeal(TCP_Hand_SendData handData, Hand_SendStruct &sendData)
    {
        sendData.PowerOn = handData.PowerOn;
        sendData.Work_Mode = handData.Work_Mode;
        sendData.Control_Mode = handData.Control_Mode;
        sendData.Move_Control = handData.Move_Control;

        sendData.Set_Point_RightHand = handData.Set_Point_RightHand;
        sendData.Set_Speed_RightHand = handData.Set_Speed_RightHand;

        sendData.Set_Point_LeftHand = handData.Set_Point_LeftHand;
        sendData.Set_Speed_LeftHand = handData.Set_Speed_LeftHand;

        sendData.Set_Force_RightHand = handData.Set_Force_RightHand;
        sendData.Set_Force_LeftHand = handData.Set_Force_LeftHand;

        sendData.Hand_Freedom = Hand_Freedom;
    }
    void TCPCommunication_Hand::Hand_RecvStructDataDeal(Hand_RecvStruct recvData, TCP_Hand_RecvData &handData)
    {
        handData.PowerOn_State = recvData.PowerOn_State;
        switch (recvData.Work_State)
        {
        case 1:
            handData.Work_State = "Cart";
            break;
        case 2:
            handData.Work_State = "Point";
            break;
        default:
            break;
        }
        switch (recvData.Control_State)
        {
        case 1:
            handData.Control_State = "Homing";
            break;
        case 2:
            handData.Control_State = "QuickMove";
            break;
        case 3:
            handData.Control_State = "Prospect";
            break;
        case 4:
            handData.Control_State = "LinearInterpolation";
            break;
        case 5:
            handData.Control_State = "ClearError";
            break;
        default:
            break;
        }

        switch (recvData.RightHand_MoveState)
        {
        case 1:
            handData.RightHand_MoveState = "Static";
            break;
        case 2:
            handData.RightHand_MoveState = "Move";
            break;
        case 3:
            handData.RightHand_MoveState = "MoveEnd";
            break;
        default:
            break;
        }
        switch (recvData.LeftHand_MoveState)
        {
        case 1:
            handData.LeftHand_MoveState = "Static";
            break;
        case 2:
            handData.LeftHand_MoveState = "Move";
            break;
        case 3:
            handData.LeftHand_MoveState = "MoveEnd";
            break;
        default:
            break;
        }
        handData.Error_State = recvData.Error_State;

        handData.Get_CurrentPoint_RightHand = recvData.Get_CurrentPoint_RightHand;
        handData.Get_CurrentPoint_LeftHand = recvData.Get_CurrentPoint_LeftHand;

        handData.Get_CurrentForce_RightHand = recvData.Get_CurrentForce_RightHand;
        handData.Get_CurrentForce_LeftHand = recvData.Get_CurrentForce_LeftHand;

        handData.Get_CurrentPoint_RightGlove = recvData.Get_CurrentPoint_RightGlove;
        handData.Get_CurrentPoint_LeftGlove = recvData.Get_CurrentPoint_LeftGlove;

        // std::cout << "recvData.Get_CurrentPoint_RightHand: " << recvData.Get_CurrentPoint_RightHand.Joint1
        // << ", " << recvData.Get_CurrentPoint_RightHand.Joint2 << ", " << recvData.Get_CurrentPoint_RightHand.Joint3
        // << ", " << recvData.Get_CurrentPoint_RightHand.Joint4 << ", " << recvData.Get_CurrentPoint_RightHand.Joint5
        // << std::endl;
    }

    int TCPCommunication_Hand::initializeArmApi()
    {
        if (!m_pApi) m_pApi = new RM_Service();
        // This controller uses six joint values and retains the RM65 model.
        return m_pApi->Service_RM_API_Init(65, MCallback);
    }
    SOCKHANDLE TCPCommunication_Hand::connectArmSocket(const std::string &ip, int port, int timeout_ms)
    {
        return Arm_Socket_Start(ip.c_str(), port, timeout_ms);
    }
    int TCPCommunication_Hand::armSocketState(SOCKHANDLE socket)
    {
        return m_pApi && socket > 0 ? m_pApi->Service_Arm_Socket_State(socket) : -1;
    }
    int TCPCommunication_Hand::moveArmJoints(const float *joints, int speed_percent)
    {
        return m_pApi->Service_Movej_Cmd(m_sockhand, joints, speed_percent, 0, 1);
    }
    int TCPCommunication_Hand::moveArmPose(const Pose &pose, int speed_percent)
    {
        return m_pApi->Service_Movej_P_Cmd(m_sockhand, pose, speed_percent, 0, 1);
    }
    bool TCPCommunication_Hand::InitArm()
    {
        if (!initTargetPoses()) return false;
        int ret = initializeArmApi();
        if (ret != 0)
        {
            ROS_ERROR("Arm API initialization failed: %d", ret);
            return false;
        }
        const auto &arm = arm_configuration.arm;
        m_sockhand = connectArmSocket(arm.ip, arm.port, arm.timeout_ms);
        if (m_sockhand <= 0)
        {
            ROS_ERROR("Arm connection failed: %s:%d", arm.ip.c_str(), arm.port);
            return false;
        }
        ret = armSocketState(m_sockhand);
        if (ret != 0)
        {
            ROS_ERROR("Arm socket state check failed: %d", ret);
            return false;
        }
        ROS_INFO("Arm initialized: %s:%d", arm.ip.c_str(), arm.port);
        return true;
    }
    // 机械臂释放资源
    void TCPCommunication_Hand::Realseaemsocket()
    {
        // 释放API资源
        if (m_pApi && m_sockhand > 0)
        {
            m_pApi->Service_Arm_Socket_Close(m_sockhand);
            m_sockhand = -1;
        }
    }
    // 机械臂控制函数--关节运动
    bool TCPCommunication_Hand::armmove_j(geometry_msgs::Pose move_joint)
    {
        if (!configuration_valid || !validJointTarget(move_joint, arm_configuration))
        {
            ROS_ERROR("Invalid arm joint target");
            return false;
        }
        int ret = armSocketState(m_sockhand);
        if (ret == 0)
        {
            float joint[7] = {};
            joint[0] = move_joint.position.x;
            joint[1] = move_joint.position.y;
            joint[2] = move_joint.position.z;
            joint[3] = move_joint.orientation.x;
            joint[4] = move_joint.orientation.y;
            joint[5] = move_joint.orientation.z;
            std::cout << "j1: " << joint[0] << "j2: " << joint[1] << "j3: " << joint[3] << "j4: " << joint[4] << std::endl;
            sleep(1);
            int moveret = moveArmJoints(joint, arm_configuration.arm.speed_percent); // 65
            if (moveret != 0)
            {
                std::cout << "机械臂运动失败: " << moveret << std::endl;
                return false;
            }
            else
            {
                std::cout << "机械臂运动成功！" << std::endl;
                return true;
            }
        }
        else
        {
            std::cout << "机械臂未连接！" << ret << std::endl;
            return false;
        }
    }
    // 机械臂控制函数--空间运动
    bool TCPCommunication_Hand::armmovej_p(OXYZ_Pose move_pose)
    {
        if (!configuration_valid || !validCartesianTarget(move_pose, arm_configuration))
        {
            ROS_ERROR("Invalid arm Cartesian target");
            return false;
        }
        int ret = armSocketState(m_sockhand);
        if (ret == 0)
        {
            Pose GraspThPose{};
            GraspThPose.position.x = move_pose.x;
            GraspThPose.position.y = move_pose.y;
            GraspThPose.position.z = move_pose.z;
            GraspThPose.euler.rx = move_pose.rx;
            GraspThPose.euler.ry = move_pose.ry;
            GraspThPose.euler.rz = move_pose.rz;

            tf2::Quaternion rotation;
            rotation.setRPY(move_pose.rx, move_pose.ry, move_pose.rz);
            GraspThPose.quaternion.x = rotation.x();
            GraspThPose.quaternion.y = rotation.y();
            GraspThPose.quaternion.z = rotation.z();
            GraspThPose.quaternion.w = rotation.w();

            int moveret = moveArmPose(GraspThPose, arm_configuration.arm.speed_percent);
            if (moveret != 0)
            {
                std::cout << "机械臂运动失败: " << moveret << std::endl;
                return false;
            }
            else
            {
                std::cout << "机械臂运动成功！" << std::endl;
                return true;
            }
        }
        else
        {
            std::cout << "机械臂未连接！" << ret << std::endl;
            return false;
        }
    }
    bool TCPCommunication_Hand::Change_Tool_Frame(const int id)
    {
        int ret;
        if (!configuration_valid || !m_pApi || m_sockhand <= 0 ||
            id < 1 || static_cast<size_t>(id) > tools_farmeName.size()) return false;
        float joint[7]{};
        Pose po1;
        uint16_t Arm_Err;
        uint16_t Sys_Err;
        ret = m_pApi->Service_Get_Current_Arm_State(m_sockhand, joint, &po1, &Arm_Err, &Sys_Err);

        if (ret == 0)
        {
            ret = m_pApi->Service_Change_Tool_Frame(m_sockhand, tools_farmeName[id - 1].c_str(), 1);
            if (ret == 0)
            {

                return true;
            }
            else
            {
                std::cout << "切换工具坐标系失败: " << ret << std::endl;
                return false;
            }
        }
        else
        {
            std::cout << "获取机械臂状态失败" << std::endl;
            return false;
        }
    }
    ///////////////////////////////手眼标定
    PoseMatrix TCPCommunication_Hand::GetHandEyeCalibrationResultX_EyeInHand()
    {
        PoseMatrix result;
        calibration::HandEyeCalibration handEyeDeal;
        result << handEyeDeal.GetCalibrationResult_EyeInHand();
        Calibration_Result_EyeInHand << result;
        std::cout << "Calibration_Result_EyeInHand = " << Calibration_Result_EyeInHand << std::endl;
        return result;
    }
    float TCPCommunication_Hand::Getry_pose(float vec_x, float vec_y, float vec_z)
    {
        Pose ori, ori_vec;
        ori.position.x = 0;
        ori.position.y = 0;
        ori.position.z = 0;
        ori.euler.rx = ori.euler.ry = ori.euler.rz = 0;

        ori_vec.position.x = vec_x;
        ori_vec.position.y = vec_y;
        ori_vec.position.z = vec_z;
        ori_vec.euler.rx = ori_vec.euler.ry = ori_vec.euler.rz = 0;

        Pose robot;
        robot.position.x = 0.382;
        robot.position.y = 0;
        robot.position.z = 0.496;
        robot.euler.rx = 3.142;
        robot.euler.ry = 1.571;
        robot.euler.rz = 0;

        Pose Ori, End_V;
        Ori = coorTrans.GetGraspPose(ori, robot, Calibration_Result_EyeInHand);
        End_V = coorTrans.GetGraspPose(ori_vec, robot, Calibration_Result_EyeInHand);

        float rz = atan2(End_V.position.y - Ori.position.y, End_V.position.x - Ori.position.x);
        if (-3.1415926 < rz && rz < -1.5707963)
            rz += 3.1415926;
        else if (1.5707963 < rz && rz < 3.1415926)
            rz -= 3.1415926;

        return rz;
    }
} // namespace AIMOcommunicate
