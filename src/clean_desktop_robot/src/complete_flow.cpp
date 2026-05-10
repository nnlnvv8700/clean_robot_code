#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <thread> 
#include <cmath>
#include <string>

double tag_x = 0.0;
double tag_y = 0.0;
double tag_yaw = 0.0;
int count = 0;
int re_grab_count = 0;
int grab_flag = 0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void logBlock(const std::string &level, const std::string &title, const std::string &detail)
{
    const std::string line = "============================================================";
    if (level == "ERROR" || level == "错误")
    {
        ROS_ERROR("%s", line.c_str());
        ROS_ERROR("[导航][%s] %s", level.c_str(), title.c_str());
        ROS_ERROR("%s", detail.c_str());
        ROS_ERROR("%s", line.c_str());
    }
    else if (level == "WARN" || level == "警告")
    {
        ROS_WARN("%s", line.c_str());
        ROS_WARN("[导航][%s] %s", level.c_str(), title.c_str());
        ROS_WARN("%s", detail.c_str());
        ROS_WARN("%s", line.c_str());
    }
    else
    {
        ROS_INFO("%s", line.c_str());
        ROS_INFO("[导航][%s] %s", level.c_str(), title.c_str());
        ROS_INFO("%s", detail.c_str());
        ROS_INFO("%s", line.c_str());
    }
}

double yawFromQuaternion(const geometry_msgs::Quaternion &q_msg)
{
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

void sleep(double second)
{
    ros::Duration(second).sleep();
}

void printRobotPose(tf2_ros::Buffer &tfBuffer)
{
    try
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("base_link", "tag_1", ros::Time(0), ros::Duration(1.0));

        double x = transformStamped.transform.translation.x;
        double y = transformStamped.transform.translation.y;

        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        if (tag_x != x and tag_y != y)
        {
        	tag_x = x;
        	tag_y = y;
        	tag_yaw = atan2(y, x) * 180.0 / M_PI;
		count = 0;
        	grab_flag = 0;
        }
        else
        {
        	count++;
        	if(count > 10)
        	{
         		grab_flag = 1;
         		count = 0;
         		
         	}
        }

        ROS_INFO("Current Pose: x=%f, y=%f, yaw=%f degrees, grab=%d", x, y, tag_yaw, grab_flag);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform map to base_link: %s", ex.what());
    }
}

void printRobotPoseLoop(tf2_ros::Buffer *tfBuffer)
{
    ros::Rate rate(10); 
    while (ros::ok())
    {
        printRobotPose(*tfBuffer);
        rate.sleep();
    }
}

bool clearCostmaps(ros::NodeHandle &nh)
{
    ros::ServiceClient clear_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty empty_srv;

    if (!clear_client.waitForExistence(ros::Duration(2.0)))
    {
        logBlock("警告", "清理代价地图服务不存在",
                 "服务: /move_base/clear_costmaps\n建议检查: move_base 是否已经启动。");
        return false;
    }

    if (!clear_client.call(empty_srv))
    {
        logBlock("警告", "清理代价地图调用失败",
                 "服务调用失败。\n建议检查: move_base 日志、costmap 插件是否正常加载。");
        return false;
    }

    ROS_INFO("[导航][正常] move_base 代价地图已清理。");
    ros::Duration(0.3).sleep();
    return true;
}

void publishSafeCmdVel(ros::Publisher &cmd_pub, double linear_x, double angular_z)
{
    geometry_msgs::Twist vel_msg;

    if (linear_x > 0.12) linear_x = 0.12;
    if (linear_x < -0.12) linear_x = -0.12;
    if (angular_z > 0.35) angular_z = 0.35;
    if (angular_z < -0.35) angular_z = -0.35;

    vel_msg.linear.x = linear_x;
    vel_msg.angular.z = angular_z;
    cmd_pub.publish(vel_msg);
}

void stopRobot(ros::Publisher &cmd_pub)
{
    publishSafeCmdVel(cmd_pub, 0.0, 0.0);
}

void runSmallRecoveryMotion(ros::Publisher &cmd_pub)
{
    ros::Rate rate(10);

    logBlock("警告", "开始手动脱困动作",
             "动作: 短距离后退，停止，然后小角度原地旋转。\n安全限制: 低速短时动作，不长期绕过 move_base 手动开车。");

    // 小幅后退约0.2m，速度很低，结束后强制发布0速度。
    for (int i = 0; ros::ok() && i < 20; ++i)
    {
        publishSafeCmdVel(cmd_pub, -0.10, 0.0);
        rate.sleep();
    }
    stopRobot(cmd_pub);
    ros::Duration(0.2).sleep();

    // 小角度原地旋转，帮助脱离局部不可行轨迹。
    for (int i = 0; ros::ok() && i < 8; ++i)
    {
        publishSafeCmdVel(cmd_pub, 0.0, 0.30);
        rate.sleep();
    }
    stopRobot(cmd_pub);
    ros::Duration(0.2).sleep();
    ROS_WARN("[导航][恢复] 手动脱困动作结束，已发布 0 速度。");
}

bool waitForValidTf(tf2_ros::Buffer &tfBuffer, geometry_msgs::TransformStamped &robot_pose, std::string &base_frame)
{
    const std::string frames[] = {"base_footprint", "base_link"};
    const ros::Time start_time = ros::Time::now();

    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 8.0)
    {
        for (int i = 0; i < 2; ++i)
        {
            try
            {
                robot_pose = tfBuffer.lookupTransform("map", frames[i], ros::Time(0), ros::Duration(0.2));
                base_frame = frames[i];

                const double x = robot_pose.transform.translation.x;
                const double y = robot_pose.transform.translation.y;
                if (std::isfinite(x) && std::isfinite(y) && std::fabs(x) < 20.0 && std::fabs(y) < 20.0)
                {
                    return true;
                }

                ROS_WARN("[导航][TF] 机器人位姿异常 map -> %s: x=%.3f y=%.3f", base_frame.c_str(), x, y);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN_THROTTLE(1.0, "[导航][TF] 正在等待 map -> %s TF: %s", frames[i].c_str(), ex.what());
            }
        }

        ros::Duration(0.2).sleep();
    }

    logBlock("错误", "TF 未就绪",
             "无法获取 map -> base_footprint 或 map -> base_link。\n建议检查: AMCL、odom 坐标系、robot_pose_ekf、TF 树。");
    return false;
}

double poseDistance(const geometry_msgs::TransformStamped &a, const geometry_msgs::TransformStamped &b)
{
    const double dx = a.transform.translation.x - b.transform.translation.x;
    const double dy = a.transform.translation.y - b.transform.translation.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool navigateToGoalWithRecovery(MoveBaseClient &ac,
                                ros::NodeHandle &nh,
                                ros::Publisher &cmd_pub,
                                tf2_ros::Buffer &tfBuffer,
                                move_base_msgs::MoveBaseGoal goal,
                                const std::string &goal_name,
                                double timeout_sec = 45.0,
                                double stuck_timeout_sec = 8.0)
{
    while (ros::ok() && !ac.waitForServer(ros::Duration(2.0)))
    {
        ROS_WARN("[导航][等待] 执行 %s 前，正在等待 move_base action server...", goal_name.c_str());
    }

    logBlock("信息", "开始导航",
             "目标: " + goal_name + "\n恢复策略: 清理代价地图后重试；短距离后退/旋转后重试；第三次失败返回 false。");

    for (int attempt = 1; ros::ok() && attempt <= 3; ++attempt)
    {
        geometry_msgs::TransformStamped current_pose;
        geometry_msgs::TransformStamped last_motion_pose;
        std::string base_frame;

        if (!waitForValidTf(tfBuffer, current_pose, base_frame))
        {
            stopRobot(cmd_pub);
            return false;
        }

        clearCostmaps(nh);

        goal.target_pose.header.stamp = ros::Time::now();
        const double current_yaw = yawFromQuaternion(current_pose.transform.rotation);
        const double target_yaw = yawFromQuaternion(goal.target_pose.pose.orientation);
        ROS_INFO("------------------------------------------------------------");
        ROS_INFO("[导航][目标] %s | 第 %d/3 次尝试", goal_name.c_str(), attempt);
        ROS_INFO("[导航][当前位姿] map->%s: x=%.3f y=%.3f yaw=%.2f 度",
                 base_frame.c_str(),
                 current_pose.transform.translation.x,
                 current_pose.transform.translation.y,
                 current_yaw * 180.0 / M_PI);
        ROS_INFO("[导航][目标位姿] map: x=%.3f y=%.3f yaw=%.2f 度",
                 goal.target_pose.pose.position.x,
                 goal.target_pose.pose.position.y,
                 target_yaw * 180.0 / M_PI);
        ROS_INFO("[导航][判定参数] 总超时=%.1fs 卡住判定时间=%.1fs 卡住位移阈值=0.03m",
                 timeout_sec, stuck_timeout_sec);
        ROS_INFO("------------------------------------------------------------");

        ac.sendGoal(goal);

        const ros::Time start_time = ros::Time::now();
        ros::Time last_motion_time = ros::Time::now();
        last_motion_pose = current_pose;
        ros::Rate rate(2);
        bool should_retry = false;

        while (ros::ok())
        {
            const actionlib::SimpleClientGoalState state = ac.getState();
            if (state.isDone())
            {
                if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    logBlock("信息", "导航成功",
                             "目标: " + goal_name + "\n结果: 第 " + std::to_string(attempt) + " 次尝试到达。");
                    return true;
                }

                logBlock("警告", "move_base 返回失败",
                         "目标: " + goal_name + "\n状态: " + state.toString() +
                         "\n建议检查 RViz: 局部路径、全局路径、local_costmap、机器人 footprint。");
                should_retry = true;
                break;
            }

            if ((ros::Time::now() - start_time).toSec() > timeout_sec)
            {
                logBlock("警告", "导航超时",
                         "目标: " + goal_name +
                         "\n原因: move_base 在限定时间内没有完成。\n建议检查: planner_frequency、controller_frequency、局部规划器日志。");
                ac.cancelGoal();
                stopRobot(cmd_pub);
                should_retry = true;
                break;
            }

            geometry_msgs::TransformStamped live_pose;
            std::string live_base_frame;
            if (waitForValidTf(tfBuffer, live_pose, live_base_frame))
            {
                if (poseDistance(live_pose, last_motion_pose) > 0.03)
                {
                    last_motion_pose = live_pose;
                    last_motion_time = ros::Time::now();
                }
                else if ((ros::Time::now() - last_motion_time).toSec() > stuck_timeout_sec)
                {
                    logBlock("警告", "机器人疑似卡住",
                             "目标: " + goal_name +
                             "\n原因: 机器人位姿长时间变化小于 0.03m。\n建议检查: 机器人周围 local_costmap、footprint、膨胀半径、/cmd_vel 是否输出。");
                    ac.cancelGoal();
                    stopRobot(cmd_pub);
                    should_retry = true;
                    break;
                }
            }

            ROS_INFO_THROTTLE(2.0, "[导航][运行中] %s | move_base状态=%s | 已用时间=%.1fs",
                              goal_name.c_str(),
                              state.toString().c_str(),
                              (ros::Time::now() - start_time).toSec());
            rate.sleep();
        }

        if (!should_retry)
        {
            break;
        }

        if (attempt == 1)
        {
            logBlock("警告", "恢复策略 1",
                     "目标: " + goal_name + "\n动作: 清理代价地图，然后重新发送同一个目标。");
            clearCostmaps(nh);
        }
        else if (attempt == 2)
        {
            logBlock("警告", "恢复策略 2",
                     "目标: " + goal_name + "\n动作: 短距离后退 + 小角度旋转，清理代价地图，然后重新发送目标。");
            runSmallRecoveryMotion(cmd_pub);
            clearCostmaps(nh);
        }
    }

    logBlock("错误", "导航三次尝试后失败",
             "目标: " + goal_name +
             "\n结果: 返回 false，并停止机器人。\n优先检查: TF 树、初始位姿、local_costmap、footprint、膨胀半径、激光雷达数据。");
    stopRobot(cmd_pub);
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    // TF listener setup
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    std::thread tf_thread(printRobotPoseLoop, &tfBuffer);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    // Parameters
    double grab_1_x = 1.90, grab_1_y = -1.83;
    double grab_2_x = 1.90, grab_2_y = -3.05;
    double offset_left = 0.4, offset_right = 0.4;

    nh.getParam("/complete_flow_node/grab_1_x", grab_1_x);
    nh.getParam("/complete_flow_node/grab_1_y", grab_1_y);
    nh.getParam("/complete_flow_node/grab_2_x", grab_2_x);
    nh.getParam("/complete_flow_node/grab_2_y", grab_2_y);
    nh.getParam("/complete_flow_node/offset_left", offset_left);
    nh.getParam("/complete_flow_node/offset_right", offset_right);

    // 使用到的 service
    ros::ServiceClient arm_move_open_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_open");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/grab_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");
    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_close");
    upros_message::ArmPosition move_srv;

    move_base_msgs::MoveBaseGoal goal;
    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel_msg;
    int count = 0;


    vel_msg.linear.x = 0.5;
    count = 0;
    while (ros::ok() && count < 8)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);




    // ---------------------- Goal 1 (Grab)
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = grab_1_x;
    goal.target_pose.pose.position.y = grab_1_y;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.stamp = ros::Time::now();
    // printRobotPose(tfBuffer);
    bool goal_1_reached = navigateToGoalWithRecovery(ac, nh, pub, tfBuffer, goal, "Goal 1 Grab");
    
    const double search_speed =0.2;  // 统一的搜索速度
    const double return_speed =0.2; // 统一的返回速度

    if (goal_1_reached)
    {
        ROS_INFO("Goal 1 Reached!");
        
        vel_msg.linear.x = 0.07;   //0.07
        count = 0;
        while (ros::ok())
        {
            if (tag_x >= 0.31)  //0.31
		{
			pub.publish(vel_msg);
			loop_rate.sleep();
		}
		else
		{
			vel_msg.linear.x = 0.0;
			pub.publish(vel_msg);
			break;
		}
        }
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);

        while(ros::ok())
        {
		if (grab_flag == 0 and re_grab_count <= 4 and tag_x != 0.0)
		{
			ROS_INFO("retry");
			if (tag_x >= 0.25)
			{
				vel_msg.linear.x = 0.1;
				count = 0;
				while (ros::ok() && count < 5)
				{
				    pub.publish(vel_msg);
				    loop_rate.sleep();
				    count++;
				}
				vel_msg.linear.x = 0.0;
				pub.publish(vel_msg);
			}
			else
			{
				vel_msg.linear.x = -0.1;
				count = 0;
				while (ros::ok() && count < 5)
				{
				    pub.publish(vel_msg);
				    loop_rate.sleep();
				    count++;
				}
				vel_msg.linear.x = 0.0;
				pub.publish(vel_msg);
			}
			system("roslaunch clean_desktop_robot arm_grab.launch");
			re_grab_count++;
		}    
		else
		{
			break;
		}
	}	
        
    }
    else
    {
        ROS_WARN("Goal 1 Failed!");
    }




    // ---------------------- Backward after grab
    vel_msg.linear.x = -0.3;
    count = 0;
    while (ros::ok() && count < 14)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);



    // ---------------------- Backward after grab
    // vel_msg.angular.z = -1.5;
    // count = 0;
    // while (ros::ok() && count < 12)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    //     count++;
    // }
    // vel_msg.angular.z = 0.0;
    // pub.publish(vel_msg);


    // // ---------------------- Backward after grab
    // vel_msg.linear.x = 0.2;
    // count = 0;
    // while (ros::ok() && count < 30)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    //     count++;
    // }
    // vel_msg.linear.x = 0.0;
    // pub.publish(vel_msg);



    // // ---------------------- Backward after grab
    // vel_msg.angular.z = 1.5;
    // count = 0;
    // while (ros::ok() && count < 12)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    //     count++;
    // }
    // vel_msg.angular.z = 0.0;
    // pub.publish(vel_msg);

    // system("roslaunch clean_desktop_robot arm_put.launch");




    // ---------------------- Goal 1 (Place)
    goal.target_pose.pose.position.y = grab_1_y - offset_right;
    goal.target_pose.header.stamp = ros::Time::now();

    bool goal_1_trash_reached = navigateToGoalWithRecovery(ac, nh, pub, tfBuffer, goal, "Goal 1 Trash");

    if (goal_1_trash_reached)
    {
        ROS_INFO("Goal 1 Trash Reached!");
        vel_msg.linear.x = 0.1;
        count = 0;
        while (ros::ok() && count < 22)  //22
        {
            pub.publish(vel_msg);
            loop_rate.sleep();
            count++;
        }
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        system("roslaunch clean_desktop_robot arm_put.launch");
    }



    vel_msg.linear.x = -0.3;
    count = 0;
    while (ros::ok() && count < 16)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);
    
    tag_x = 0.0;
    tag_y = 0.0;
    tag_yaw = 0.0;





    // ---------------------- Goal 2 (Grab)
    goal.target_pose.pose.position.x = grab_2_x;
    goal.target_pose.pose.position.y = grab_2_y;
    goal.target_pose.header.stamp = ros::Time::now();

    bool goal_2_reached = navigateToGoalWithRecovery(ac, nh, pub, tfBuffer, goal, "Goal 2 Grab");
    
    

    if (goal_2_reached)
    {
    	 ROS_INFO("Goal 2 Reached!");
	 
        vel_msg.linear.x = 0.07;
        count = 0;
        while (ros::ok())
        {
            if (tag_x >= 0.29)  //0.29
		{
			pub.publish(vel_msg);
			loop_rate.sleep();
		}
		else
		{
			vel_msg.linear.x = 0.0;
			pub.publish(vel_msg);
			break;
		}
        }
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        system("roslaunch clean_desktop_robot arm_grab.launch");
        while(ros::ok())
        {
		if (grab_flag == 0 and re_grab_count <= 4 and tag_x != 0.0)
		{
			ROS_INFO("retry");
			if (tag_x >= 0.25)
			{
				vel_msg.linear.x = 0.1;
				count = 0;
				while (ros::ok() && count < 5)
				{
				    pub.publish(vel_msg);
				    loop_rate.sleep();
				    count++;
				}
				vel_msg.linear.x = 0.0;
				pub.publish(vel_msg);
			}
			else
			{
				vel_msg.linear.x = -0.1;
				count = 0;
				while (ros::ok() && count < 5)
				{
				    pub.publish(vel_msg);
				    loop_rate.sleep();
				    count++;
				}
				vel_msg.linear.x = 0.0;
				pub.publish(vel_msg);
			}
			system("roslaunch clean_desktop_robot arm_grab.launch");
			re_grab_count++;
		}    
		else
		{
			break;
		}
	}
    }





    // vel_msg.linear.x = -0.3;
    // count = 0;
    // while (ros::ok() && count < 14)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    //     count++;
    // }
    // vel_msg.linear.x = 0.0;
    // pub.publish(vel_msg);




    // ---------------------- Goal 2 (Place)
    goal.target_pose.pose.position.y = grab_2_y + offset_left;
    goal.target_pose.header.stamp = ros::Time::now();

    bool goal_2_trash_reached = navigateToGoalWithRecovery(ac, nh, pub, tfBuffer, goal, "Goal 2 Trash");

    if (goal_2_trash_reached)
    {
        ROS_INFO("Goal 2 Trash Reached!");
        vel_msg.linear.x = 0.1;
        count = 0;
        while (ros::ok() && count < 24)   //24
        {
            pub.publish(vel_msg);
            loop_rate.sleep();
            count++;
        }
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
        system("roslaunch clean_desktop_robot arm_put.launch");
    }

    vel_msg.linear.x = -0.5;
    count = 0;
    while (ros::ok() && count < 50)    //50
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);















    // ---------------------- Return Home 1
    //goal.target_pose.pose.position.x = 2.1;
    //goal.target_pose.pose.position.y = 1.6;
    //goal.target_pose.pose.orientation.z = 0.81271;
    ///goal.target_pose.pose.orientation.w = 0.58267;
    ///goal.target_pose.header.stamp = ros::Time::now();

    //ac.sendGoal(goal);
    //ROS_INFO("Send Goal Home 1 !!!");
    //ac.waitForResult();

    //if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //{
        //ROS_INFO("Back to Home 1!");
    //}
    
    















    // ---------------------- Return Home 2
    goal.target_pose.pose.position.x = 0.2;
    goal.target_pose.pose.position.y = 0.2;
    goal.target_pose.pose.orientation.z = 0.92015;
    goal.target_pose.pose.orientation.w = -0.39157;
    goal.target_pose.header.stamp = ros::Time::now();

    bool home_reached = navigateToGoalWithRecovery(ac, nh, pub, tfBuffer, goal, "Return Home 2");

    if (home_reached)
    {
        ROS_INFO("Back to Home 2!");
    }

    vel_msg.linear.x = 0.2;
    count = 0;
    while (ros::ok() && count < 13)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }

    return 0;
}

