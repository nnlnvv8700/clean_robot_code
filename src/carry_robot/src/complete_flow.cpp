#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <cmath>

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sleep(double second)
{
    ros::Duration(second).sleep();
}

// 停止机器人并置零速度
void stopRobot(const ros::Publisher &pub, geometry_msgs::Twist &vel_msg)
{
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);
}

// 清理 costmap，帮助 TEB 重新规划
bool clearCostmaps(ros::NodeHandle &nh)
{
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty srv;
    if (!client.waitForExistence(ros::Duration(1.0)))
    {
        ROS_WARN("clear_costmaps service not available");
        return false;
    }
    if (client.call(srv))
    {
        ROS_INFO("Costmap cleared");
        ros::Duration(0.3).sleep();
        return true;
    }
    ROS_WARN("Failed to call clear_costmaps");
    return false;
}

// 设置导航目标朝向，使其面向目标点（假设从近似原点出发）
void setGoalOrientation(move_base_msgs::MoveBaseGoal &goal, double target_x, double target_y)
{
    double yaw = atan2(target_y, target_x);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();
}

// 发送导航目标，带清理 + 重试
bool sendGoalWithRetry(MoveBaseClient &ac, ros::NodeHandle &nh,
                       move_base_msgs::MoveBaseGoal &goal,
                       const std::string &desc, double first_clear_delay = 0.0)
{
    for (int attempt = 1; attempt <= 2; attempt++)
    {
        if (first_clear_delay > 0)
            ros::Duration(first_clear_delay).sleep();

        clearCostmaps(nh);
        goal.target_pose.header.stamp = ros::Time::now();

        ROS_INFO("[%s] Attempt %d/2: x=%.3f y=%.3f", desc.c_str(), attempt,
                 goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        ac.sendGoal(goal);

        if (ac.waitForResult(ros::Duration(30.0)))
        {
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("[%s] SUCCEEDED!", desc.c_str());
                return true;
            }
        }
        else
        {
            ROS_WARN("[%s] Timeout on attempt %d", desc.c_str(), attempt);
            ac.cancelGoal();
        }

        if (attempt == 1)
        {
            ROS_WARN("[%s] Retrying after clear costmaps...", desc.c_str());
            clearCostmaps(nh);
        }
    }

    ROS_ERROR("[%s] FAILED after 2 attempts", desc.c_str());
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    MoveBaseClient ac("move_base", true);

    ac.waitForServer();

    double grab_desk_x = 1.90;  // 抓取的桌子的x坐标
    double grab_desk_y = -1.80; // 抓取的桌子的y坐标

    double tag_1_put_x = 1.0; // 放置tag1的x坐标
    double tag_1_put_y = -0.77; // 放置tag1的y坐标

    double tag_2_put_x = 1.6; // 放置tag2的x坐标
    double tag_2_put_y = -0.77; // 放置tag2的y坐标

    double offset_left = 0.1; // tag1在桌子中轴线左侧10cm
    double offset_right = 0.1; // tag2在桌子中轴线右侧10cm

    // 从参数服务器覆盖默认值（由 launch 文件传入）
    nh.getParam("/w4a_complete_flow_node/grab_desk_x", grab_desk_x);
    nh.getParam("/w4a_complete_flow_node/grab_desk_y", grab_desk_y);
    nh.getParam("/w4a_complete_flow_node/tag_1_put_x", tag_1_put_x);
    nh.getParam("/w4a_complete_flow_node/tag_1_put_y", tag_1_put_y);
    nh.getParam("/w4a_complete_flow_node/tag_2_put_x", tag_2_put_x);
    nh.getParam("/w4a_complete_flow_node/tag_2_put_y", tag_2_put_y);
    nh.getParam("/w4a_complete_flow_node/offset_left", offset_left);
    nh.getParam("/w4a_complete_flow_node/offset_right", offset_right);

    move_base_msgs::MoveBaseGoal goal;

    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel_msg;

    // ==================== Step 1: Navigate to Grab Desk (TAG1) ====================
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = grab_desk_x;
    goal.target_pose.pose.position.y = grab_desk_y + offset_left;
    setGoalOrientation(goal, grab_desk_x, grab_desk_y + offset_left);

    if (sendGoalWithRetry(ac, nh, goal, "Grab TAG1", 1.0))
    {
        // 前进10cm到桌前
        vel_msg.linear.x = 0.1;
        for (int i = 0; ros::ok() && i < 10; i++)
        {
            pub.publish(vel_msg);
            loop_rate.sleep();
        }
        stopRobot(pub, vel_msg);

        // 执行TAG1抓取
        system("roslaunch carry_robot arm_grab_1.launch");
    }
    else
    {
        ROS_WARN("Grab TAG1 failed, continuing...");
    }

    // 后退30cm
    vel_msg.linear.x = -0.1;
    for (int i = 0; ros::ok() && i < 30; i++)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
    }
    stopRobot(pub, vel_msg);

    // ==================== Step 2: Navigate to Place TAG1 ====================
    goal.target_pose.pose.position.x = tag_1_put_x;
    goal.target_pose.pose.position.y = tag_1_put_y;
    setGoalOrientation(goal, tag_1_put_x, tag_1_put_y);

    if (sendGoalWithRetry(ac, nh, goal, "Place TAG1"))
    {
        // 左移10cm
        vel_msg.linear.y = 0.1;
        for (int i = 0; ros::ok() && i < 10; i++)
        {
            pub.publish(vel_msg);
            loop_rate.sleep();
        }
        stopRobot(pub, vel_msg);

        system("roslaunch carry_robot arm_put.launch");
    }
    else
    {
        ROS_WARN("Place TAG1 failed, continuing...");
    }

    // 右移30cm
    vel_msg.linear.y = -0.1;
    for (int i = 0; ros::ok() && i < 30; i++)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
    }
    stopRobot(pub, vel_msg);

    // ==================== Step 3: Navigate to Grab Desk (TAG2) ====================
    goal.target_pose.pose.position.x = grab_desk_x;
    goal.target_pose.pose.position.y = grab_desk_y - offset_right;
    setGoalOrientation(goal, grab_desk_x, grab_desk_y - offset_right);

    if (sendGoalWithRetry(ac, nh, goal, "Grab TAG2"))
    {
        // 前进10cm
        vel_msg.linear.x = 0.1;
        for (int i = 0; ros::ok() && i < 10; i++)
        {
            pub.publish(vel_msg);
            loop_rate.sleep();
        }
        stopRobot(pub, vel_msg);

        system("roslaunch carry_robot arm_grab_2.launch");
    }
    else
    {
        ROS_WARN("Grab TAG2 failed, continuing...");
    }

    // 后退30cm
    vel_msg.linear.x = -0.1;
    for (int i = 0; ros::ok() && i < 30; i++)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
    }
    stopRobot(pub, vel_msg);

    // ==================== Step 4: Navigate to Place TAG2 ====================
    goal.target_pose.pose.position.x = tag_2_put_x;
    goal.target_pose.pose.position.y = tag_2_put_y;
    setGoalOrientation(goal, tag_2_put_x, tag_2_put_y);

    if (sendGoalWithRetry(ac, nh, goal, "Place TAG2"))
    {
        // 左移10cm
        vel_msg.linear.y = 0.1;
        for (int i = 0; ros::ok() && i < 10; i++)
        {
            pub.publish(vel_msg);
            loop_rate.sleep();
        }
        stopRobot(pub, vel_msg);

        system("roslaunch carry_robot arm_put.launch");
    }
    else
    {
        ROS_WARN("Place TAG2 failed, continuing...");
    }

    // 右移30cm
    vel_msg.linear.y = -0.1;
    for (int i = 0; ros::ok() && i < 30; i++)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
    }
    stopRobot(pub, vel_msg);

    // ==================== Step 5: Return Home ====================
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    setGoalOrientation(goal, 0.0, 0.0); // 朝向原点（identity）

    if (sendGoalWithRetry(ac, nh, goal, "Return Home"))
    {
        ROS_INFO("Back Home!");
    }
    else
    {
        ROS_WARN("Return Home failed");
    }

    return 0;
}
