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
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Config.h>
#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <thread> 
#include <cmath>
#include <string>
#include <mutex>
#include <limits>
#include <algorithm>

// 全局变量和TF反馈

double tag_x = 0.0;
double tag_y = 0.0;
double tag_yaw = 0.0;
int count = 0;
int re_grab_count = 0;
int grab_flag = 0;
ros::Time last_tag_time;
std::mutex tag_mutex;

double front_min_range = std::numeric_limits<double>::infinity();
ros::Time last_scan_time;
std::mutex scan_mutex;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sleep(double second)
{
    ros::Duration(second).sleep();
}

//printRobotPose和循环监听函数

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
        
        // 稳定性判断

        {
            std::lock_guard<std::mutex> lock(tag_mutex);
            if (std::fabs(tag_x - x) > 0.003 || std::fabs(tag_y - y) > 0.003)
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
            last_tag_time = ros::Time::now();
        }

        ROS_INFO_THROTTLE(1.0, "Current tag pose: x=%f, y=%f, yaw=%f degrees, stable=%d", x, y, tag_yaw, grab_flag);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_THROTTLE(1.0, "Could not transform base_link to tag_1: %s", ex.what());
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

// 导航和靠桌阶段的稳定性辅助逻辑：只做清图、重试、限速和传感器保护，不改变比赛主流程。

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    double min_range = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        const double angle = scan->angle_min + i * scan->angle_increment;
        const double range = scan->ranges[i];
        if (std::isfinite(range) && std::fabs(angle) < 0.35)
        {
            min_range = std::min(min_range, range);
        }
    }

    std::lock_guard<std::mutex> lock(scan_mutex);
    front_min_range = min_range;
    last_scan_time = scan->header.stamp.isZero() ? ros::Time::now() : scan->header.stamp;
}

bool hasFreshTag(double timeout_sec)
{
    std::lock_guard<std::mutex> lock(tag_mutex);
    return !last_tag_time.isZero() && (ros::Time::now() - last_tag_time).toSec() <= timeout_sec && tag_x > 0.0;
}

double getTagX()
{
    std::lock_guard<std::mutex> lock(tag_mutex);
    return tag_x;
}

int getGrabFlag()
{
    std::lock_guard<std::mutex> lock(tag_mutex);
    return grab_flag;
}

void resetTagState()
{
    std::lock_guard<std::mutex> lock(tag_mutex);
    tag_x = 0.0;
    tag_y = 0.0;
    tag_yaw = 0.0;
    grab_flag = 0;
    count = 0;
    last_tag_time = ros::Time(0);
}

bool frontPathClear(double stop_distance)
{
    std::lock_guard<std::mutex> lock(scan_mutex);
    if (last_scan_time.isZero() || (ros::Time::now() - last_scan_time).toSec() > 1.0)
    {
        ROS_WARN_THROTTLE(1.0, "No fresh /scan_filtered data; stop forward fine approach.");
        return false;
    }
    if (!std::isfinite(front_min_range))
    {
        return true;
    }
    return front_min_range > stop_distance;
}

void publishLimitedTwist(ros::Publisher &pub, double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = std::max(-0.30, std::min(0.30, linear_x));
    msg.angular.z = std::max(-0.80, std::min(0.80, angular_z));
    pub.publish(msg);
}

void stopRobot(ros::Publisher &pub)
{
    publishLimitedTwist(pub, 0.0, 0.0);
}

void timedTwist(ros::Publisher &pub, ros::Rate &rate, double linear_x, double angular_z, int ticks, bool require_front_clear)
{
    for (int i = 0; ros::ok() && i < ticks; ++i)
    {
        if (require_front_clear && linear_x > 0.0 && !frontPathClear(0.28))
        {
            ROS_WARN("Forward motion blocked by front obstacle; stop manual fine motion.");
            break;
        }
        publishLimitedTwist(pub, linear_x, angular_z);
        rate.sleep();
    }
    stopRobot(pub);
}

bool approachTagUntil(ros::Publisher &pub, ros::Rate &rate, double target_x, double timeout_sec)
{
    const ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < timeout_sec)
    {
        if (!hasFreshTag(1.0))
        {
            ROS_WARN_THROTTLE(1.0, "Tag TF timeout during fine approach; stop.");
            stopRobot(pub);
            rate.sleep();
            continue;
        }
        if (!frontPathClear(0.26))
        {
            ROS_WARN("Front obstacle too close during fine approach; stop before collision.");
            stopRobot(pub);
            return false;
        }
        if (getTagX() < target_x)
        {
            stopRobot(pub);
            return true;
        }
        publishLimitedTwist(pub, 0.07, 0.0);
        rate.sleep();
    }
    stopRobot(pub);
    ROS_WARN("Fine approach timed out before reaching tag target distance %.2f.", target_x);
    return false;
}

bool waitForRobotPose(tf2_ros::Buffer &tfBuffer, std::string *base_frame)
{
    const std::string frames[] = {"base_footprint", "base_link"};
    const ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < 8.0)
    {
        for (const std::string &frame : frames)
        {
            try
            {
                geometry_msgs::TransformStamped tf = tfBuffer.lookupTransform("map", frame, ros::Time(0), ros::Duration(0.2));
                const double x = tf.transform.translation.x;
                const double y = tf.transform.translation.y;
                if (std::isfinite(x) && std::isfinite(y) && std::fabs(x) < 20.0 && std::fabs(y) < 20.0)
                {
                    *base_frame = frame;
                    ROS_INFO("Robot pose ready: map -> %s, x=%.3f, y=%.3f", frame.c_str(), x, y);
                    return true;
                }
                ROS_WARN("Robot pose looks unreasonable: map -> %s, x=%.3f, y=%.3f", frame.c_str(), x, y);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN_THROTTLE(1.0, "Waiting for map -> %s TF: %s", frame.c_str(), ex.what());
            }
        }
        ros::Duration(0.2).sleep();
    }
    ROS_ERROR("Robot pose TF is not ready before navigation.");
    return false;
}

bool clearMoveBaseCostmaps(ros::NodeHandle &nh)
{
    ros::ServiceClient clear_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty empty_srv;
    if (!clear_client.waitForExistence(ros::Duration(2.0)))
    {
        ROS_WARN("/move_base/clear_costmaps service is not available.");
        return false;
    }
    if (!clear_client.call(empty_srv))
    {
        ROS_WARN("Failed to call /move_base/clear_costmaps.");
        return false;
    }
    ros::Duration(0.3).sleep();
    return true;
}

bool sendGoalWithRecovery(MoveBaseClient &ac, ros::NodeHandle &nh, ros::Publisher &pub, ros::Rate &rate,
                          tf2_ros::Buffer &tfBuffer, move_base_msgs::MoveBaseGoal goal,
                          const std::string &name, double timeout_sec)
{
    std::string base_frame;
    if (!waitForRobotPose(tfBuffer, &base_frame))
    {
        stopRobot(pub);
        return false;
    }

    for (int attempt = 1; ros::ok() && attempt <= 3; ++attempt)
    {
        clearMoveBaseCostmaps(nh);
        goal.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal);
        ROS_INFO("%s: sent move_base goal, attempt %d.", name.c_str(), attempt);

        const bool finished = ac.waitForResult(ros::Duration(timeout_sec));
        if (!finished)
        {
            ROS_WARN("%s: move_base timeout after %.1f seconds, state=%s.", name.c_str(), timeout_sec, ac.getState().toString().c_str());
            ac.cancelGoal();
            stopRobot(pub);
        }
        else if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("%s: goal reached.", name.c_str());
            return true;
        }
        else
        {
            ROS_WARN("%s: move_base failed, state=%s.", name.c_str(), ac.getState().toString().c_str());
        }

        if (attempt == 1)
        {
            ROS_WARN("%s: recovery 1, clear costmaps and retry.", name.c_str());
        }
        else if (attempt == 2)
        {
            ROS_WARN("%s: recovery 2, small reverse and rotation before retry.", name.c_str());
            timedTwist(pub, rate, -0.10, 0.0, 8, false);
            timedTwist(pub, rate, 0.0, 0.35, 8, false);
        }
    }

    ROS_ERROR("%s: failed after 3 attempts; stop and let task flow decide next step.", name.c_str());
    stopRobot(pub);
    return false;
}

// 临时清理代价地图

bool setCostmapLayerEnabled(ros::NodeHandle &nh, const std::string &service_name, bool enabled)
{
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::BoolParameter enabled_param;
    enabled_param.name = "enabled";
    enabled_param.value = enabled;
    srv.request.config.bools.push_back(enabled_param);

    ros::ServiceClient client = nh.serviceClient<dynamic_reconfigure::Reconfigure>(service_name);
    if (!client.waitForExistence(ros::Duration(1.0)))
    {
        ROS_WARN("Dynamic reconfigure service not available: %s", service_name.c_str());
        return false;
    }

    if (!client.call(srv))
    {
        ROS_WARN("Failed to set costmap layer: %s", service_name.c_str());
        return false;
    }

    return true;
}

void setReturnLocalObstacleBlindMode(ros::NodeHandle &nh, bool blind_mode)
{
    // Return-only mode: ignore local laser obstacles near the wall, then restore normal navigation.
    setCostmapLayerEnabled(nh, "/move_base/local_costmap/obstacle_layer/set_parameters", !blind_mode);
    clearMoveBaseCostmaps(nh);
}

// 主函数初始化

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    // TF listener setup
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    std::thread tf_thread(printRobotPoseLoop, &tfBuffer);
    tf_thread.detach();

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber scan_sub = nh.subscribe("/scan_filtered", 10, scanCallback);
    MoveBaseClient ac("move_base", true);
    while (ros::ok() && !ac.waitForServer(ros::Duration(2.0)))
    {
        ROS_WARN("Waiting for move_base action server...");
    }

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

    //出发区到第一张桌子

    timedTwist(pub, loop_rate, 0.20, 0.0, 8, true);




    // ---------------------- Goal 1 (Grab)
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = grab_1_x;
    goal.target_pose.pose.position.y = grab_1_y;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.stamp = ros::Time::now();
    // printRobotPose(tfBuffer);
    const bool goal1_ok = sendGoalWithRecovery(ac, nh, pub, loop_rate, tfBuffer, goal, "Goal 1 Grab", 45.0);
    
    const double search_speed =0.2;  // 统一的搜索速度
    const double return_speed =0.2; // 统一的返回速度

    bool table1_has_object = false;
    if (goal1_ok)
    {
        ROS_INFO("Goal 1 Reached!");
        
        const bool approach_ok = approachTagUntil(pub, loop_rate, 0.31, 8.0);
        if (approach_ok)
        {
            system("roslaunch clean_desktop_robot arm_grab.launch");
            table1_has_object = true;
        }

        while(ros::ok() && approach_ok)
        {
		if (getGrabFlag() == 0 and re_grab_count <= 4 and hasFreshTag(1.0))         //微调和重试
		{
			ROS_INFO("retry");
			if (getTagX() >= 0.25)
			{
				timedTwist(pub, loop_rate, 0.08, 0.0, 10, true);
			}
			else
			{
				timedTwist(pub, loop_rate, -0.08, 0.0, 5, false);
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
        ROS_WARN("Goal 1 Failed, skip table 1 grab and place.");
    }




    if (table1_has_object)
    {
    //第一次投放
    // ---------------------- Backward after grab
    timedTwist(pub, loop_rate, 0.0, -0.8, 9, false);


    // ---------------------- Backward after grab
    timedTwist(pub, loop_rate, 0.18, 0.0, 25, true);



    // ---------------------- Backward after grab
    timedTwist(pub, loop_rate, 0.0, 0.8, 12, false);




    // ---------------------- Backward after grab
    timedTwist(pub, loop_rate, 0.12, 0.0, 10, true);




    system("roslaunch clean_desktop_robot arm_put.launch");



















    // // ---------------------- Goal 1 (Place)
    // goal.target_pose.pose.position.y = grab_1_y - offset_right;
    // goal.target_pose.header.stamp = ros::Time::now();

    // ac.sendGoal(goal);
    // ROS_INFO("MoveBase Send Goal 1 Trash !!!");
    // ac.waitForResult();

    // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     ROS_INFO("Goal 1 Trash Reached!");
    //     vel_msg.linear.x = 0.1;
    //     count = 0;
    //     while (ros::ok() && count < 22)  //22
    //     {
    //         pub.publish(vel_msg);
    //         loop_rate.sleep();
    //         count++;
    //     }
    //     vel_msg.linear.x = 0.0;
    //     pub.publish(vel_msg);
    //     system("roslaunch clean_desktop_robot arm_put.launch");
    // }


    //撤离
    timedTwist(pub, loop_rate, -0.25, 0.0, 16, false);
    }
    
    resetTagState();
    re_grab_count = 0;




    //第二张桌子
    // ---------------------- Goal 2 (Grab)
    goal.target_pose.pose.position.x = grab_2_x;
    goal.target_pose.pose.position.y = grab_2_y;
    // Goal 2 grab heading: turn left 5 degrees from the original yaw=0 heading.
    goal.target_pose.pose.orientation.z = 0.04362;
    goal.target_pose.pose.orientation.w = 0.99905;
    goal.target_pose.header.stamp = ros::Time::now();

    const bool goal2_ok = sendGoalWithRecovery(ac, nh, pub, loop_rate, tfBuffer, goal, "Goal 2 Grab", 45.0);
    
    

    bool table2_has_object = false;
    if (goal2_ok)
    {
    	 ROS_INFO("Goal 2 Reached!");
	 
        const bool approach_ok = approachTagUntil(pub, loop_rate, 0.29, 8.0);
        if (approach_ok)
        {
            system("roslaunch clean_desktop_robot arm_grab.launch");
            table2_has_object = true;
        }
        while(ros::ok() && approach_ok)
        {
		if (getGrabFlag() == 0 and re_grab_count <= 4 and hasFreshTag(1.0))
		{
			ROS_INFO("retry");
			if (getTagX() >= 0.25)
			{
				timedTwist(pub, loop_rate, 0.08, 0.0, 10, true);
			}
			else
			{
				timedTwist(pub, loop_rate, -0.08, 0.0, 5, false);
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
        ROS_WARN("Goal 2 Failed, skip table 2 grab and place.");
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








    if (table2_has_object)
    {
    //第二次投放
    // ---------------------- Backward after grab
    timedTwist(pub, loop_rate, 0.0, 0.8, 10, false);


    // ---------------------- Backward after grab
    timedTwist(pub, loop_rate, 0.18, 0.0, 30, true);



    // ---------------------- Backward after grab
    timedTwist(pub, loop_rate, 0.0, -0.8, 11, false);


   // ---------------------- Backward after grab
    timedTwist(pub, loop_rate, 0.12, 0.0, 10, true);




    system("roslaunch clean_desktop_robot arm_put.launch");








    // // ---------------------- Goal 2 (Place)
    // goal.target_pose.pose.position.y = grab_2_y + offset_left;
    // goal.target_pose.header.stamp = ros::Time::now();

    // ac.sendGoal(goal);
    // ROS_INFO("MoveBase Send Goal 2 Trash !!!");
    // ac.waitForResult();

    // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     ROS_INFO("Goal 2 Trash Reached!");
    //     vel_msg.linear.x = 0.1;
    //     count = 0;
    //     while (ros::ok() && count < 24)   //24
    //     {
    //         pub.publish(vel_msg);
    //         loop_rate.sleep();
    //         count++;
    //     }
    //     vel_msg.linear.x = 0.0;
    //     pub.publish(vel_msg);
    //     system("roslaunch clean_desktop_robot arm_put.launch");
    // }


    //撤离

    timedTwist(pub, loop_rate, -0.25, 0.0, 30, false);
    }















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
    
    













    //返回出发区

    // ---------------------- Return Home 2
    // Return home: navigate to a safer point away from the wall, then drive straight into the start area.
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 0.4;
    goal.target_pose.pose.position.y = 0.4;
    goal.target_pose.pose.orientation.z = 0.93358;
    goal.target_pose.pose.orientation.w = -0.35837;
    goal.target_pose.header.stamp = ros::Time::now();

    // Return-only blind navigation: disable local laser obstacles before entering the wall-side start area.
    setReturnLocalObstacleBlindMode(nh, true);

    const bool home_ok = sendGoalWithRecovery(ac, nh, pub, loop_rate, tfBuffer, goal, "Return Home", 45.0);

    setReturnLocalObstacleBlindMode(nh, false);

    if (home_ok)
    {
        ROS_INFO("Back to Home 2!");
    }
    else
    {
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        pub.publish(vel_msg);
        return 0;
    }

    // Final entry: keep the reached heading and drive straight back into the start area.
    timedTwist(pub, loop_rate, 0.20, 0.0, 15, true);

    return 0;
}
