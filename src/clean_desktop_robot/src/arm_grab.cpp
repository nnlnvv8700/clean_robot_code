#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "upros_message/ArmPosition.h"
#include <upros_message/SingleServo.h>
#include <upros_message/MultipleServo.h>
#include "std_srvs/Empty.h"
#include <ros/ros.h>

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mgrab_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    // 使用到的 service
    ros::ServiceClient arm_move_open_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_open");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/zero_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/grab_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/upros_arm_control/release_service");
    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/upros_arm_control/arm_pos_service_close");

    ros::Publisher single_joint_pub_;
    ros::Publisher multiple_joint_pub_;
    single_joint_pub_ = nh.advertise<upros_message::SingleServo>("/single_servo_topic", 10);
    multiple_joint_pub_ = nh.advertise<upros_message::MultipleServo>("/multiple_servo_topic", 10);
    upros_message::SingleServo single_servo;
    std::vector<int> servo_bias_ = {0, 0, 0, 0, 0, 0};

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ROS_INFO("tf coordinate transformaing....");

    // 获取tag到机械臂基坐标的坐标变换
    geometry_msgs::TransformStamped tfs_1;
    try
    {
        ROS_INFO("Waiting for tag 'tag36h11:1' in TF tree (max 30s)...");
        if (!buffer.canTransform("arm_base_link", "tag36h11:1", ros::Time(0), ros::Duration(30.0)))
        {
            ROS_ERROR("Tag 'tag36h11:1' not detected within 30s.\n"
                      "  Check: (1) AprilTag detector is running\n"
                      "         (2) Tag family is tag36h11\n"
                      "         (3) Tag 1 is in camera FOV");
            ros::shutdown();
            return 1;
        }
        tfs_1 = buffer.lookupTransform("arm_base_link", "tag36h11:1", ros::Time(0));
        ROS_INFO("Transform for 'tag36h11:1' OK");
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("TF lookup failed for 'tag36h11:1': %s", ex.what());
        ros::shutdown();
        return 1;
    }

    // 单位转换，ros坐标系到逆运算坐标系
    int x = -int(tfs_1.transform.translation.y * 1000);
    int y = int(tfs_1.transform.translation.x * 1000) + 30;
    int z = int(tfs_1.transform.translation.z * 1000 + 40);
    
    ROS_INFO("Target X: %d, Target Y: %d, Target Z: %d", x, y, z);
    std_srvs::Empty empty_srv;
    
    // 第一步，打开夹爪
    arm_release_client.call(empty_srv);
    sleep(0.6);  //3.0

    // 第二步，运动到抓取位置
    upros_message::ArmPosition move_srv;
    move_srv.request.x = x;
    move_srv.request.y = y;
    move_srv.request.z = z;
    arm_move_open_client.call(move_srv);
    sleep(1.5);  //3.0


    // 第三步，闭合夹爪
    arm_grab_client.call(empty_srv);
    sleep(1.0);  //3.0

    // 第四步，返回零位
    arm_zero_client.call(empty_srv);
    sleep(1.5);

            
    // upros_message::MultipleServo multiple_servo;
    // for (int i = 0; i < 6; i++)
    // {
    //             upros_message::SingleServo single_servo;
    //             single_servo.ID = i + 1;
    //             single_servo.Rotation_Speed = 150;
                
    //             if(i == 1){
    //                 single_servo.Target_position_Angle = -800;
    //             }
    //             if(i == 5){
    //                 single_servo.Target_position_Angle = -400;
    //             }
    //             multiple_servo.servo_gather.push_back(single_servo);
    // }
    // multiple_joint_pub_.publish(multiple_servo);

    // sleep(1.5);

    ros::shutdown();

    return 0;
}
