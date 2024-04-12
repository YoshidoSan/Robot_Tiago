#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>
#include <ctime>
#include <math.h>

using namespace std;

double x_odom = 0;
double y_odom = 0;
//double z_odom = 0;
double w_odom = 0;
double z_odom = 0;

double x_gaz = 0;
double y_gaz = 0;
//double z_gaz = 0;
double w_gaz = 0;
double z_gaz = 0;

double x_ide = 0;
double y_ide = 0;
double ang_ide = 0;
double pos_er = 0;
double pos_er_sum = 0;
double cal_er = 0;
double cal_er_sum = 0;

void pos_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    x_odom = msg->pose.pose.position.x;
    y_odom = msg->pose.pose.position.y;
    w_odom = msg->pose.pose.orientation.w;
    z_odom = msg->pose.pose.orientation.z;
}

void gazebo_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    x_gaz = msg->pose[1].position.x;
    y_gaz = msg->pose[1].position.y;
    w_gaz = msg->pose[1].orientation.w;
    z_gaz = msg->pose[1].orientation.z;
}

void ide_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    x_ide = msg->data[0];
    y_ide = msg->data[1];
    ang_ide = msg->data[2];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lab1_er");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    ros::Subscriber sub_odom = n.subscribe("/mobile_base_controller/odom", 1000, pos_callback);
    ros::Subscriber sub_gazebo = n.subscribe("/gazebo/model_states", 1000, gazebo_callback);
    ros::Subscriber sub_pose_est = n.subscribe("/est_val", 1000, ide_callback);

    ros::Publisher pos_er_pub = n.advertise<std_msgs::Float64>("/odometry_error", 1000);
    ros::Publisher pos_er_sum_pub = n.advertise<std_msgs::Float64>("/odometry_error_sum", 1000);
    ros::Publisher cal_er_pub = n.advertise<std_msgs::Float64>("/calcualted_error", 1000);
    ros::Publisher cal_er_sum_pub = n.advertise<std_msgs::Float64>("/calculated_error_sum", 1000);

    std_msgs::Float64 pos_er_msg;
    std_msgs::Float64 pos_er_sum_msg;
    std_msgs::Float64 cal_er_msg;
    std_msgs::Float64 cal_er_sum_msg;

    while (ros::ok())
    {
        double gaz_angle = atan2(2 * w_gaz * z_gaz, 1 - 2 * z_gaz * z_gaz);
        double odom_angle = atan2(2 * w_odom * z_odom, 1 - 2 * z_odom * z_odom);
        cout << "===============" << endl;
        cout << "Gaz: [x, y, angle]  " << x_gaz << " | " << y_gaz << " | " << gaz_angle << endl;
        cout << "Otom: [x, y, angle] " << x_odom << " | " << y_odom << " | " << odom_angle << endl;
        cout << "Est: [x, y, angle]  " << x_ide << " | " << y_ide << " | " << ang_ide << std::endl;

        pos_er = sqrt(pow((x_odom - x_gaz), 2) + pow((y_odom - y_gaz), 2) + pow((odom_angle - gaz_angle), 2));
        cout << "pos_er: " << pos_er << endl;

        cal_er = sqrt(pow((x_ide - x_gaz), 2) + pow((y_ide - y_gaz), 2) + pow((ang_ide - gaz_angle), 2));
        cout << "cal_er: " << cal_er << endl;

        pos_er_sum += pos_er;
        cout << "pos_er_sum: " << pos_er_sum << endl;

        cal_er_sum += cal_er;
        cout << "cal_er_sum: " << cal_er_sum << endl;

        cout << "===============" << endl;

        pos_er_msg.data = pos_er;
        pos_er_pub.publish(pos_er_msg);
        pos_er_sum_msg.data = pos_er_sum;
        pos_er_sum_pub.publish(pos_er_sum_msg);
        cal_er_msg.data = cal_er;
        cal_er_pub.publish(cal_er_msg);
        cal_er_sum_msg.data = cal_er_sum;
        cal_er_sum_pub.publish(cal_er_sum_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}