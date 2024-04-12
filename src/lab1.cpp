#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>
#include <ctime>
#include <math.h>

using namespace std;

double pos_x = 0;
double pos_y = 0;
double or_x = 0;
double or_y = 0;
double or_z = 0;
double or_w = 1;


void pos_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pos_x = msg->pose.pose.position.x;
    pos_y = msg->pose.pose.position.y;
    or_x = msg->pose.pose.orientation.x;
    or_y = msg->pose.pose.orientation.y;
    or_z = msg->pose.pose.orientation.z;
    or_w = msg->pose.pose.orientation.w;
}


int main(int argc, char** argv) {
    string mode = "lo";
    double vl = 0.22631579 * 2;
    //double vl = 1;
    double vr = 0.12368421 * 2;
    //double vr = 1;
    double d = 2;
    double stan = 0;
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n = ros::NodeHandle("~");
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/key_vel", 1000);
    //ros::Publisher error_publisher = n.advertise<geometry_msgs::Pose>("/velocity_pose",1000);
    ros::Publisher cal_pose_pub = n.advertise<std_msgs::Float64MultiArray>("/est_val", 1000);
    ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/mobile_base_controller/odom", 1000, pos_callback);

    ros::Rate loop_rate(100);

    n.getParam("lab1/vl", vl);
    n.getParam("lab1/vr", vr);
    n.getParam("lab1/mode", mode);
    n.getParam("lab1/d", d);

    if (mode == "lt" || mode == "lo") {
        vr = abs(vr);
    }
    if (mode == "rt" || mode == "ro") {
        vr = abs(vr) * (-1);
    }

    cout << "Vl: " << vl << endl;
    cout << "Vr: " << vr << endl;
    cout << "Mode: " << mode << endl;
    cout << "d: " << d << endl;

    int i = 0;
    int count = 0;
    int counter = 0;
    int run = 1;

    geometry_msgs::Twist msg;
    std_msgs::Float64MultiArray cal_pose_msg;
    cal_pose_msg.data.resize(3);

    ros::Time start_time = ros::Time::now();
    ros::Duration last_time(0.0);

    double current_x = 0;
    double current_y = 0;
    double current_angle = 0;

    while (ros::ok()) {
        cout << stan << endl;
        loop_rate.sleep();
        if (mode == "rt" || mode == "lt") {
            if (counter == 4) {
                return 0;
            }
            if (i == 0) {
                i = 1;
                start_time = ros::Time::now();
                ros::Duration last_time1(0.0);
                last_time = last_time1;
            }
            if (stan == 0) {
                msg.linear.x = vl;
                msg.angular.z = 0;
                ros::Duration time_now = ros::Duration(ros::Time::now() - start_time);
                ros::Duration time_to_move = ros::Duration(d / vl);
                ros::Duration time_diff = ros::Duration(time_now - last_time);
                last_time = time_now;

                if (counter == 0) {
                    current_x += time_diff.toSec() * vl;
                }
                if (counter == 1) {
                    if (mode == "rt" || mode == "ro") {
                        current_y -= time_diff.toSec() * vl;
                    }
                    else {
                        current_y += time_diff.toSec() * vl;
                    }
                }
                if (counter == 2) {
                    current_x -= time_diff.toSec() * vl;
                }
                if (counter == 3) {
                    if (mode == "rt" || mode == "ro") {
                        current_y += time_diff.toSec() * vl;
                    }
                    else {
                        current_y -= time_diff.toSec() * vl;
                    }
                }

                cal_pose_msg.data[0] = current_x;
                cal_pose_msg.data[1] = current_y;
                cal_pose_msg.data[2] = current_angle;

                cout << "ile jedziemy " << time_now << endl;
                cout << "ile mamy jechac " << time_to_move << endl;
                cout << "X: " << current_x << " Y: " << current_y << " Kąt: " << current_angle << endl;

                if (time_now > time_to_move) {
                    stan = 1;
                    i = 0;
                }
            }
            else if (stan == 1) {
                msg.linear.x = 0;
                msg.angular.z = vr;
                ros::Duration time_now = ros::Duration(ros::Time::now() - start_time);
                ros::Duration time_to_move = ros::Duration(abs(double((M_PI / 2) / vr)));
                ros::Duration time_diff = ros::Duration(time_now - last_time);
                last_time = time_now;

                current_angle += time_diff.toSec() * vr;

                cal_pose_msg.data[0] = current_x;
                cal_pose_msg.data[1] = current_y;
                cal_pose_msg.data[2] = current_angle;

                cout << "current_angle: " << current_angle << endl;
                cout << "ile obracamy " << time_now << endl;
                cout << "ile mamy obracac " << time_to_move << endl;

                if (time_now > time_to_move) {
                    stan = 0;
                    i = 0;
                    start_time = ros::Time::now();
                    counter += 1;
                    if (counter % 2 == 0) {
                        current_angle = -1 * current_angle;
                    }
                }
            }
        }
        if (mode == "ro" || mode == "lo") {
            if (counter == 4) {
                return 0;
            }
            if (i == 0) {
                i = 1;
                start_time = ros::Time::now();
                ros::Duration last_time1(0.0);
                last_time = last_time1;
            }
            if (stan == 0) {
                ros::spinOnce();
                double start_x = pos_x;
                double start_y = pos_y;
                msg.linear.x = vl;
                msg.angular.z = 0;
                while (run == 1) {
                    chatter_pub.publish(msg);
                    double distance = sqrt(pow(pos_x - start_x, 2) + pow(pos_y - start_y, 2));

                    ros::Duration time_now = ros::Duration(ros::Time::now() - start_time);
                    ros::Duration time_to_move = ros::Duration(d / vl);
                    ros::Duration time_diff = ros::Duration(time_now - last_time);
                    last_time = time_now;

                    if (counter == 0) {
                        current_x += time_diff.toSec() * vl;
                    }
                    if (counter == 1) {
                        if (mode == "lt" || mode == "lo") {
                            current_y += time_diff.toSec() * vl;
                        }
                        else {
                            current_y -= time_diff.toSec() * vl;
                        }
                    }
                    if (counter == 2) {
                        current_x -= time_diff.toSec() * vl;
                    }
                    if (counter == 3) {
                        if (mode == "lt" || mode == "lo") {
                            current_y -= time_diff.toSec() * vl;
                        }
                        else {
                            current_y += time_diff.toSec() * vl;
                        }
                    }

                    cout << "distance: " << distance << "position: " << pos_x << "start: " << start_x << endl;
                    cout << "X: " << current_x << " Y: " << current_y << " Kąt: " << current_angle << endl;

                    cal_pose_msg.data[0] = current_x;
                    cal_pose_msg.data[1] = current_y;
                    cal_pose_msg.data[2] = current_angle;

                    cal_pose_pub.publish(cal_pose_msg);
                    ros::spinOnce();

                    if (distance > d) {
                        msg.linear.x = 0;
                        run = 0;
                        stan = 1;
                        i = 0;
                    }
                }
            }
            if (stan == 1) {
                cout << "obrot" << endl;
                ros::spinOnce();
                double start_th = atan2(2 * or_w * or_z, 1 - 2 * or_z * or_z);
                msg.linear.x = 0;
                msg.angular.z = vr;
                while (run == 0) {
                    chatter_pub.publish(msg);

                    double ang = atan2(2 * or_w * or_z, 1 - 2 * or_z * or_z) - start_th;
                    ang = abs(ang);

                    cout << "start angle: " << start_th << "angle difference: " << ang << endl;
                    ros::Duration time_now = ros::Duration(ros::Time::now() - start_time);
                    ros::Duration time_diff = ros::Duration(time_now - last_time);
                    last_time = time_now;

                    current_angle += time_diff.toSec() * vr;

                    cal_pose_msg.data[0] = current_x;
                    cal_pose_msg.data[1] = current_y;
                    cal_pose_msg.data[2] = current_angle;

                    cal_pose_pub.publish(cal_pose_msg);
                    ros::spinOnce();

                    if (ang >= M_PI / 2) {
                        msg.angular.z = 0;
                        run = 1;
                        i = 0;
                        stan = 0;
                        start_time = ros::Time::now();
                        counter += 1;
                        if (counter % 2 == 0) {
                            current_angle = -1 * current_angle;
                        }
                    }
                }
            }
        }

        cal_pose_pub.publish(cal_pose_msg);
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}