#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

/*
    Here are links to the referred documents:

    Derivation of the equations:
    https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf 

    How to use the equations:
    https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf 

    Excel spreadsheet swerve calculator:
    https://www.chiefdelphi.com/uploads/default/original/3X/0/2/02eeda134c9afd5cc57c28b554ca8c20632467ab.xls 
*/

////////////////////////////// Variables & Function Prototype - main //////////////////////////////
ros::Subscriber   subOdom, subCmdVel;
ros::Publisher    pubLegLF, pubLegRF, pubLegLH, pubLegRH;
ros::Publisher    pubWheelLF, pubWheelRF, pubWheelLH, pubWheelRH;

std_msgs::Float64 lfWheelMsg, rfWheelMsg, lhWheelMsg, rhWheelMsg;
std_msgs::Float64 lfLegMsg, rfLegMsg, lhLegMsg, rhLegMsg;

int main(int argc, char **argv);

////////////////////////////// Variables & Function Prototype - cmdVelCallback //////////////////////////////
double _velX, _velY, _angZ;
double A, B, C, D;
double lfVel, rfVel, lhVel, rhVel; 
double lfAng, rfAng, lhAng, rhAng;
double lfAngOffset =  M_PI/2.0 * 0;
double rfAngOffset =  M_PI/2.0 * 0;
double lhAngOffset =  M_PI/2.0 * 0;
double rhAngOffset =  M_PI/2.0 * 0;

double bodyLength   	= 1.20; // metre(m)
double bodyWidth	= 0.80; // metre(m)
double wheelradius 	= 0.15; // metre(m)

void cmdVelCallback(const geometry_msgs::Twist& msg);

////////////////////////////// Variables & Function Prototype - odomCallback //////////////////////////////

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

////////////////////////////// Function Definition //////////////////////////////
void cmdVelCallback(const geometry_msgs::Twist& msg) {
    // ROS_INFO("Subscriber velocities:"<<" linearX="<<msg.linear.x<<" linearY="<<msg.linear.y<<" angular="<<msg.angular.z);
    ROS_INFO("Velocity I/P : linX=[%lf]; linY=[%lf]; angZ=[%lf];", msg.linear.x, msg.linear.y, msg.angular.z);

    _velX = -msg.linear.y;
    _velY = +msg.linear.x;
    _angZ = -msg.angular.z;

    A = _velX - (_angZ * bodyLength/2.0);
    B = _velX + (_angZ * bodyLength/2.0);
    C = _velY - (_angZ * bodyWidth/2.0);
    D = _velY + (_angZ * bodyWidth/2.0);

    lfVel = sqrt(pow(B, 2) + pow(D, 2));
    lfAng = atan2(B, D);

    rfVel = sqrt(pow(B, 2) + pow(C, 2));
    rfAng = atan2(B, C);

    lhVel = sqrt(pow(A, 2) + pow(D, 2));
    lhAng = atan2(A, D);

    rhVel = sqrt(pow(A, 2) + pow(C, 2));
    rhAng = atan2(A, C);

    if ((lfAng <= -M_PI/2.0) || (lfAng >= M_PI/2.0)) {
	ROS_INFO("Finding new angles LF");
        lfVel = -lfVel;
        lfAng = ( lfAng < -M_PI/2.0 ? M_PI + lfAng : ( M_PI/2.0 >= lfAng ?  : lfAng - M_PI ));
    }
    if ((rfAng <= -M_PI/2.0) || (rfAng >= M_PI/2.0)) {
        ROS_INFO("Finding new angles RF");
        rfVel = -rfVel;
        rfAng = ( rfAng < -M_PI/2.0 ? rfAng + M_PI : ( M_PI/2.0 >= rfAng ?  : rfAng - M_PI ));
    }
    if ((lhAng <= -M_PI/2.0) || (lhAng >= M_PI/2.0)) {
        ROS_INFO("Finding new angles LH");
        lhVel = -lhVel;
        lhAng = ( lhAng < -M_PI/2.0 ? lhAng + M_PI : ( M_PI/2.0 >= lhAng ?  : lhAng - M_PI ));
    }
    if ((rhAng <= -M_PI/2.0) || (rhAng >= M_PI/2.0)) {
        ROS_INFO("Finding new angles RH");
        rhVel = -rhVel;
        rhAng = ( rhAng < -M_PI/2.0 ? rhAng + M_PI : ( M_PI/2.0 >= rhAng ?  : rhAng - M_PI ));
    }

    // rad per min = metre per second / (2*pi*r)rad.m * 60sec
    lfWheelMsg.data = lfVel / (2.0 * M_PI * wheelradius) * 60.0 / 10.0 ;// / (2 * M_PI * wheelradius); // m/sec / rad.m * sec = rad/min
    rfWheelMsg.data = rfVel / (2.0 * M_PI * wheelradius) * 60.0 / 10.0 ;// / (2 * M_PI * wheelradius); // m/sec / rad.m * sec = rad/min
    lhWheelMsg.data = lhVel / (2.0 * M_PI * wheelradius) * 60.0 / 10.0 ;// / (2 * M_PI * wheelradius); // m/sec / rad.m * sec = rad/min
    rhWheelMsg.data = rhVel / (2.0 * M_PI * wheelradius) * 60.0 / 10.0 ;// / (2 * M_PI * wheelradius); // m/sec / rad.m * sec = rad/min
    lfLegMsg.data = -lfAng + lfAngOffset; // radians
    rfLegMsg.data = -rfAng + rfAngOffset; // radians
    lhLegMsg.data = -lhAng + lhAngOffset; // radians
    rhLegMsg.data = -rhAng + rhAngOffset; // radians


    pubWheelLF.publish(lfWheelMsg);
    pubWheelRF.publish(rfWheelMsg);
    pubWheelLH.publish(lhWheelMsg);
    pubWheelRH.publish(rhWheelMsg);
    pubLegLF.publish(lfLegMsg);
    pubLegRF.publish(rfLegMsg);
    pubLegLH.publish(lhLegMsg);
    pubLegRH.publish(rhLegMsg);

    ROS_INFO("Velocities\t-> lf: [%lf], rf: [%lf], lh: [%lf], rh: [%lf]\n", lfVel, rfVel, lhVel, rhVel);
    ROS_INFO("Angles\t-> lf: [%lf], rf: [%lf], lh: [%lf], rh: [%lf]\n", lfAng, rfAng, lhAng, rhAng);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "swerve_drive_plugin");
    ros::NodeHandle n;

    ros::Rate loop_rate(100);

    subCmdVel   = n.subscribe("/swervebot/cmd_vel", 1000, cmdVelCallback);
    // subOdom     = n.subscribe("/swervebot/odom", 1000, odomCallback);

    pubLegLF    = n.advertise<std_msgs::Float64>("/swervebot/leg_lf_pos_controller/command", 1000);
    pubLegRF    = n.advertise<std_msgs::Float64>("/swervebot/leg_rf_pos_controller/command", 1000);
    pubLegLH    = n.advertise<std_msgs::Float64>("/swervebot/leg_lh_pos_controller/command", 1000);
    pubLegRH    = n.advertise<std_msgs::Float64>("/swervebot/leg_rh_pos_controller/command", 1000);

    pubWheelLF  = n.advertise<std_msgs::Float64>("/swervebot/wheel_lf_vel_controller/command", 1000);
    pubWheelRF  = n.advertise<std_msgs::Float64>("/swervebot/wheel_rf_vel_controller/command", 1000);
    pubWheelLH  = n.advertise<std_msgs::Float64>("/swervebot/wheel_lh_vel_controller/command", 1000);
    pubWheelRH  = n.advertise<std_msgs::Float64>("/swervebot/wheel_rh_vel_controller/command", 1000);

    int count = 0;
    while (ros::ok()) {
        /*
        std_msgs::Float64 msg;

        msg.data = 0.2 * count;

        ROS_INFO("%f", msg.data);

        pubLegLF.publish(msg);

        ++count;
        */

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
