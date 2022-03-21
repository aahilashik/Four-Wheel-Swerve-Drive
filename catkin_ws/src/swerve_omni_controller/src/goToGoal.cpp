#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <string>
#include <stack>
#include <tf/tf.h>

using namespace std;

class goToGoalController {
private:
  ros::Publisher pubVel;
  ros::Subscriber subOdom;

  geometry_msgs::Pose currentPose;
  geometry_msgs::Twist velMsg;

  const double PI = 3.14159265359;

  const double KpX = 0.2;
  const double KpZ = 0.5;

  const double minLinVel = -0.2;
  const double maxLinVel = +1.2;
  const double minAngVel = -0.5;
  const double maxAngVel = +0.5;

  vector<vector<double>> wayPoints = {};

public:
  goToGoalController(string nodeName, int argc, char **argv, string velTopic = "/cmd_vel", string odomTopic = "/odom") {
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;

    pubVel  = nh.advertise<geometry_msgs::Twist>(velTopic, 1000);
    subOdom = nh.subscribe(odomTopic, 1000, &goToGoalController::odomCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg) {
    tf::Quaternion quaternion(
        odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y,
        odomMsg->pose.pose.orientation.z, odomMsg->pose.pose.orientation.w);

    currentPose.position.x = odomMsg->pose.pose.position.x;
    currentPose.position.y = odomMsg->pose.pose.position.y;
    currentPose.orientation.z = tf::getYaw(quaternion);
  }

  double getRestrictedVelocity(double velocity, double minVelocity,
                               double maxVelocity) {
    if (velocity < minVelocity)
      return minVelocity;
    else if (velocity > maxVelocity)
      return maxVelocity;
    else
      return velocity;
  }

  void addWayPoint(double x, double y, double theta) {
    wayPoints.push_back({x, y, theta});
  }
  double getEuclideanDistance(geometry_msgs::Pose currentPose,
                              geometry_msgs::Pose goalPose) {
    return sqrt(pow((goalPose.position.x - currentPose.position.x), 2) +
                pow((goalPose.position.y - currentPose.position.y), 2));
  }

  void startMove(double distanceTolerance = 0.4) {
    ros::Rate loopRate(10);
    geometry_msgs::Pose goalPose;

    ROS_INFO("Started to Move");
    for (int i = 0; i < wayPoints.size(); i++) {

      goalPose.position.x = wayPoints[i][0];
      goalPose.position.y = wayPoints[i][1];
      goalPose.orientation.z = wayPoints[i][2];

      do {
        double errorX = getEuclideanDistance(currentPose, goalPose);
        double errorZ = atan2(goalPose.position.y - currentPose.position.y,
                              goalPose.position.x - currentPose.position.x) -
                        currentPose.orientation.z;

        errorZ = (((PI > errorZ) && (errorZ > -PI)) ? errorZ : ((errorZ < -PI) ? errorZ + (2 * PI) : errorZ - (2 * PI)));

        velMsg.linear.x = KpX * errorX;
        velMsg.angular.z = KpZ * errorZ; // -

        velMsg.linear.x  = getRestrictedVelocity(velMsg.linear.x, minLinVel, maxLinVel);
        velMsg.angular.z = getRestrictedVelocity(velMsg.angular.z, minAngVel, maxAngVel);

        pubVel.publish(velMsg);

        ROS_INFO("\nError:\n\tDistance: %0.4lf\t| Angle: "
                 "%0.4lf\nControl:\n\tLin.Vel: "
                 "%0.4lf\t| Ang.Vel: %0.4lf",
                 errorX, errorZ, velMsg.linear.x, velMsg.angular.z);

        ros::spinOnce();
        loopRate.sleep();
      } while (
                (getEuclideanDistance(currentPose, goalPose) > distanceTolerance) &&
                (ros::ok())   );

      ROS_INFO("Reached %dth Goal!!", i + 1);
    }

    ROS_INFO("End Goal Reached!!");

    velMsg.linear.x = 0;
    velMsg.angular.z = 0;
    pubVel.publish(velMsg);

    loopRate.sleep();
  }
};

int main(int argc, char **argv) {
  goToGoalController botNavCtrl = goToGoalController("swervePoseController", argc, argv, "/swervebot/cmd_vel", "/swervebot/odom");
  double goals[4][3] = {{4, 4, 0}, {-4, 4, 0}, {4, -4, 0}, {-4, -4, 0}};

  for (int i = 0; i < 4; i++) {
    botNavCtrl.addWayPoint(goals[i][0], goals[i][1], goals[i][2]);
  }
  botNavCtrl.startMove();
  return 0;
}
