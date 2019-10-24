#include "ros/ros.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TransformStamped.h"

#include "robot_state.h"

#include "driverV2.h"

#include "base/wheel.h"

#include "pid.h"

ros::Publisher imuPublisher, odomPublisher, velocityPublisher;

tf2_ros::TransformBroadcaster *tfBroadcaster;

void onData(library::Driver2Sensor sensor)
{
    static ros::Time prevT;

    if (prevT.toSec() == 0)
    {
        ROS_INFO("Starting broadcast");
        prevT = ros::Time::now();
        return;
    }
    ros::Time t = ros::Time::now();
    // double dt = (t - prevT).toSec();
    // dt handle by interrupt in device
    double dt = 0.05;
    double dl = distancePerPulseLeft * sensor.encoder.left;
    double dr = distancePerPulseRight * sensor.encoder.right;

    // odometry
    static double x = 0.0;
    static double y = 0.0;
    static double th = 0.0;

    // Caculate vx, vy and vth
    double vLeft = dl / dt;
    double vRight = dr / dt;

    double vRx = (vRight + vLeft) / 2;
    double vth = (vRight - vLeft) / robotBaseWidth;

    double vWx = vRx * cos(th);
    double vWy = vRx * sin(th);

    double dx = vWx * dt;
    double dy = vWy * dt;
    double dth = vth * dt;

    // Control wheel

    // Speed PID

    pidLeft->set(wheelTargetSpeed.left);
    pidRight->set(wheelTargetSpeed.right);

    wheelControls.left = pidLeft->compute(vLeft, dt);
    wheelControls.right = pidRight->compute(vRight, dt);
    if (pidDebug)
        ROS_INFO("%f | % 1.4f, % 1.4f, % 1.4f, % 1.4f | % 1.4f, % 1.4f, % 1.4f, % 1.4f", dt, wheelTargetSpeed.left, vLeft, pidLeft->error, wheelControls.left, wheelTargetSpeed.right, vRight, pidRight->error, wheelControls.right);

    // Publish data

    // Publish velocity data
    base::wheel vel;
    vel.left = vLeft;
    vel.right = vRight;
    vel.eLeft = pidLeft->error;
    vel.eRight = pidRight->error;
    vel.cLeft = wheelControls.left;
    vel.cRight = wheelControls.right;

    velocityPublisher.publish(vel);
    // Publish IMU Data
    sensor_msgs::Imu imu;

    imu.header.stamp = t;
    imu.header.frame_id = "base_imu";

    imu.linear_acceleration.x = sensor.accelerometer.x;
    imu.linear_acceleration.y = sensor.accelerometer.y;
    imu.linear_acceleration.z = sensor.accelerometer.z;

    imu.angular_velocity.x = sensor.gyroscope.x;
    imu.angular_velocity.y = sensor.gyroscope.y;
    imu.angular_velocity.z = sensor.gyroscope.z;

    imuPublisher.publish(imu);

    // Update location
    x += dx;
    y += dy;
    th += dth;

    tf2::Quaternion q;
    q.setRPY(0, 0, th);

    // tf publish
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = t;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_footprint";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tfBroadcaster->sendTransform(transformStamped);

    // Odom Publish
    nav_msgs::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    //set the velocity
    odom.twist.twist.linear.x = vWx;
    odom.twist.twist.linear.y = vWy;
    odom.twist.twist.angular.z = vth;

    odomPublisher.publish(odom);

    // Update for next cycle
    prevT = t;
}

void onVelocity(geometry_msgs::Twist velocity)
{
    // Use m/s for control
    double angularV = velocity.angular.z > maxAngularVelocity ? maxAngularVelocity : velocity.angular.z < -maxAngularVelocity ? -maxAngularVelocity : velocity.angular.z;
    double linearV = velocity.linear.x > maxLinearVelocity ? maxLinearVelocity : velocity.linear.x < -maxLinearVelocity ? -maxLinearVelocity : velocity.linear.x;
    double rightTarget = (angularV * robotBaseWidth) / 2 + linearV;
    double leftTarget = linearV * 2 - rightTarget;
    wheelTargetSpeed.right = rightTarget;
    wheelTargetSpeed.left = leftTarget;
}

int main(int argc, char **argv)
{
    double bound = 0;
    int ppr;
    double baseHeight;
    bool flipEncoderLeft, flipEncoderRight, flipControlLeft, flipControlRight;
    ros::init(argc, argv, "robot_state");

    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.param<double>("baseWidth", robotBaseWidth, 0.50);
    nh.param<double>("baseHeight", baseHeight, 0.10);
    nh.param<double>("wheelDiameter", robotWheelDiameter, 0.1);

    nh.param<int>("pulsePerRevolution", ppr, 500);

    nh.param<double>("maxLinearVelocity", maxLinearVelocity, 0.2);
    nh.param<double>("maxAngularVelocity", maxAngularVelocity, 0.2);

    nh.param<bool>("flipEncoderLeft", flipEncoderLeft, false);
    nh.param<bool>("flipEncoderRight", flipEncoderRight, false);
    nh.param<bool>("flipControlLeft", flipControlLeft, false);
    nh.param<bool>("flipControlRight", flipControlRight, false);

    nh.param<bool>("pidDebug", pidDebug, false);

    double kp = 0.01, ki = 0, kd = 0, acceptedError = 0;
    nh.param<double>("kp", kp, 0.01);
    nh.param<double>("ki", ki, 0);
    nh.param<double>("kd", kd, 0);
    nh.param<double>("pidAcceptedError", acceptedError, 0);
    Pid _pidLeft(kp, ki, kd, acceptedError);
    Pid _pidRight(kp, ki, kd, acceptedError);

    pidLeft = &_pidLeft;
    pidRight = &_pidRight;

    // nh.param<double>("pidInputLimit", pidInputLimit, 1);
    nh.param<double>("driverOutputLimit", bound, 0.2);

    // distancePerEncoderRevolution = wheelDiameter * M_PI * gearRatio;
    double laserX, laserY, laserZ, laserTheta, imuX, imuY, imuZ, imuTheta;

    nh.param<double>("laserX", laserX, 0);
    nh.param<double>("laserY", laserY, 0);
    nh.param<double>("laserZ", laserZ, 0);
    nh.param<double>("laserTheta", laserTheta, 0);

    nh.param<double>("imuX", imuX, 0);
    nh.param<double>("imuY", imuY, 0);
    nh.param<double>("imuZ", imuZ, 0);
    nh.param<double>("imuTheta", imuTheta, 0);

    string port;
    nh.param<string>("port", port, "/dev/ttyUSB0");

    // Calculate required information
    distancePerPulseLeft = robotWheelDiameter * M_PI / ppr;
    distancePerPulseRight = robotWheelDiameter * M_PI / ppr;
    if (flipEncoderLeft)
        distancePerPulseLeft *= -1;
    if (flipEncoderRight)
        distancePerPulseRight *= -1;

    // Display information
    ROS_INFO("Robot Information:");
    ROS_INFO("Sensor Port: %s", port.c_str());
    ROS_INFO("Distance between wheel: %f | Wheel Diameter: %f", robotBaseWidth, robotWheelDiameter);
    ROS_INFO("Height: %f", baseHeight);
    ROS_INFO("Laser location from middle of wheel:");
    ROS_INFO("X: %f, Y: %f, Z: %f, Theta: %f", laserX, laserY, laserZ, laserTheta);
    ROS_INFO("IMU location from middle of wheel:");
    ROS_INFO("X: %f, Y: %f, Z: %f, Theta: %f", imuX, imuY, imuZ, imuTheta);
    ROS_INFO("Wheel Encoder Flip:");
    ROS_INFO("Left: %s Right: %s", flipEncoderLeft ? "true" : "false", flipEncoderRight ? "true" : "false");
    ROS_INFO("Wheel Control Flip:");
    ROS_INFO("Left: %s Right: %s", flipControlLeft ? "true" : "false", flipControlRight ? "true" : "false");
    ROS_INFO("Distance per pulse left : %f", distancePerPulseLeft);
    ROS_INFO("Distance per pulse right: %f", distancePerPulseRight);

    // Display end

    library::DriverV2 d = library::DriverV2(port);
    d.onData = onData;

    tf2_ros::StaticTransformBroadcaster staticBroadcaster;
    imuPublisher = n.advertise<sensor_msgs::Imu>("imu_raw", 50);
    odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 50);
    velocityPublisher = n.advertise<base::wheel>("vel", 50);

    tfBroadcaster = new tf2_ros::TransformBroadcaster();
    geometry_msgs::TransformStamped base_link, base_laser, base_imu;

    ros::Time t = ros::Time::now();

    base_link.header.stamp = t;
    base_link.header.frame_id = "base_footprint";
    base_link.child_frame_id = "base_link";
    base_link.transform.translation.z = 0;
    base_link.transform.rotation.w = 1;

    base_laser.header.stamp = t;
    base_laser.header.frame_id = "base_link";
    base_laser.child_frame_id = "base_laser";
    base_laser.transform.translation.x = laserX;
    base_laser.transform.translation.y = laserY;
    base_laser.transform.translation.z = laserZ;

    tf2::Quaternion laserQ;
    laserQ.setRPY(0, 0, laserTheta);

    base_laser.transform.rotation.x = laserQ.x();
    base_laser.transform.rotation.y = laserQ.y();
    base_laser.transform.rotation.z = laserQ.z();
    base_laser.transform.rotation.w = laserQ.w();

    base_imu.header.stamp = t;
    base_imu.header.frame_id = "base_link";
    base_imu.child_frame_id = "base_imu";
    base_imu.transform.translation.x = imuX;
    base_imu.transform.translation.y = imuY;
    base_imu.transform.translation.z = imuZ;

    tf2::Quaternion imuQ;
    imuQ.setRPY(0, 0, imuTheta);

    base_imu.transform.rotation.x = imuQ.x();
    base_imu.transform.rotation.y = imuQ.y();
    base_imu.transform.rotation.z = imuQ.z();
    base_imu.transform.rotation.w = imuQ.w();

    staticBroadcaster.sendTransform(base_link);
    staticBroadcaster.sendTransform(base_laser);
    staticBroadcaster.sendTransform(base_imu);

    // Static link complete
    ros::Rate r(20.0);

    ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 50, onVelocity);

    d.start();
    int aliveCount = 0;
    while (n.ok())
    {
        ros::spinOnce();
        // Keep driver alive at rate of 10Hz
        if (++aliveCount == 5)
        {
            d.keepAlive();
            aliveCount = 0;
        }
        // Update control velocity here

        // Scale control
        double left = wheelControls.left > bound ? bound : wheelControls.left < -bound ? -bound : wheelControls.left;
        double right = wheelControls.right > bound ? bound : wheelControls.right < -bound ? -bound : wheelControls.right;

        // double left = wheelControls.left;
        // double right = wheelControls.right;

        // Flip Control parameters
        left *= flipControlLeft ? -1 : 1;
        right *= flipControlRight ? -1 : 1;

        d.UpdateControls(left, right);

        r.sleep();
    }
    return 0;
}
