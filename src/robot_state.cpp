#include "ros/ros.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TransformStamped.h"

#include "driverV2.h"

#include "base/wheel.h"

ros::Publisher imuPublisher, odomPublisher, velocityPublisher;

tf2_ros::TransformBroadcaster *tfBroadcaster;

base::wheel wheelTargetSpeed;
base::wheel wheelControls;

typedef enum publisherStatus
{
    S_INIT,
    S_READY
} publisherStatus;

double wheelbase, wheelRight, wheelLeft, baseHeight, wheelDiameter, distancePerEncoderRevolution, gearRatio;
double laserX, laserY, laserZ, laserTheta, imuX, imuY, imuZ, imuTheta, kp, ki, kd, pidInputLimit;
double ppr, minControl, minSpeed;
bool flipEncoderLeft, flipEncoderRight, flipControlLeft, flipControlRight, highInertia;
library::IMU accBias, gyroBias;
publisherStatus status;

void initialize(library::Driver2Sensor sensor)
{
    static int count = 0;
    int maxCount = 100;
    static ros::Time t, prevT;

    static double averageRate = 0;

    if (prevT.toSec() == 0)
    {
        ROS_INFO("Measuring sensor bias");
        prevT = ros::Time::now();
        return;
    }
    t = ros::Time::now();
    double dt = (t - prevT).toSec();

    averageRate += dt;

    accBias.x += sensor.accelerometer.x;
    accBias.y += sensor.accelerometer.y;
    accBias.z += sensor.accelerometer.z;
    gyroBias.x += sensor.gyroscope.x;
    gyroBias.y += sensor.gyroscope.y;
    gyroBias.z += sensor.gyroscope.z;

    count++;
    if (count == maxCount)
    {
        averageRate /= count;

        accBias.x /= count;
        accBias.y /= count;
        accBias.z /= count;
        // Add gravity for z direction
        accBias.z += 9.80665;

        gyroBias.x /= count;
        gyroBias.y /= count;
        gyroBias.z /= count;

        ROS_INFO("Measurement complete");
        ROS_INFO("Average rate (hz): %f", 1.0 / averageRate);
        ROS_INFO("Sensor bias");
        ROS_INFO("Accelerometer:");
        ROS_INFO("X: %f, Y: %f, Z: %f", accBias.x, accBias.y, accBias.z);
        ROS_INFO("Gyroscope:");
        ROS_INFO("X: %f, Y: %f, Z: %f", gyroBias.x, gyroBias.y, gyroBias.z);

        status = S_READY;
    }
    prevT = t;
}

// Only for high inertia load
void wheelControlFix(double *control, double minControl, double target, double minSpeed, double currentSpeed)
{
    if (abs(target) < minSpeed)
    {
        if (currentSpeed < minSpeed)
            *control = 0;
    }
    else if (abs(*control) < minControl && target != 0)
    {
        if (*control < 0)
            *control = -minControl;
        else
            *control = minControl;
    }
}

void publish(library::Driver2Sensor sensor)
{
    static ros::Time prevT;
    static library::Encoder prevEncoder;
    static base::wheel prevError;
    static base::wheel eI;

    // static int data = 0;

    if (prevT.toSec() == 0)
    {
        ROS_INFO("Starting broadcast");
        prevT = ros::Time::now();
        prevEncoder = sensor.encoder;
        prevError.left = 0;
        prevError.right = 0;
        eI.left = 0;
        eI.right = 0;
        return;
    }

    // if (data++ == 0)
    // {
    //     return;
    // }
    // else
    // {
    //     data = 0;
    // }

    ros::Time t = ros::Time::now();
    // Publish IMU Data
    sensor_msgs::Imu imu;

    imu.header.stamp = t;
    imu.header.frame_id = "base_imu";

    imu.linear_acceleration.x = sensor.accelerometer.x - accBias.x;
    imu.linear_acceleration.y = sensor.accelerometer.y - accBias.y;
    imu.linear_acceleration.z = sensor.accelerometer.z - accBias.z;

    imu.angular_velocity.x = sensor.gyroscope.x - gyroBias.x;
    imu.angular_velocity.y = sensor.gyroscope.y - gyroBias.y;
    imu.angular_velocity.z = sensor.gyroscope.z - gyroBias.z;

    // ROS_INFO("%f, %f, %f | %f, %f, %f", sensor.accelerometer.x, sensor.accelerometer.y, sensor.accelerometer.z, sensor.gyroscope.x, sensor.gyroscope.y, sensor.gyroscope.z);
    imuPublisher.publish(imu);

    // odometry
    static double x = 0.0;
    static double y = 0.0;
    static double th = 0.0;

    // Caculate vx, vy and vth
    double distLeft = (sensor.encoder.left - prevEncoder.left) * distancePerEncoderRevolution;
    double distRight = (sensor.encoder.right - prevEncoder.right) * distancePerEncoderRevolution;
    double dt = (t - prevT).toSec();

    double vLeft = distLeft / dt;
    double vRight = distRight / dt;

    double vRx = (vRight + vLeft) / 2;
    double vth = (vRight - vLeft) / wheelbase; // d denotes the distance between both wheels (track)

    double vWx = vRx * cos(th);
    double vWy = vRx * sin(th);

    double dx = vWx * dt;
    double dy = vWy * dt;
    double dth = vth * dt;

    // double rightLeft = distRight - distLeft;
    // double a = (distLeft + distRight) * 0.5;
    // double fraction = rightLeft / wheelbase;
    // double vx = a * cos(th + (fraction / 2.0)) / dt;
    // double vy = a * sin(th + (fraction / 2.0)) / dt;
    // double vth = fraction / dt;

    // Calculate odom changes
    // double dx = (vx * cos(th) - vy * sin(th)) * dt;
    // double dy = (vx * sin(th) + vy * cos(th)) * dt;
    // double dth = vth * dt;

    // Control update

    double maxSpeed = 1; // Meter/s

    double eLeft = (wheelTargetSpeed.left - vLeft) / maxSpeed;
    double eRight = (wheelTargetSpeed.right - vRight) / maxSpeed;

    // integral directional reset
    if ((wheelTargetSpeed.left > 0 && eI.left < 0) || (wheelTargetSpeed.left < 0 && eI.left > 0))
        eI.left = 0;
    if ((wheelTargetSpeed.right > 0 && eI.right < 0) || (wheelTargetSpeed.right < 0 && eI.right > 0))
        eI.right = 0;

    eI.left += eLeft * dt;
    eI.right += eRight * dt;
    wheelControls.left = kp * eLeft + ki * eI.left + kd * (eLeft - prevError.left) / dt;
    wheelControls.right = kp * eRight + ki * eI.right + kd * (eRight - prevError.right) / dt;

    double beforeFixLeft = wheelControls.left;
    double beforeFixRight = wheelControls.right;
    // Special settings to overcome high load inertia
    if (highInertia)
    {
        if (-minSpeed < wheelTargetSpeed.left && wheelTargetSpeed.left < minSpeed)
            eI.left = 0;
        else if (minSpeed < wheelTargetSpeed.left && eI.left < minControl)
            eI.left = minControl;
        else if (-minSpeed > wheelTargetSpeed.left && eI.left > -minControl)
            eI.left = -minControl;
        if (-minSpeed < wheelTargetSpeed.right && wheelTargetSpeed.right < minSpeed)
            eI.right = 0;
        else if (minSpeed < wheelTargetSpeed.right && eI.right < minControl)
            eI.right = minControl;
        else if (-minSpeed > wheelTargetSpeed.right && eI.right > -minControl)
            eI.right = -minControl;
        // wheelControlFix(&wheelControls.left, minControl, wheelTargetSpeed.left, minSpeed, vLeft);
        // wheelControlFix(&wheelControls.right, minControl, wheelTargetSpeed.right, minSpeed, vRight);
    }

    // ROS_INFO("%f %f %f %f %f %f %f %f %f %f", eLeft, eI.left, (eLeft - prevError.left) / dt, eRight, eI.right, (eRight - prevError.right) / dt,
    //          beforeFixLeft, beforeFixRight, wheelControls.left, wheelControls.right);

    // ROS_INFO("CTRL: %f, %f, %f, %f, %f, %f, %f, %f", wheelTargetSpeed.left, wheelTargetSpeed.right,
    //          wheelControls.left, wheelControls.right, vLeft, vRight, eLeft, eRight);
    // Update end

    // Integral Error Reset
    if (wheelTargetSpeed.left == 0 && vLeft == 0)
        eI.left = 0;

    if (wheelTargetSpeed.right == 0 && vRight == 0)
        eI.right = 0;

    x += dx;
    y += dy;
    th += dth;
    // ROS_INFO("encoder: %f %f %f %f %f", distLeft, dx, check, x, y);

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
    base::wheel vel;
    vel.left = vLeft;
    vel.right = vRight;
    vel.eLeft = eLeft;
    vel.eRight = eRight;
    vel.cLeft = wheelControls.left;
    vel.cRight = wheelControls.right;

    velocityPublisher.publish(vel);

    // ROS_INFO("Data: %f, %f, %f, %f, %f, %f", vx, vy, vth, x, y, th);

    // Updated value for next cycle
    prevT = t;
    prevEncoder = sensor.encoder;
}

void onData(library::Driver2Sensor sensor)
{

    // Flip
    sensor.encoder.left *= flipEncoderLeft ? -1 : 1;
    sensor.encoder.right *= flipEncoderRight ? -1 : 1;
    // Scalling encoder back to ppr of encoder used
    sensor.encoder.left = sensor.encoder.left * 2000 / ppr;
    sensor.encoder.right = sensor.encoder.right * 2000 / ppr;

    switch (status)
    {
    case S_INIT:
        initialize(sensor);
        break;
    case S_READY:
        publish(sensor);
        break;
    }

    // ROS_INFO("%f, %f, %f | %f, %f, %f | %f, %f, %f",
    //          odom.accelerometer.x, odom.accelerometer.y, odom.accelerometer.z, // in m/s^2
    //          odom.gyroscope.x, odom.gyroscope.y, odom.gyroscope.z,             // in rad/s
    //          odom.magnetometer.x, odom.magnetometer.y, odom.magnetometer.z);   // in milliGauss
}

void onVelocity(geometry_msgs::Twist velocity)
{
    // Use m/s for control
    /*

    ROS_INFO("CTRL: %f, %f, %f, %f, %f, %f, %f, %f", wheelTargetSpeed.left, wheelTargetSpeed.right,
             wheelControls.left, wheelControls.right, vLeft, vRight, eLeft, eRight);*/
    // ROS_INFO("input %f, %f",  velocity.linear.x, velocity.angular.z );
    double rightTarget = (velocity.angular.z * wheelbase) / 2 + velocity.linear.x;
    double leftTarget = velocity.linear.x * 2 - rightTarget;
    // ROS_INFO("before %f, %f", leftTarget, rightTarget);
    if (rightTarget > pidInputLimit)
        rightTarget = pidInputLimit;
    if (leftTarget > pidInputLimit)
        leftTarget = pidInputLimit;
    if (rightTarget < -pidInputLimit)
        rightTarget = -pidInputLimit;
    if (leftTarget < -pidInputLimit)
        leftTarget = -pidInputLimit;

    wheelTargetSpeed.right = rightTarget;
    wheelTargetSpeed.left = leftTarget;
    // ROS_INFO("after  %f, %f",  leftTarget, rightTarget);
}

int main(int argc, char **argv)
{
    double bound = 0;
    ros::init(argc, argv, "robot_state");

    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.param<double>("wheelbase", wheelbase, 0.60);
    nh.param<double>("wheelRight", wheelRight, 0.30);
    nh.param<double>("wheelLeft", wheelLeft, 0.30);
    nh.param<double>("baseHeight", baseHeight, 0.10);
    nh.param<double>("wheelDiameter", wheelDiameter, 0.1);

    nh.param<double>("gearRatio", gearRatio, 1);
    nh.param<double>("pulsePerRevolution", ppr, 2000);

    nh.param<bool>("flipEncoderLeft", flipEncoderLeft, false);
    nh.param<bool>("flipEncoderRight", flipEncoderRight, false);

    nh.param<bool>("highInertia", highInertia, false);
    nh.param<double>("minimumSpeed", minSpeed, 0.175);
    nh.param<double>("minimumControl", minControl, 0.35);
    nh.param<bool>("flipControlLeft", flipControlLeft, false);
    nh.param<bool>("flipControlRight", flipControlRight, false);

    nh.param<double>("kp", kp, 0.01);
    nh.param<double>("ki", ki, 0.01);
    nh.param<double>("kd", kd, 0.01);
    nh.param<double>("pidInputLimit", pidInputLimit, 1);
    nh.param<double>("pidControllerOutputLimit", bound, 1);

    distancePerEncoderRevolution = wheelDiameter * M_PI * gearRatio;

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

    // Display information
    ROS_INFO("Robot Information: ");
    ROS_INFO("Sensor Port: %s", port.c_str());
    ROS_INFO("Wheel Base: %f | Base Height: %f", wheelbase, baseHeight);
    ROS_INFO("Left Wheel: %f | Right Wheel: %f", wheelLeft, wheelRight);
    ROS_INFO("Laser:");
    ROS_INFO("X: %f, Y: %f, Z: %f, Theta: %f", laserX, laserY, laserZ, laserTheta);
    ROS_INFO("IMU:");
    ROS_INFO("X: %f, Y: %f, Z: %f, Theta: %f", imuX, imuY, imuZ, imuTheta);
    ROS_INFO("Controls:");
    ROS_INFO("Kp: %f, Ki: %f, Kd: %f, Output limit: %f", kp, ki, kd, bound);
    ROS_INFO("Wheel Control Flip:");
    ROS_INFO("Left: %s Right: %s", flipControlLeft ? "true" : "false", flipControlRight ? "true" : "false");
    ROS_INFO("Encoder Pulse Per Revolution: %f", ppr);
    ROS_INFO("Distance Per Encoder Revolution: %f", distancePerEncoderRevolution);
    ROS_INFO("Wheel Encoder Flip:");
    ROS_INFO("Left: %s Right: %s", flipEncoderLeft ? "true" : "false", flipEncoderRight ? "true" : "false");
    ROS_INFO("High Inertia: %s", highInertia ? "true" : "false");
    if (highInertia)
        ROS_INFO("Minimum Speed: %f, Minimum Control: %f", minSpeed, minControl);

    // Display end

    library::DriverV2 d = library::DriverV2(port);
    d.onData = onData;

    tf2_ros::StaticTransformBroadcaster staticBroadcaster;
    imuPublisher = n.advertise<sensor_msgs::Imu>("imu", 50);
    odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 50);
    velocityPublisher = n.advertise<base::wheel>("vel", 50);
    tfBroadcaster = new tf2_ros::TransformBroadcaster();

    geometry_msgs::TransformStamped base_link, base_laser, base_imu;

    ros::Time t = ros::Time::now();

    base_link.header.stamp = t;
    base_link.header.frame_id = "base_footprint";
    base_link.child_frame_id = "base_link";
    base_link.transform.translation.z = baseHeight;
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

    // Imu

    // Static link complete
    ros::Rate r(10.0);

    // Listen to wheel power control

    ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 50, onVelocity);

    d.start();
    int aliveCount = 0;
    double testLeft = 0;
    double testRight = 0;
    while (n.ok())
    {
        ros::spinOnce();
        // Keep driver alive at rate of 10Hz
        if (++aliveCount == 10)
        {
            d.keepAlive();
            aliveCount = 0;
        }
        // Update control velocity here

        // Scale control
        double left = wheelControls.left > bound ? bound : wheelControls.left < -bound ? -bound : wheelControls.left;
        double right = wheelControls.right > bound ? bound : wheelControls.right < -bound ? -bound : wheelControls.right;

        // Flip Control parameters
        left *= flipControlLeft ? -1 : 1;
        right *= flipControlRight ? -1 : 1;

        d.UpdateControls(left, right);
        // d.UpdateControls(0.11, 0.11);

        //
        // testRight += 0.1;
        // if (testRight > 1)
        //     testRight = 0;
        // testLeft = testRight;
        // d.UpdateControls(1,1);

        r.sleep();
    }
    // p.disconnect();
    return 0;
}
