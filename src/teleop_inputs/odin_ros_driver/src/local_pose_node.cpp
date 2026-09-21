/*
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
*/

#ifdef ROS2

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LocalPoseNode : public rclcpp::Node
{
public:
    LocalPoseNode()
        : Node("local_pose_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        odometry_topic_ = this->declare_parameter<std::string>("odometry_topic", "/odin1/odometry");
        target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
        local_pose_topic_ = this->declare_parameter<std::string>("local_pose_topic", "/odin1/local_pose");
        local_odometry_topic_ = this->declare_parameter<std::string>("local_odometry_topic", "/odin1/local_odometry");
        tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 0.05);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(local_pose_topic_, 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(local_odometry_topic_, 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&LocalPoseNode::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Local pose node: %s -> %s, publishing %s and %s",
            odometry_topic_.c_str(),
            target_frame_.c_str(),
            local_pose_topic_.c_str(),
            local_odometry_topic_.c_str());
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose_in;
        pose_in.header = msg->header;
        pose_in.pose = msg->pose.pose;

        if (pose_in.header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Received odometry with empty frame_id; cannot compute local pose");
            return;
        }

        geometry_msgs::msg::PoseStamped pose_out;
        if (pose_in.header.frame_id == target_frame_) {
            pose_out = pose_in;
        } else {
            try {
                const auto timeout = rclcpp::Duration::from_seconds(tf_timeout_sec_);
                const auto transform = tf_buffer_.lookupTransform(
                    target_frame_, pose_in.header.frame_id, pose_in.header.stamp, timeout);
                tf2::doTransform(pose_in, pose_out, transform);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Waiting for TF %s <- %s: %s",
                    target_frame_.c_str(), pose_in.header.frame_id.c_str(), ex.what());
                return;
            }
        }

        auto odom_out = *msg;
        odom_out.header = pose_out.header;
        odom_out.pose.pose = pose_out.pose;

        pose_pub_->publish(pose_out);
        odom_pub_->publish(odom_out);
    }

    std::string odometry_topic_;
    std::string target_frame_;
    std::string local_pose_topic_;
    std::string local_odometry_topic_;
    double tf_timeout_sec_{0.05};

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPoseNode>());
    rclcpp::shutdown();
    return 0;
}

#else

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LocalPoseNode
{
public:
    LocalPoseNode()
        : nh_("~"),
          tf_listener_(tf_buffer_)
    {
        nh_.param<std::string>("odometry_topic", odometry_topic_, "/odin1/odometry");
        nh_.param<std::string>("target_frame", target_frame_, "map");
        nh_.param<std::string>("local_pose_topic", local_pose_topic_, "/odin1/local_pose");
        nh_.param<std::string>("local_odometry_topic", local_odometry_topic_, "/odin1/local_odometry");
        nh_.param<double>("tf_timeout_sec", tf_timeout_sec_, 0.05);

        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(local_pose_topic_, 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(local_odometry_topic_, 10);
        odom_sub_ = nh_.subscribe(odometry_topic_, 50, &LocalPoseNode::odomCallback, this);

        ROS_INFO(
            "Local pose node: %s -> %s, publishing %s and %s",
            odometry_topic_.c_str(),
            target_frame_.c_str(),
            local_pose_topic_.c_str(),
            local_odometry_topic_.c_str());
    }

private:
    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose_in;
        pose_in.header = msg->header;
        pose_in.pose = msg->pose.pose;

        if (pose_in.header.frame_id.empty()) {
            ROS_WARN_THROTTLE(2.0, "Received odometry with empty frame_id; cannot compute local pose");
            return;
        }

        geometry_msgs::PoseStamped pose_out;
        if (pose_in.header.frame_id == target_frame_) {
            pose_out = pose_in;
        } else {
            try {
                const auto transform = tf_buffer_.lookupTransform(
                    target_frame_,
                    pose_in.header.frame_id,
                    pose_in.header.stamp,
                    ros::Duration(tf_timeout_sec_));
                tf2::doTransform(pose_in, pose_out, transform);
            } catch (const tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(
                    2.0,
                    "Waiting for TF %s <- %s: %s",
                    target_frame_.c_str(),
                    pose_in.header.frame_id.c_str(),
                    ex.what());
                return;
            }
        }

        nav_msgs::Odometry odom_out = *msg;
        odom_out.header = pose_out.header;
        odom_out.pose.pose = pose_out.pose;

        pose_pub_.publish(pose_out);
        odom_pub_.publish(odom_out);
    }

    ros::NodeHandle nh_;
    std::string odometry_topic_;
    std::string target_frame_;
    std::string local_pose_topic_;
    std::string local_odometry_topic_;
    double tf_timeout_sec_{0.05};

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber odom_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher odom_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_pose_node");
    LocalPoseNode node;
    ros::spin();
    return 0;
}

#endif
