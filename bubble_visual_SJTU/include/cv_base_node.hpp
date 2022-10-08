/*
 * @Author: Ligcox
 * @Date: 2022-01-27 06:41:01
 * @FilePath: /bubble/src/bubble_detector/bubble_visual_SJTU/include/cv_base_node.hpp
 * @LastEditors: Ligcox
 * @LastEditTime: 2022-07-03 17:26:45
 * License: GNU General Public License v3.0. See LICENSE file in root directory.
 * Copyright (c) 2022 Birdiebot R&D Department
 * Shanghai University Of Engineering Science. All Rights Reserved
 */
#pragma once
#include <queue>
#include <iostream>
#include <exception>
#include <chrono>
#include <functional>
#include <memory>
#include <stdlib.h>
#include <ctime>
#include <string>
#include <sys/time.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "bboxes_ex_msgs/msg/bounding_polygon_box2_d.hpp"
#include "bboxes_ex_msgs/msg/bounding_polygon_box2_d.hpp"
#include "bboxes_ex_msgs/msg/bounding_polygon_boxes2_d.hpp"

#include "TRTModule.hpp"

#define GAME_MODE_BLUE_COLOR 0
#define GAME_MODE_RED_COLOR 1
#define GAME_MODE_GRAY_COLOR 2

#define MANIFOLD_CTRL_NONE 1
#define MANIFOLD_CTRL_ARMOUR 1
#define MANIFOLD_CTRL_RUNE 2

using namespace std::chrono_literals;

const int MAX_SIZE = 30;
std::deque<cv::Mat> seqQueue;

class VisualNode : public rclcpp::Node
{
public:
    explicit VisualNode(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
    ~VisualNode();

protected:
    image_transport::Subscriber image_sub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr manifold_ctrl_sub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr game_mode_sub;

    rclcpp::Publisher<bboxes_ex_msgs::msg::BoundingPolygonBoxes2D>::SharedPtr armour_pub;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr armour_point_test_pub;

    void process(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void manifold_ctrl_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void game_mode_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void findArmour(cv::Mat &image, std_msgs::msg::Header image_header);

private:
    // image process
    TRTModule trtmodule;
    std::string model_path;
    cv::Mat frame;
    cv_bridge::CvImageConstPtr rawmsg;

    // declare debug variable
    bool debug_mode;
    int game_mode = GAME_MODE_GRAY_COLOR;
    int manifold_ctrl = MANIFOLD_CTRL_NONE;

    // time test
    struct timeval test_time;

    // parameter setting
    void parameter_init();
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
};