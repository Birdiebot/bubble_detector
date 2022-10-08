/*
 * @Author: Ligcox
 * @Date: 2022-01-27 06:41:01
 * @FilePath: /bubble/src/bubble_detector/bubble_rune/include/cv_base_node.hpp
 * @LastEditors: HarryWen
 * @LastEditTime: 2022-07-08 13:18:32
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "bboxes_ex_msgs/msg/bounding_polygon_box2_d.hpp"
#include "bboxes_ex_msgs/msg/bounding_polygon_boxes2_d.hpp"

#include "filter.hpp"
#include "detector.hpp"

#define GAME_MODE_BLUE_COLOR 0
#define GAME_MODE_RED_COLOR 1
#define GAME_MODE_GRAY_COLOR 2

#define MANIFOLD_CTRL_ARMOUR 1
#define MANIFOLD_CTRL_RUNE 2

using namespace std::chrono_literals;

const int MAX_SIZE = 30;
std::deque<cv::Mat> seqQueue;

class VisualRune : public rclcpp::Node
{
public:
    explicit VisualRune(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
    ~VisualRune();
    
private:
    cv::Mat frame;
    cv_bridge::CvImageConstPtr rawmsg;
    rclcpp::TimerBase::SharedPtr timer_;

private:
    void parameter_init();
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    cv_bridge::CvImagePtr img_msg_ptr = std::make_shared<cv_bridge::CvImage>();

private:
    template <typename T>
    void drawRoi(cv::Mat &image, T *roi_list, cv::Scalar color = (255, 0, 255), int thickness = 10);
    void debugMulChannelImg(std::vector<cv::Mat> channel_masks);
    void debugSingleChannelImg(cv::Mat singleChannel);
    void degbugRect(cv::Mat &image, std::vector<RUNESTRIP> lightstripList);

protected:
    void process(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void pushStack(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void mode_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void game_mode_callback(const std_msgs::msg::Int8::SharedPtr msg);

protected:
    cv_bridge::CvImagePtr imgmsg_ptr = std::make_shared<cv_bridge::CvImage>();
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr manifold_ctrl_sub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr game_mode_sub;
    rclcpp::Publisher<bboxes_ex_msgs::msg::BoundingPolygonBoxes2D>::SharedPtr rune_pub;

protected:
    image_transport::Publisher mul_channel_debug_pub;
    image_transport::Publisher sin_channel_debug_pub;

protected:
    void findRune(cv::Mat &image, std_msgs::msg::Header image_header);

public:
    int seq_ = 0; 
    /*********************************/
    // declare debug variable
    bool debug_mode;
    int game_mode;
    int manifold_ctrl = 1;
    /*********************************/
    // declare HSV params
    int hueMin;
    int saturationMin;
    int valueMin;
    int hueMax;
    int saturationMax;
    int valueMax;
    /*********************************/
    // declare BGR params
    int grayThres;
    int morphologyexIter;
    /*********************************/
    // declare rune params
    int lowerHammerStrip;
    int upperHammerStrip;
    int lowerArmourStrip;
    int upperArmourStrip;
    int lowerCircleStrip;
    int upperCircleStrip;
    /*********************************/
};