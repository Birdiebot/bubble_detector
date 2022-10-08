/*
 * @Author: Ligcox
 * @Date: 2022-02-13 21:09:28
 * @FilePath: /bubble_bringup/home/nvidia/Desktop/bubble/src/bubble_detector/bubble_rune/src/cv_base_node.cpp
 * @LastEditors: HarryWen
 * @LastEditTime: 2022-07-11 16:34:23
 * License: GNU General Public License v3.0. See LICENSE file in root directory.
 * Copyright (c) 2022 Birdiebot R&D Department
 * Shanghai University Of Engineering Science. All Rights Reserved
 */
#include "cv_base_node.hpp"

VisualRune::VisualRune(const rclcpp::NodeOptions &node_options) : Node("VisualRune", "/", node_options)
{
    parameter_init();

    image_sub_ = image_transport::create_subscription(this, "/raw_image", std::bind(&VisualRune::process, this, std::placeholders::_1), "raw");
    // manifold_ctrl_sub = this->create_subscription<std_msgs::msg::Int8>("/status/manifold_ctrl", 10, std::bind(&VisualNode::mode_callback, this, std::placeholders::_1));
    game_mode_sub = this->create_subscription<std_msgs::msg::Int8>("/status/game_mode", 10, std::bind(&VisualRune::game_mode_callback, this, std::placeholders::_1));
    rune_pub = this->create_publisher<bboxes_ex_msgs::msg::BoundingPolygonBoxes2D>("/cv/rune", 5);
}

VisualRune::~VisualRune()
{
}

void VisualRune::process(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    try
    {
        cv_bridge::CvImageConstPtr rawmsg = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat frame = rawmsg->image;
        if (manifold_ctrl == MANIFOLD_CTRL_RUNE)
        {
            findRune(frame, rawmsg->header);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void VisualRune::findRune(cv::Mat &image, std_msgs::msg::Header image_header)
{

    cv::Mat channels[3];
    cv::Mat enhancedImg;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<bboxes_ex_msgs::msg::BoundingPolygonBox2D> msgList;

    cv::resize(image, image, cv::Size(640, 480));

    cv::Mat binByRGB = GrayFilter(image, 1, this->grayThres); // RGB滤波器

    // 由于在agx上使用getStructuringElement生成kernel会报错，故自己手写了kernel
    cv::Mat morph_rect_kernel = (cv::Mat_<double>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);    // MORPH_RECT
    cv::Mat morph_ellipse_kernel = (cv::Mat_<double>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0); // MORPH_ELLIPSE
    // cv::Mat kernel = cv::getStructuringElement(2, cv::Size(3, 3), cv::Point(-1, -1));

    cv::morphologyEx(binByRGB, enhancedImg, 3, morph_rect_kernel, cv::Point(-1, -1), this->morphologyexIter); // 闭操作
    cv::dilate(enhancedImg, enhancedImg, morph_rect_kernel, cv::Point(-1, -1), 1);                            // 膨胀操作（便于面积筛选）
    cv::imshow("enhancedImg", enhancedImg);
    cv::waitKey(1);

    cv::findContours(enhancedImg, contours, hierarchy, CV_RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.size() - 1 < 1000) // 防止空contours
    {

        std::vector<RUNESTRIP> stripList = findStrip(contours, this);
        if (stripList.size() != 0)
            degbugRect(image, stripList);

        auto armourMsg = bboxes_ex_msgs::msg::BoundingPolygonBox2D();
        armourMsg.class_id = "Rune";
        armourMsg.type = "armour";
        // std::vector<cv::Point> runeStripRoi = stripList[0].hammerStrip;
        for (auto point : stripList[0].hammerStrip)
        {
            geometry_msgs::msg::Point32 pointMsg;
            pointMsg.x = point.x;
            pointMsg.y = point.y;
            armourMsg.pose.points.push_back(pointMsg);
        }
        msgList.push_back(armourMsg);

        auto centerMsg = bboxes_ex_msgs::msg::BoundingPolygonBox2D();
        centerMsg.class_id = "Rune";
        centerMsg.type = "center";
        // std::vector<cv::Point> circleStripRoi = stripList[0].hammerStrip;
        for (auto point : stripList[0].circleStrip)
        {
            geometry_msgs::msg::Point32 pointMsg;
            pointMsg.x = point.x;
            pointMsg.y = point.y;
            centerMsg.pose.points.push_back(pointMsg);
        }
        msgList.push_back(centerMsg);
        auto outMsg = bboxes_ex_msgs::msg::BoundingPolygonBoxes2D();
        outMsg.image_header = image_header;
        outMsg.header.stamp = rclcpp::Clock().now();
        outMsg.header.frame_id = "gimbal_optical";
        outMsg.bounding_boxes = msgList;
        rune_pub->publish(outMsg);
    }
}

void VisualRune::game_mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    game_mode = int(msg->data);
}

void VisualRune::mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    manifold_ctrl = int(msg->data);
}

void VisualRune::parameter_init()
{
    /*********************************/
    // Filter params
    rcl_interfaces::msg::ParameterDescriptor iterDescriptor;
    iterDescriptor.integer_range.resize(1);
    auto &iter_point_range = iterDescriptor.integer_range.at(0);
    iter_point_range.from_value = 0;
    iter_point_range.to_value = 40;
    iter_point_range.step = 1;

    rcl_interfaces::msg::ParameterDescriptor hsvDescriptor;
    hsvDescriptor.integer_range.resize(1);
    auto &hsv_point_range = hsvDescriptor.integer_range.at(0);
    hsv_point_range.from_value = 0;
    hsv_point_range.to_value = 255;
    hsv_point_range.step = 1;

    this->declare_parameter<int>("grayThres", 255, hsvDescriptor);
    this->declare_parameter<int>("morphologyexIter", 20, iterDescriptor);

    grayThres = this->get_parameter("grayThres").as_int();
    morphologyexIter = this->get_parameter("morphologyexIter").as_int();

    /*********************************/
    // Rune detector params
    rcl_interfaces::msg::ParameterDescriptor stripAreaDescriptor; // 面积筛选
    stripAreaDescriptor.integer_range.resize(1);
    auto &strip_point_range = stripAreaDescriptor.integer_range.at(0);
    strip_point_range.from_value = 0;
    strip_point_range.to_value = 10000;
    strip_point_range.step = 1;
    this->declare_parameter<int>("lowerHammerStrip", 0, stripAreaDescriptor);
    this->declare_parameter<int>("upperHammerStrip", 0, stripAreaDescriptor);

    this->declare_parameter<int>("lowerCircleStrip", 0, stripAreaDescriptor);
    this->declare_parameter<int>("upperCircleStrip", 0, stripAreaDescriptor);

    lowerHammerStrip = this->get_parameter("lowerHammerStrip").as_int();
    upperHammerStrip = this->get_parameter("upperHammerStrip").as_int();

    lowerCircleStrip = this->get_parameter("lowerCircleStrip").as_int();
    upperCircleStrip = this->get_parameter("upperCircleStrip").as_int();

    /*********************************/
    this->declare_parameter<bool>("debug_mode", true);
    debug_mode = this->get_parameter("debug_mode").as_bool();

    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&VisualRune::parametersCallback, this, std::placeholders::_1));
}

void VisualRune::degbugRect(cv::Mat &image, std::vector<RUNESTRIP> lightstripList)
{
    try
    {
        static cv::Scalar color = cv::Scalar(255, 0, 255);
        static int thickness = 2;
        cv::putText(image, "ROI", cv::Point(0, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
        if (lightstripList.size() != 0)
        {
            for (auto item : lightstripList)
            {
                for (int r = 0; r < 4; r++)
                {
                    // cv::line(image, item.armourStrip[r], item.armourStrip[(r + 1) % 4], color, thickness, 8);
                    cv::line(image, item.circleStrip[r], item.circleStrip[(r + 1) % 4], color, thickness, 8);
                    cv::line(image, item.hammerStrip[r], item.hammerStrip[(r + 1) % 4], color, thickness, 8);
                }
            }
            cv::resize(image, image, cv::Size(1280, 960));
            cv::imshow("debug", image);
            cv::waitKey(1);
        }
    }
    catch (const cv::Exception &e)
    {
        cv::imshow("debug", image);
        cv::waitKey(1);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualRune>());
    rclcpp::shutdown();
    return 0;
}

rcl_interfaces::msg::SetParametersResult VisualRune::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
        if (param.get_name() == "grayThres")
        {
            grayThres = param.as_int();
        }
        else if (param.get_name() == "morphologyexIter")
        {
            morphologyexIter = param.as_int();
        }
        else if (param.get_name() == "debug_mode")
        {
            debug_mode = param.as_bool();
        }

        else if (param.get_name() == "lowerHammerStrip")
        {
            lowerHammerStrip = param.as_int();
        }
        else if (param.get_name() == "upperHammerStrip")
        {
            upperHammerStrip = param.as_int();
        }
        else if (param.get_name() == "lowerCircleStrip")
        {
            lowerCircleStrip = param.as_int();
        }
        else if (param.get_name() == "upperCircleStrip")
        {
            upperCircleStrip = param.as_int();
        }
    }
    return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(VisualRune)