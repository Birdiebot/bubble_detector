/*
 * @Author: Ligcox
 * @Date: 2022-02-13 21:09:28
 * @FilePath: /bubble/src/bubble_detector/bubble_visual_SJTU/src/cv_base_node.cpp
 * @LastEditors: Ligcox
 * @LastEditTime: 2022-07-04 16:26:21
 * License: GNU General Public License v3.0. See LICENSE file in root directory.
 * Copyright (c) 2022 Birdiebot R&D Department
 * Shanghai University Of Engineering Science. All Rights Reserved
 */
#include "cv_base_node.hpp"

VisualNode::VisualNode(const rclcpp::NodeOptions &node_options) : Node("VisualNode", "/", node_options)
{
    parameter_init();
    this->trtmodule.create_module(this->model_path);

    image_sub = image_transport::create_subscription(this, "/raw_image", std::bind(&VisualNode::process, this, std::placeholders::_1), "raw");
    manifold_ctrl_sub = this->create_subscription<std_msgs::msg::Int8>("/status/manifold_ctrl", 10, std::bind(&VisualNode::manifold_ctrl_callback, this, std::placeholders::_1));
    game_mode_sub = this->create_subscription<std_msgs::msg::Int8>("/status/game_mode", 10, std::bind(&VisualNode::game_mode_callback, this, std::placeholders::_1));
    armour_pub = this->create_publisher<bboxes_ex_msgs::msg::BoundingPolygonBoxes2D>("/cv/armour", 5);

    armour_point_test_pub = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/debug/armour_point", 5);
}

VisualNode::~VisualNode()
{
}

void VisualNode::process(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    try
    {
        cv_bridge::CvImageConstPtr rawmsg = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat frame = rawmsg->image;
        if (manifold_ctrl == MANIFOLD_CTRL_ARMOUR)
        {
            findArmour(frame, rawmsg->header);
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "error" << std::endl;
    }
}

void debug_armour(cv::Mat &frame, std::vector<bbox_t> detections)
{
    cv::Mat im2show = frame.clone();
    for (const auto &b : detections)
    {
        cv::line(im2show, b.pts[0], b.pts[1], {0, 0, 255}, 2);
        cv::line(im2show, b.pts[1], b.pts[2], {0, 255, 0}, 2);
        cv::line(im2show, b.pts[2], b.pts[3], {255, 0, 0}, 2);
        cv::line(im2show, b.pts[3], b.pts[0], {255, 255, 255}, 2);
        cv::putText(im2show, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255});
    }
    // cv::putText(im2show, fmt::format("fps={}", fps), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
    cv::imshow("INFER", im2show);
    cv::waitKey(1);
}

void VisualNode::findArmour(cv::Mat &frame, std_msgs::msg::Header image_header)
{
    std::vector<bbox_t> detections = this->trtmodule.doinfer(frame);
    if (debug_mode)
    {
        debug_armour(frame, detections);
    }
    std::vector<bboxes_ex_msgs::msg::BoundingPolygonBox2D> msgList;
    for (int i = 0; i < (int)detections.size(); i++)
    {
        auto armour = detections[i];
        if (armour.color_id == this->game_mode || armour.color_id == GAME_MODE_GRAY_COLOR)
        {
            std::cout << "find other color armour, color: " << armour.color_id << ", id: " << armour.tag_id << std::endl;
            continue;
        }

        auto msg = bboxes_ex_msgs::msg::BoundingPolygonBox2D();
        msg.class_id = "Armour";
        msg.type = std::to_string(armour.tag_id);
        for (int j = 0; j < 4; j++)
        {
            auto point = armour.pts[j];
            geometry_msgs::msg::Point32 pointMsg;
            pointMsg.x = point.x;
            pointMsg.y = point.y;
            msg.pose.points.push_back(pointMsg);
        }

        if (debug_mode)
        {
            auto point_msg = geometry_msgs::msg::PolygonStamped();
            point_msg.header.frame_id = "gimbal_optical";
            point_msg.header.stamp = rclcpp::Clock().now();
            point_msg.polygon = msg.pose;
            armour_point_test_pub->publish(point_msg);
        }
        msgList.push_back(msg);
    }

    auto outMsg = bboxes_ex_msgs::msg::BoundingPolygonBoxes2D();
    outMsg.header.stamp = rclcpp::Clock().now();
    outMsg.image_header = image_header;
    outMsg.header.frame_id = "gimbal_optical";
    outMsg.bounding_boxes = msgList;
    armour_pub->publish(outMsg);
}

void VisualNode::parameter_init()
{
    this->declare_parameter<std::string>("model_path", "/home/nv/Desktop/bubble/src/bubble_contrib/bubble_visual_SJTU/asset/autoaming_SJTU2021_base.onnx");
    this->declare_parameter<bool>("debug_mode", false);

    this->get_parameter("model_path", this->model_path);
    this->get_parameter("debug_mode", this->debug_mode);

    RCLCPP_INFO(this->get_logger(), "Set parameter model_path: '%s'", this->model_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Set debug_mode: '%d'", this->debug_mode);
}

void VisualNode::game_mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    game_mode = int(msg->data);
}

void VisualNode::manifold_ctrl_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    manifold_ctrl = int(msg->data);
}

rcl_interfaces::msg::SetParametersResult VisualNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualNode>());
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(VisualNode)
