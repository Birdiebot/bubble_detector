/*
 * @Author: Ligcox
 * @Date: 2022-02-13 21:09:28
 * @FilePath: /bubble/src/bubble_detector/bubble_visual/src/cv_base_node.cpp
 * @LastEditors: Ligcox
 * @LastEditTime: 2022-07-02 16:49:00
 * License: GNU General Public License v3.0. See LICENSE file in root directory.
 * Copyright (c) 2022 Birdiebot R&D Department
 * Shanghai University Of Engineering Science. All Rights Reserved
 */
#include "cv_base_node.hpp"

VisualNode::VisualNode(const rclcpp::NodeOptions &node_options) : Node("VisualNode", "/", node_options)
{
    parameter_init();

    image_sub_ = image_transport::create_subscription(this, "/raw_image", std::bind(&VisualNode::process, this, std::placeholders::_1), "raw");
    manifold_ctrl_sub = this->create_subscription<std_msgs::msg::Int8>("/status/manifold_ctrl", 10, std::bind(&VisualNode::mode_callback, this, std::placeholders::_1));
    game_mode_sub = this->create_subscription<std_msgs::msg::Int8>("/status/game_mode", 10, std::bind(&VisualNode::game_mode_callback, this, std::placeholders::_1));

    armour_pub = this->create_publisher<bboxes_ex_msgs::msg::BoundingPolygonBoxes2D>("/cv/armour", 5);
    rune_pub = this->create_publisher<bboxes_ex_msgs::msg::BoundingPolygonBoxes2D>("/cv/rune", 5);

    // mul_channel_debug_pub = image_transport::create_publisher(this, "/debug/mul_channel_image");
    // sin_channel_debug_pub = image_transport::create_publisher(this, "/debug/sin_channel_image");
}

VisualNode::~VisualNode()
{
}

void VisualNode::process(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    try
    {

        // clock_t start,end;
        // double fps;
        // start=clock();
        cv_bridge::CvImageConstPtr rawmsg = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat frame = rawmsg->image;
        // for debug
        // std::cout << "pub::"<< std::to_string(seq_)<<"   time:"<< time(0) << std::endl;
        // seq_ += 1;
        // cv::imshow("processInput", frame);
        // cv::waitKey(1);
        manifold_ctrl = 1;

        switch (manifold_ctrl)
        {
        case 1:
            findArmour(frame, rawmsg->header);
            break;
        case 2:
            findRune(frame);
            break;
        default:
            break;
        }
        // end=clock();
        // double endtime=(double)(end-start)/CLOCKS_PER_SEC;
        // fps = 1/endtime;
        // std::cout<<"time:"<< fps << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << "";
    }
}

void VisualNode::debugMulChannelImg(std::vector<cv::Mat> channel_masks)
{
    if (debug_mode)
    {
        cv::Mat dstImage;
        cv::merge(channel_masks, dstImage);

        img_msg_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        img_msg_ptr->image = dstImage;
        sensor_msgs::msg::Image::SharedPtr img_msg;
        img_msg = img_msg_ptr->toImageMsg();

        img_msg->header.stamp = rclcpp::Clock().now();
        img_msg->header.frame_id = std::to_string(0);
        mul_channel_debug_pub.publish(img_msg);
    }
}

void VisualNode::debugSingleChannelImg(cv::Mat singleChannel)
{
    if (debug_mode)
    {

        cv::Mat finalImage;
        cv::Mat in[] = {singleChannel, singleChannel, singleChannel};
        cv::merge(in, 3, finalImage);

        img_msg_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        img_msg_ptr->image = finalImage;
        sensor_msgs::msg::Image::SharedPtr img_msg;
        img_msg = img_msg_ptr->toImageMsg();

        img_msg->header.stamp = rclcpp::Clock().now();
        img_msg->header.frame_id = std::to_string(0);
        sin_channel_debug_pub.publish(img_msg);
    }
}

void VisualNode::degbugRect(cv::Mat &image, std::vector<STRIP> lightstripList, std::vector<ARMOUR> armourList)
{
    static cv::Scalar color = cv::Scalar(255, 0, 255);
    static int thickness = 2;
    cv::putText(image, "ROI", cv::Point(0, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    for (auto item : lightstripList)
    {
        cv::circle(image, item.ROI_RECT.center, 0, color, thickness);
        for (int r = 0; r < 4; r++)
        {
            cv::line(image, item.ROI_BOX[r], item.ROI_BOX[(r + 1) % 4], color, thickness, 8);
        }
    }
    for (auto item : armourList)
    {
        cv::circle(image, item.ROI_RECT.center, 0, color, thickness);
        for (int r = 0; r < 4; r++)
        {
            cv::line(image, item.ROI_BOX[r], item.ROI_BOX[(r + 1) % 4], color, thickness, 8);
        }
    }
    cv::resize(image, image, cv::Size(640, 480));
    cv::imshow("debug", image);
    cv::waitKey(1);
}

void VisualNode::degbugRect2(cv::Mat &image, std::vector<RUNESTRIP> lightstripList)
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
            cv::resize(image, image, cv::Size(640, 480));
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

void VisualNode::findArmour(cv::Mat &image, std_msgs::msg::Header image_header)
{
    cv::Mat hsv_img;
    cv::Mat final_mask;

    cv::cvtColor(image, hsv_img, cv::COLOR_BGR2HSV);
    // cv::Mat binByHSV = HSVFilter(hsv_img, this);                      // HSV滤波器
    // cv::Mat binByRGB = GrayFilter(image, game_mode, this->grayThres); // RGB滤波器

    cv::Mat binByRGB = GrayFilter(image, 0, this->grayThres); // RGB滤波器

    // VisualNode::debugSingleChannelImg(binByRGB);

    // cv::bitwise_and(binByRGB, binByHSV, final_mask);
    cv::Mat enhancedImg = enhanceContour(binByRGB, this->morphologyexIter); //图像增强
    // cv::imshow("enhancedImg", enhancedImg);
    // cv::waitKey(1);
    std::vector<STRIP> lightstripList = detectLightStrip(enhancedImg, this);
    std::vector<ARMOUR> armourList = detectArmour(lightstripList, this);
    degbugRect(enhancedImg, lightstripList, armourList);

    std::vector<bboxes_ex_msgs::msg::BoundingPolygonBox2D> msgList;

    for (int i = 0; i < (int)lightstripList.size(); i++)
    {
        auto lightstrip = lightstripList[i];
        auto msg = bboxes_ex_msgs::msg::BoundingPolygonBox2D();
        msg.class_id = "Strip";
        msg.type = "Strip";
        auto strip_roi = lightstrip.ROI_BOX;
        for (int j = 0; j < 4; j++)
        {
            auto point = strip_roi[j];
            geometry_msgs::msg::Point32 pointMsg;
            pointMsg.x = point.x;
            pointMsg.y = point.y;
            msg.pose.points.push_back(pointMsg);
        }

        msgList.push_back(msg);
    }

    for (int i = 0; i < (int)armourList.size(); i++)
    {
        auto armour = armourList[i];
        auto msg = bboxes_ex_msgs::msg::BoundingPolygonBox2D();
        msg.class_id = "Armour";
        auto armour_roi = armour.ROI_BOX;
        for (int j = 0; j < 4; j++)
        {
            auto point = armour_roi[j];
            geometry_msgs::msg::Point32 pointMsg;
            pointMsg.x = point.x;
            pointMsg.y = point.y;
            msg.pose.points.push_back(pointMsg);
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

void VisualNode::findRune(cv::Mat &image)
{

    cv::Mat channels[3];
    cv::Mat enhancedImg;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    cv::resize(image, image, cv::Size(640, 480));

    cv::Mat finalMask = cv::Mat(image.rows, image.cols, CV_8UC1, cv::Scalar(255, 255, 255));
    std::vector<bboxes_ex_msgs::msg::BoundingPolygonBox2D> msgList;
    cv::split(image, channels);

    // cv::Mat hsvImg;
    // cv::cvtColor(image, hsvImg, cv::COLOR_BGR2HSV);
    // cv::Mat binByHSV = HSVFilter(image, this);                              // HSV滤波器
    cv::Mat binByRGB = GrayFilter(image, 1, this->grayThres); // RGB滤波器
    // cv::bitwise_and(binByRGB, binByHSV, finalMask);
    cv::Mat kernel = cv::getStructuringElement(2, cv::Size(3, 3), cv::Point(-1, -1));        // 0为MORPH_RECT
    cv::morphologyEx(binByRGB, enhancedImg, 3, kernel, cv::Point(-1, -1), this->morphologyexIter); // 闭操作
    cv::imshow("enhancedImg", enhancedImg);
    cv::waitKey(1);

    cv::findContours(enhancedImg, contours, hierarchy, CV_RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
   
    if (contours.size() - 1 < 1000) // 防止空contours
    {
        std::vector<RUNESTRIP> stripList = findStrip(contours, this);
        degbugRect2(image, stripList);
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
        outMsg.header.stamp = rclcpp::Clock().now();
        outMsg.header.frame_id = "gimbal_optical";
        outMsg.bounding_boxes = msgList;
        armour_pub->publish(outMsg);
    }
}

void VisualNode::game_mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    game_mode = int(msg->data);
}

void VisualNode::mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    manifold_ctrl = int(msg->data);
}

void VisualNode::parameter_init()
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

    // this->declare_parameter<int>("hueMin", 57, hsvDescriptor);
    // this->declare_parameter<int>("saturationMin", 40, hsvDescriptor);
    // this->declare_parameter<int>("valueMin", 230, hsvDescriptor);
    // this->declare_parameter<int>("hueMax", 200, hsvDescriptor);
    // this->declare_parameter<int>("saturationMax", 235, hsvDescriptor);
    // this->declare_parameter<int>("valueMax", 255, hsvDescriptor);
    this->declare_parameter<int>("grayThres", 255, hsvDescriptor);
    this->declare_parameter<int>("morphologyexIter", 20, iterDescriptor);

    // hueMin = this->get_parameter("hueMin").as_int();
    // saturationMin = this->get_parameter("saturationMin").as_int();
    // valueMin = this->get_parameter("valueMin").as_int();
    // hueMax = this->get_parameter("hueMax").as_int();
    // saturationMax = this->get_parameter("saturationMax").as_int();
    // valueMax = this->get_parameter("valueMax").as_int();
    grayThres = this->get_parameter("grayThres").as_int();
    morphologyexIter = this->get_parameter("morphologyexIter").as_int();

    /*********************************/
    // Armour detector params
    rcl_interfaces::msg::ParameterDescriptor cntDescriptor;
    cntDescriptor.floating_point_range.resize(1);
    auto &cnt_point_range = cntDescriptor.floating_point_range.at(0);
    cnt_point_range.from_value = 0;
    cnt_point_range.to_value = 100;
    cnt_point_range.step = 0.01;

    this->declare_parameter<double>("lightStripSizeDiffRatio", 1.8, cntDescriptor);
    this->declare_parameter<double>("lightStripAngleDiff", 5, cntDescriptor);
    this->declare_parameter<double>("lightStripDisplacementDiffMin", 1, cntDescriptor);
    this->declare_parameter<double>("lightStripDisplacementDiffMax", 3, cntDescriptor);

    lightStripSizeDiffRatio = this->get_parameter("lightStripSizeDiffRatio").as_double();
    lightStripAngleDiff = this->get_parameter("lightStripAngleDiff").as_double();
    lightStripDisplacementDiffMin = this->get_parameter("lightStripDisplacementDiffMin").as_double();
    lightStripDisplacementDiffMax = this->get_parameter("lightStripDisplacementDiffMax").as_double();
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
    this->declare_parameter<int>("lowerArmourStrip", 0, stripAreaDescriptor);
    this->declare_parameter<int>("upperArmourStrip", 0, stripAreaDescriptor);
    this->declare_parameter<int>("lowerCircleStrip", 0, stripAreaDescriptor);
    this->declare_parameter<int>("upperCircleStrip", 0, stripAreaDescriptor);
    this->declare_parameter<double>("contourAreaThreshold", 0, stripAreaDescriptor);

    lowerHammerStrip = this->get_parameter("lowerHammerStrip").as_int();
    upperHammerStrip = this->get_parameter("upperHammerStrip").as_int();
    lowerArmourStrip = this->get_parameter("lowerArmourStrip").as_int();
    upperArmourStrip = this->get_parameter("upperArmourStrip").as_int();
    lowerCircleStrip = this->get_parameter("lowerCircleStrip").as_int();
    upperCircleStrip = this->get_parameter("upperCircleStrip").as_int();
    contourAreaThreshold = this->get_parameter("contourAreaThreshold").as_double();

    /*********************************/
    this->declare_parameter<bool>("debug_mode", true);
    debug_mode = this->get_parameter("debug_mode").as_bool();

    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&VisualNode::parametersCallback, this, std::placeholders::_1));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualNode>());
    rclcpp::shutdown();
    return 0;
}

rcl_interfaces::msg::SetParametersResult VisualNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
        // if (param.get_name() == "hueMin")
        // {
        //     hueMin = param.as_int();
        // }
        // else if (param.get_name() == "saturationMin")
        // {
        //     saturationMin = param.as_int();
        // }
        // else if (param.get_name() == "valueMin")
        // {
        //     valueMin = param.as_int();
        // }
        // else if (param.get_name() == "hueMax")
        // {
        //     hueMax = param.as_int();
        // }
        // else if (param.get_name() == "saturationMax")
        // {
        //     saturationMax = param.as_int();
        // }
        // else if (param.get_name() == "valueMax")
        // {
        //     valueMax = param.as_int();
        // }

        if (param.get_name() == "grayThres")
        {
            grayThres = param.as_int();
        }
        else if (param.get_name() == "morphologyexIter")
        {
            morphologyexIter = param.as_int();
        }
        else if (param.get_name() == "contourAreaThreshold")
        {
            contourAreaThreshold = param.as_double();
        }
        else if (param.get_name() == "lightStripSizeDiffRatio")
        {
            lightStripSizeDiffRatio = param.as_double();
        }
        else if (param.get_name() == "lightStripAngleDiff")
        {
            lightStripAngleDiff = param.as_double();
        }
        else if (param.get_name() == "lightStripDisplacementDiffMin")
        {
            lightStripDisplacementDiffMin = param.as_double();
        }
        else if (param.get_name() == "lightStripDisplacementDiffMax")
        {
            lightStripDisplacementDiffMax = param.as_double();
        }
        else if (param.get_name() == "debug_mode")
        {
            debug_mode = param.as_bool();
        }
    }
    return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(VisualNode)